#![feature(asm)]
#![feature(naked_functions)]
#![feature(default_alloc_error_handler)]
#![no_main]
#![no_std]

extern crate alloc;

use panic_halt as _;

use core::{mem, cell::RefCell};
use alloc::string::String;
use lazy_static::lazy_static;
use cortex_m::{asm, interrupt, peripheral::syst::SystClkSource, Peripherals};
use cortex_m_semihosting::{hprint, hprintln};
use alloc_cortex_m::CortexMHeap;
use stm32f4::stm32f407;

/// This is a wrapper that provides an immutable reference to its
/// contained value only when we are within an interrupt-free context.
/// Its original name is confusing since it does not really provide any
/// mutual exclusion, we rename it as exception cell (ExcpCell).
use cortex_m::interrupt::Mutex as ExcpCell;

mod boot;
mod uart;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
/**
 *  SRAM Layout:
 *           High Address
 *  +-----------------------------+  - 0x2002_0000  Pointed by MSP (Main Stack Pointer)
 *  |     Task Kernel Stack 0     |
 *  +-----------------------------+  - 0x2001_f000  Pointed by MSP
 *  |     Task Kernel Stack 1     |
 *  +-----------------------------+  - 0x2001_e000  Pointed by MSP
 *  |     Task Kernel Stack 2     |
 *  +-----------------------------+  - 0x2001_d000  Pointed by MSP
 *  |     Task Kernel Stack 3     |
 *  +-----------------------------+  - 0x2001_c000  Pointed by PSP (Process Stack Pointer)
 *  |      Task User Stack 0      |
 *  +-----------------------------+  - 0x2001_b000  Pointed by PSP
 *  |      Task User Stack 1      |
 *  +-----------------------------+  - 0x2001_a000  Pointed by PSP
 *  |      Task User Stack 2      |
 *  +-----------------------------+  - 0x2001_9000  Pointed by PSP
 *  |      Task User Stack 3      |
 *  +-----------------------------+  - 0x2001_8000
 *  |            Heap             |
 *  +-----------------------------+  - Heap Start
 *  |        DATA + .BSS          |
 *  +-----------------------------+  - 0x2000_0000
 *            Low Address
 */

#[repr(usize)]
enum StackBottom {
    TaskKernStack0 = 0x2002_0000,
    TaskKernStack1 = 0x2001_f000,
    TaskKernStack2 = 0x2001_e000,
    TaskKernStack3 = 0x2001_d000,
    TaskUserStack0 = 0x2001_c000,
    TaskUserStack1 = 0x2001_b000,
    TaskUserStack2 = 0x2001_a000,
    TaskUserStack3 = 0x2001_9000
}

/// Registers that should be saved upon context switch.
#[repr(C)]
struct KernelContext {
    lr: usize,   // return address to return from `context_switch()`
    msp: usize,  // the exception stack pointer of a task
    r4: usize,     // the registers below are callee-saved
    r5: usize,     // in extern "C" calling convention
    r6: usize,
    r7: usize,
    r8: usize,
    r9: usize,
    r10: usize,
    r11: usize
}

impl KernelContext {
    fn new() -> KernelContext {
        KernelContext {
            lr: 0,
            msp: 0,
            r4: 0,
            r5: 0,
            r6: 0,
            r7: 0,
            r8: 0,
            r9: get_r9(),
            r10: 0,
            r11: 0
        }
    }
}

/// When a new task starts, it should has an exception stack frame
/// structured as bellow. Note that this is pointed by MSP.
#[repr(C)]
struct TaskStartKernelStackFrame {
    id: usize,
    psp: usize,
    lr: usize
}

/// Hardware saved registers upon exception. Note that this is pointed by PSP.
#[repr(C)]
struct TrapFrame {
    r0: u32,
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    lr: usize,
    pc: usize,
    psr: u32
}

/// Possible states of a task. Currently, we have only tasks with
/// forever loop, so these two states suffice.
enum TaskState {
    Empty,   // empty task struct
    Sleep,   // sleeping and should not be scheduled
    Ready    // task struct is loaded with a context and can be scheduled
}

/// The structure representing a task.
struct Task {
    id: usize,                  // the id of this task
    user_stack_bottom: usize,   // the address of the user stack bottom
    kernel_stack_bottom: usize, // the address of the exception stack bottom
    state: TaskState,           // the task state
    kernel_ctxt: KernelContext
}

impl Task {
    fn new(id: usize, user_stack_bottom: usize, kernel_stack_bottom: usize) -> Task {
        Task {
            id,
            user_stack_bottom,
            kernel_stack_bottom,
            state: TaskState::Empty,
            kernel_ctxt: KernelContext::new()
        }
    }

    /// Load the task structure with a task to run by providing the entry
    /// function. Here we set up a dummy trap frame, so that if we perform an
    /// exception returning to this task, it will start executing from the
    /// entry function.
    fn load(&mut self, start: extern "C" fn()->!) {
        // Reserve space for a dummy trap frame in the user stack.
        let user_stack_top
            = self.user_stack_bottom - mem::size_of::<TrapFrame>();
        let tf = {
            let tf_ptr = user_stack_top as *mut TrapFrame;
            unsafe { &mut *tf_ptr }
        };

        // The only bit we need to set in the program state register (PSR)
        // is the Thumb bit (the 24th). Cortex-m4 always run in Thumb state,
        // so we must always set the Thumb bit.
        tf.psr = 0x01000000;

        // This is the actual return address of the exception handler. We set
        // it to the entry function of the new task.
        tf.pc = start as usize;

        // All the left registers can have arbitrary value.
        tf.lr = 0;
        tf.r12 = 0;
        tf.r3 = 0;
        tf.r2 = 0;
        tf.r1 = 0;
        tf.r0 = 0;

        // Reserve space for the start stack frame in the kernel stack.
        let kern_stack_top =
            self.kernel_stack_bottom - mem::size_of::<TaskStartKernelStackFrame>();
        let start_stack_frame = {
            let start_stack_frame_ptr
                = kern_stack_top as *mut TaskStartKernelStackFrame;
            unsafe { &mut *start_stack_frame_ptr }
        };

        // The field below will be loaded into LR register in `task_start()`.
        // This special "return address" will trigger an exception return when it
        // is loaded into PC. Specifically, this value indicates the CPU to perform
        // 1. Return to thread mode (from exception mode).
        // 2. Use PSP for return.
        // 3. FPU was not used. (So do not try to restore FP regs from the stack.)
        start_stack_frame.lr = 0xffff_fffd;

        // The field below will be loaded into PSP register when the task starts.
        // By setting PSP, the exception return procedure will use the dummy trap
        // frame we create above to restore user context, which essentially causes
        // the CPU to start running the user code entry function.
        start_stack_frame.psp = user_stack_top;

        // The field below will be passed as the argument to `task_init()`.
        start_stack_frame.id = self.id;

        // The field below will be loaded into MSP register upon context switch.
        // This makes sure each task runs with its own kernel stack.
        self.kernel_ctxt.msp = kern_stack_top;

        // The field below will be loaded into LR register in `context_switch()`.
        // This causes `context_switch()` to return to `task_start()` to start the
        // new task.
        self.kernel_ctxt.lr = task_start as usize;

        self.state = TaskState::Ready;
    }
}

/// The singleton containing all task structures in the system.
struct TaskList {
    tasks: [Task; 4],        // an array of task structures
    last_run: Option<usize>  // the index of the last run task
}

impl TaskList {
    /// Force the system to think that it is running the task of the given `task_id`
    /// and `state`. This function is unsafe because if we set the wrong `task_id`,
    /// we will use a wrong stack later on which breaks memory safety. This function
    /// is designed to use only during system initialization.
    unsafe fn override_running_task(&mut self, task_id: usize, state: TaskState) {
        self.last_run = Some(task_id);
        self.tasks[task_id].state = state;
    }
}

lazy_static! {
    static ref PERIPHERALS: ExcpCell<RefCell<Peripherals>> = {
        let p = cortex_m::Peripherals::take().unwrap();
        ExcpCell::new(RefCell::new(p))
    };

    static ref TASKLIST: ExcpCell<RefCell<TaskList>> = {
        let tasks = [
            Task::new(0, StackBottom::TaskUserStack0 as usize,
                      StackBottom::TaskKernStack0 as usize),
            Task::new(1, StackBottom::TaskUserStack1 as usize,
                      StackBottom::TaskKernStack1 as usize),
            Task::new(2, StackBottom::TaskUserStack2 as usize,
                      StackBottom::TaskKernStack2 as usize),
            Task::new(3, StackBottom::TaskUserStack3 as usize,
                      StackBottom::TaskKernStack3 as usize)
        ];
        ExcpCell::new(RefCell::new(TaskList {
            tasks,
            last_run: None
        }))
    };
}

/// The entry function of our OS. We are now running with main
/// stack pointer (MSP). MSP is also used by exception handlers.
/// To avoid storing normal execution context and exception handling
/// context in the same stack, we first switch to process stack
/// pointer (PSP). As such, normal and exception context will be
/// saved in two different stacks pointed by PSP and MSP, respectively.
extern "C" fn main() -> ! {
    switch_to_psp_then_init(
        StackBottom::TaskUserStack0 as usize, 
        StackBottom::TaskKernStack0 as usize
    )
}

#[naked]
extern "C" fn switch_to_psp_then_init(_user_stack: usize, _exep_stack: usize) -> ! {
    unsafe {
        asm!(
            "msr  psp, r0",       // update PSP value
            "mrs  r0, control",   // retrive CONTROL register value
            "orr  r0, r0, #2",    // set the bit indicating we want to use PSP
            "msr  control, r0",   // update CONTROL register
            "msr  msp, r1",       // update MSP value
            "b    {init}",        // jump to the initialization function
            init = sym init,
            options(noreturn)
        )
    }
}

static mut SHOULD_BE_0: u32 = 0;
static mut SHOULD_BE_42: u32 = 42;

/// Initialize our system. This function performs the following:
/// 1. Set up and enable the timer.
/// 2. Load two tasks of ID 1 and 2.
/// 3. Treat the current running thread as Task ID 0.
extern "C" fn init() -> ! {
    // An interrupt free scope.
    interrupt::free(|cs| {
        let heap_start = boot::heap_start() as usize;
        let heap_end = 0x2001_8000;
        let heap_size = heap_end - heap_start;
        unsafe { ALLOCATOR.init(heap_start, heap_size); }

        hprintln!("heap initialized at 0x{:x?}-0x{:x?}", heap_start, heap_end).unwrap();

        hprintln!("checking static variables...").unwrap();
        unsafe {
            hprintln!("SHOULD_BE_0 has value {}", SHOULD_BE_0).unwrap();
            hprintln!("SHOULD_BE_42 has value {}", SHOULD_BE_42).unwrap();
            SHOULD_BE_0 += 0xdead_beaf;
            hprintln!("SHOULD_BE_0 incremented by 0xdeadbeaf becomes 0x{:x}",
                      SHOULD_BE_0).unwrap();
            SHOULD_BE_42 -= 42;
            hprintln!("SHOULD_BE_42 decremented by 42 becomes {}",
                      SHOULD_BE_42).unwrap();
        }

        {
            let mut p = PERIPHERALS.borrow(cs).borrow_mut();
            let syst = &mut p.SYST;

            // Configures the system timer to trigger a SysTick exception every 10ms.
            // It has a default CPU clock of 16 MHz so we set the counter to 160_000.
            syst.set_clock_source(SystClkSource::Core);
            syst.set_reload(160_000);
            syst.clear_current();
            syst.enable_counter();
            syst.enable_interrupt();
        }

        {
            // Spawn two tasks. These tasks will be scheduled upon SysTick exception.
            let mut tasklist = TASKLIST.borrow(cs).borrow_mut();
            tasklist.tasks[1].load(loop_hello_uart);
            tasklist.tasks[2].load(loop_world_uart);

            unsafe { tasklist.override_running_task(0, TaskState::Sleep); }
        }
    });

    // initialize uart so we can use it
    uart::uart_init();

    // We will never return here after the spawned tasks (ID 1 and 2) are scheduled.
    loop {
        asm::wfi();
    }
}

/// Round robin schedule function. Prohibiting this function from being inlined
/// is an optimization. See `SysTick()` for details.
#[inline(never)]
extern "C" fn schedule() {
    let mut cur_task = 0;

    let switch = 
        interrupt::free(|cs| {
            let mut tasklist = TASKLIST.borrow(cs).borrow_mut();

            // The system should already be initialized if we are here. We are
            // sure that it is not None.
            cur_task = tasklist.last_run.unwrap();

            // The next task in round-robin order.
            let rr_next = cur_task + 1;

            // Pick the next task to run in a round robin fashion.
            let mut next_task = cur_task;
            for (i, task) in tasklist.tasks.iter().enumerate().skip(rr_next) {
                if let TaskState::Ready = task.state {
                    next_task = i;
                    break;
                }
            }
            if cur_task == next_task {
                for (i, task) in tasklist.tasks[0..rr_next].iter().enumerate() {
                    if let TaskState::Ready = task.state {
                        next_task = i;
                        break;
                    }
                }
            }

            // Return Some(from_ptr, to_ptr) if there is a task to be switched to.
            if cur_task < next_task {
                let (first_half, second_half) = tasklist.tasks.split_at_mut(next_task);
                return Some((&mut first_half[cur_task].kernel_ctxt as *mut KernelContext,
                             &second_half[0].kernel_ctxt as *const KernelContext));
            } else if cur_task > next_task {
                let (first_half, second_half) = tasklist.tasks.split_at_mut(cur_task);
                return Some((&mut second_half[0].kernel_ctxt as *mut KernelContext,
                             &first_half[next_task].kernel_ctxt as *const KernelContext));
            }

            // If we have no task other than the current running task to schedule,
            // return None.
            return None;
        });

    // Save the user stack pointer in the kernel stack.
    let psp = get_psp();

    // Perform context switch if there is another task to be scheduled.
    if let Some((from_ptr, to_ptr)) = switch {
        unsafe { context_switch(from_ptr, to_ptr); }
    }

    // Restore the user stack pointer from the kernel stack.
    set_psp(psp);

    interrupt::free(|cs| {
        let mut tasklist = TASKLIST.borrow(cs).borrow_mut();
        tasklist.last_run = Some(cur_task);
    });
}

/// Switch kernel context. This function is unsafe because the caller of this
/// function must make sure that the two pointer arguments are valid.
#[naked]
unsafe extern "C" fn context_switch(_src_ctxt: *mut KernelContext,
                                    _dst_ctxt: *const KernelContext) {
    asm!(
        "str   lr, [r0], #4",   // save LR of the src context
        "str   sp, [r0], #4",   // save MSP of the src context
        "stmia r0, {{r4-r11}}", // save R4-R11 of the src context
        "ldr   lr, [r1], #4",   // restore LR of the dst context
        "ldr   sp, [r1], #4",   // restore MSP of the dst context
        "ldmia r1, {{r4-r11}}", // restore R4-R11 of the dst context
        "bx    lr",             // resume executing in the dst context
        options(noreturn)       // supress compiler generated return instruction
    )
}

/// The entry function to start a task. This function should be run in kernel
/// mode. This function is unsafe because the caller must set up the correct
/// stack frame before jumping to this function. The required layout of the
/// stack frame is defined in `TaskStartKernelStackFrame`. The stack frame is
/// prepared in `Task::load()`.
/// Required stack frame:
///            High Address
///     +-----------------------+
///     |          ...          |
///     +-----------------------+
///     | Exception Return Addr |
///     +-----------------------+
///     |     New PSP Value     |
///     +-----------------------+
///     |      New Task ID      |
///     +-----------------------+  <-- MSP
///            Low Address
#[naked]
unsafe extern "C" fn task_start() -> ! {
    asm!(
        "pop   {{r0}}",      // perpare arguments for `task_init()`
        "bl    {task_init}", // run `task_init()`
        "pop   {{r0}}",      // fetch new PSP value
        "msr   psp, r0",     // set new PSP value
        "pop   {{pc}}",      // perform exception return
        task_init = sym task_init,
        options(noreturn)
    )
}

/// Perform task initialization. Currently it only sets the current running
/// task ID in the task list.
extern "C" fn task_init(task_id: usize) {
    interrupt::free(|cs| {
        let mut tasklist = TASKLIST.borrow(cs).borrow_mut();
        tasklist.last_run = Some(task_id);
    });
}

// Endlessly print "hello ".
extern "C" fn loop_hello_uart() -> ! {
    let s = String::from("hello,\n");
    loop {
        uart::uart_print_str(&s);
        // Wait for 1s (task CPU time) after every print.
        // The SysTick exception occurs every 10ms, so the loop below
        // for 100 times.
        for _ in 0..100 {
            asm::wfi(); // halt the CPU until interrupt
        }
    }
}

// Endlessly print "world!". We intentionally make it less symmetric
// to `loop_hello()` so that it may potentially reveal more bugs in
// context switch.
extern "C" fn loop_world_uart() -> ! {
    let s = String::from("world!\n");
    loop {
        uart::uart_print_str(&s);
        // Wait for 1s (task CPU time) after every print.
        // The SysTick exception occurs every 10ms, so the loop below
        // for 100 times.
        for _ in 0..100 {
            asm::wfi(); // halt the CPU until interrupt
        }
    }
}

fn set_psp(stack: usize) {
    unsafe { asm!(
        "msr psp, {}",
        in(reg) stack
    )}
}

fn get_psp() -> usize {
    let psp: usize;
    unsafe { asm!(
        "mrs {}, psp",
        out(reg) psp
    )}
    psp
}

fn get_r9() -> usize {
    let r9: usize;
    unsafe { asm!(
        "mov {}, r9",
        out(reg) r9
    )}
    r9
}

/// SysTick exception is generated every 10ms. We perform a context switch
/// every 100ms. We prohibit `schedule()` from being inlined, so that the
/// body of this function is small enough that using only R0-R3 as scratch
/// registers suffice. Hardware already saves R0-R3 for us upon interrupt
/// (see TrapFrame for details). We save other registers lazily only if
/// we need to perform a context switch (through `schedule()`).
#[export_name = "SysTick"]
pub unsafe extern "C" fn systick_handler() {
    static mut CNT: u32 = 0;

    // This is incremented every 10ms.
    CNT += 1;

    // Every 100ms we perform a context switch.
    if CNT == 10 {
        CNT = 0;
        schedule();
    }
}

#[export_name = "HardFault"]
pub unsafe extern "C" fn hardfault_handler(ef: &boot::ExceptionFrame) -> ! {
    hprintln!("HardFault occured, exception frame:").unwrap();
    hprintln!("{:?}", ef).unwrap();
    loop {}
}

#[export_name = "DefaultHandler"]
pub unsafe extern "C" fn default_handler(ex_num: i16) {
    hprintln!("Exception {} occured!", ex_num).unwrap();
}
