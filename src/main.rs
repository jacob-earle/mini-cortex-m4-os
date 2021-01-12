#![feature(asm)]
#![feature(naked_functions)]
#![no_main]
#![no_std]

use panic_halt as _;

use core::cell::RefCell;
use lazy_static::lazy_static;
use cortex_m::{asm, interrupt, peripheral::syst::SystClkSource, Peripherals};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{hprint, hprintln};

/// This is a wrapper that provides an immutable reference to its
/// contained value only when we are within an interrupt-free context.
/// Its original name is confusing since it does not really provide any
/// mutual exclusion, we rename it as exception cell (ExcpCell).
use cortex_m::interrupt::Mutex as ExcpCell;

/**
 *  SRAM Layout:
 *       High Address
 *  +-------------------+  - 0x2002_0000  Pointed by MSP (Main Stack Pointer)
 *  |     Exception     |
 *  |       Stack       |
 *  +-------------------+  - 0x2001_e000  Pointed by PSP (Process Stack Pointer)
 *  |       Init        |
 *  |       Stack       |
 *  +-------------------+  - 0x2001_c000  Pointed by PSP
 *  |    Task Stack 0   |
 *  +-------------------+  - 0x2001_b000  Pointed by PSP
 *  |        ...        |        ...
 *  +-------------------+  - 0x2001_9000  Pointed by PSP
 *  |    Task Stack 3   |
 *  +-------------------+  - 0x2001_8000
 *  |       Free        |
 *  +-------------------+  - 0x2000_0000
 */

#[repr(usize)]
enum StackBottom {
    ExceptionStack = 0x2002_0000,
    InitStack      = 0x2001_e000,
    TaskStack0     = 0x2001_c000,
    TaskStack1     = 0x2001_b000,
    TaskStack2     = 0x2001_a000,
    TaskStack3     = 0x2001_9000,
}

/// Software saved registers. These registers are saved upon
/// systick handler entry.
#[repr(C)]
struct SWSavedRegs {
    r4: u32,
    r5: u32,
    r6: u32,
    r7: u32,
    r8: u32,
    r9: u32,
    r10: u32,
    r11: u32,
}

/// Hardware saved registers. These registers are saved automatically
/// by hardware upon exceptions.
#[repr(C)]
struct HWSavedRegs {
    r0: u32,
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    lr: u32,
    pc: u32,
    psr: u32
}

/// The full trap frame, combining software and hardware saved registers.
#[repr(C)]
struct TrapFrame {
    sw_saved_regs: SWSavedRegs,
    hw_saved_regs: HWSavedRegs
}

/// Possible states of a task. Currently, we have only tasks with
/// forever loop, so these two states suffice.
enum TaskState {
    Empty,   // empty task struct
    Ready    // task struct is loaded with a context and can be switched to
}

/// The structure representing a task.
struct Task {
    stack_bottom: usize, // the address of the runtime stack bottom
    stack_top: usize,    // the address of the current runtime stack top
    state: TaskState     // the task state
}

impl Task {
    fn new(stack_bottom: usize) -> Task {
        Task {
            stack_top: 0,
            stack_bottom,
            state: TaskState::Empty
        }
    }

    /// Load the task structure with a task to run by providing the entry
    /// function. Here we set up a dummy trap frame, so that if we perform an
    /// exception return to this task, it will start executing from the
    /// entry function.
    fn load(&mut self, start: extern "C" fn()->!) {
        // Reserve space for a dummy trap frame.
        self.stack_top = self.stack_bottom - core::mem::size_of::<TrapFrame>();

        let tf = {
            let tf_ptr = self.stack_top as *mut TrapFrame;
            unsafe { &mut *tf_ptr }
        };

        // The only bit we need to set in the program state register (PSR)
        // is the Thumb bit (the 24th). Cortex-m4 always run in Thumb state,
        //  so we always have to set the Thumb bit.
        tf.hw_saved_regs.psr = 0x01000000;

        // This is the actual return address of the exception handler. We set
        // it to the entry function of the new task.
        tf.hw_saved_regs.pc = start as u32;

        // All the left registers can have arbitrary value.
        tf.hw_saved_regs.lr = 0;
        tf.hw_saved_regs.r12 = 0;
        tf.hw_saved_regs.r3 = 0;
        tf.hw_saved_regs.r2 = 0;
        tf.hw_saved_regs.r1 = 0;
        tf.hw_saved_regs.r0 = 0;
        tf.sw_saved_regs.r4 = 0;
        tf.sw_saved_regs.r5 = 0;
        tf.sw_saved_regs.r6 = 0;
        tf.sw_saved_regs.r7 = 0;
        tf.sw_saved_regs.r8 = 0;
        tf.sw_saved_regs.r9 = 0;
        tf.sw_saved_regs.r10 = 0;
        tf.sw_saved_regs.r11 = 0;

        self.state = TaskState::Ready;
    }
}

/// The singleton containing all task structures in the system.
struct TaskList {
    tasks: [Task; 4],        // an array of task structures
    last_run: Option<usize>  // the index of the last run task
}


lazy_static! {
    static ref PERIPHERALS: ExcpCell<RefCell<Peripherals>> = {
        let p = cortex_m::Peripherals::take().unwrap();
        ExcpCell::new(RefCell::new(p))
    };

    static ref TASKLIST: ExcpCell<RefCell<TaskList>> = {
        let tasks = [
            Task::new(StackBottom::TaskStack0 as usize),
            Task::new(StackBottom::TaskStack1 as usize),
            Task::new(StackBottom::TaskStack2 as usize),
            Task::new(StackBottom::TaskStack3 as usize)
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
#[entry]
fn main() -> ! {
    switch_to_psp_then_init(
        StackBottom::InitStack as usize, 
        StackBottom::ExceptionStack as usize
    )
}

#[naked]
extern "C" fn switch_to_psp_then_init(_spin_stack: usize, _exep_stack: usize) -> ! {
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

/// Initialize our system. This set up the timer and spawn two tasks.
extern "C" fn init() -> ! {
    // An interrupt free scope.
    interrupt::free(|cs| {
        {
            let mut p = PERIPHERALS.borrow(cs).borrow_mut();
            let syst = &mut p.SYST;

            // Configures the system timer to trigger a SysTick exception every second.
            // It has a default CPU clock of 16 MHz so we set the counter to 16_000_000.
            syst.set_clock_source(SystClkSource::Core);
            syst.set_reload(16_000_000);
            syst.clear_current();
            syst.enable_counter();
            syst.enable_interrupt();
        }

        {
            // Spawn two tasks. These tasks will be scheduled upon SysTick exception.
            let mut tasklist = TASKLIST.borrow(cs).borrow_mut();
            tasklist.tasks[0].load(loop_hello);
            tasklist.tasks[1].load(loop_world);
        }
    });

    // We will never return here after the spawned tasks are scheduled.
    loop {
        asm::wfi();
    }
}


// Set the export name so that the linker can find this function and
// put it to the correct entry in the exception vector table.
#[export_name = "SysTick"]
#[naked]
pub unsafe extern "C" fn syst_entry() -> ! {
    asm!(
        "push  {{lr}}",          // save LR to the *exception* stack
        "mrs   r0, psp",         // --+ 
        "stmdb r0!, {{r4-r11}}", //   | push r4-r11 to the *task* stack
        "msr   psp, r0",         // --+
        "bl    {schedule}",      // run the schedule function
        "mrs   r0, psp",         // --+ 
        "ldmia r0!, {{r4-r11}}", //   | pop r4-r11 from the *new task* stack
        "msr   psp, r0",         // --+
        "pop   {{pc}}",          // perform exception return
        schedule = sym schedule,
        options(noreturn)
    )
}

/// Round robin schedule function.
extern "C" fn schedule() {
    interrupt::free(|cs| {
        let mut tasklist = TASKLIST.borrow(cs).borrow_mut();
        let mut rr_start = 0;

        // Update the stack top stored in the task structure.
        if let Some(suspended_task_id) = tasklist.last_run {
            tasklist.tasks[suspended_task_id].stack_top = get_psp();
            rr_start = suspended_task_id + 1;
        }

        // Pick the next task to run in a round robin fashion.
        for (i, task) in tasklist.tasks.iter().enumerate().skip(rr_start) {
            if let TaskState::Ready = task.state {
                set_psp(task.stack_top);
                tasklist.last_run = Some(i);
                return;
            }
        }
        for (i, task) in tasklist.tasks[0..rr_start].iter().enumerate() {
            if let TaskState::Ready = task.state {
                set_psp(task.stack_top);
                tasklist.last_run = Some(i);
                return;
            }
        }
    });
}

// Endlessly print "hello ".
extern "C" fn loop_hello() -> ! {
    loop { 
        hprint!("hello ").unwrap();
        asm::wfi(); // halt the CPU until interrupt to save power
    }
}

// Endlessly print "world!". We intentionally make it less symmetric
// to `loop_hello()` so that it may potentially reveal more bugs in
// context switch.
extern "C" fn loop_world() -> ! {
    let c = '!';
    loop { 
        hprintln!("world{}", c).unwrap();
        asm::wfi(); // halt the CPU until interrupt to save power
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

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    hprintln!("HardFault occured, exception frame:").unwrap();
    hprintln!("{:?}", ef).unwrap();
    loop {}
}

#[exception]
fn DefaultHandler(ex_num: i16) {
    hprintln!("Exception {} occured!", ex_num).unwrap();
}
