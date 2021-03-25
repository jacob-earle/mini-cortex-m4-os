//! This file is derived from `cortex-m-rt` crate, with `Reset`
//! function modified.

use core::{ptr, mem, fmt};
use super::main;

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static __RESET_VECTOR: unsafe extern "C" fn() -> ! = Reset;

/// Registers stacked (pushed into the stack) during an exception
#[derive(Clone, Copy)]
#[repr(C)]
pub struct ExceptionFrame {
    /// (General purpose) Register 0
    pub r0: u32,

    /// (General purpose) Register 1
    pub r1: u32,

    /// (General purpose) Register 2
    pub r2: u32,

    /// (General purpose) Register 3
    pub r3: u32,

    /// (General purpose) Register 12
    pub r12: u32,

    /// Linker Register
    pub lr: u32,

    /// Program Counter
    pub pc: u32,

    /// Program Status Register
    pub xpsr: u32,
}

impl fmt::Debug for ExceptionFrame {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        struct Hex(u32);
        impl fmt::Debug for Hex {
            fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
                write!(f, "0x{:08x}", self.0)
            }
        }
        f.debug_struct("ExceptionFrame")
            .field("r0", &Hex(self.r0))
            .field("r1", &Hex(self.r1))
            .field("r2", &Hex(self.r2))
            .field("r3", &Hex(self.r3))
            .field("r12", &Hex(self.r12))
            .field("lr", &Hex(self.lr))
            .field("pc", &Hex(self.pc))
            .field("xpsr", &Hex(self.xpsr))
            .finish()
    }
}

/// Returns a pointer to the start of the heap
///
/// The returned pointer is guaranteed to be 4-byte aligned.
#[inline]
pub fn heap_start() -> *mut u32 {
    extern "C" {
        static mut __sheap: u32;
    }

    let p: *mut u32;
    unsafe {
        asm!(
            "ldr  {r}, ={sheap}",
            r = out(reg) p,
            sheap = sym __sheap
        );
    }
    p
}

#[link_section = ".Reset"]
#[allow(non_snake_case)]
#[export_name = "Reset"]
unsafe extern "C" fn Reset() -> ! {
    extern "C" {
        // These symbols come from `link.x`
        static mut __sbss: u32;
        static mut __ebss: u32;
        static mut __sdata: u32;
        static mut __edata: u32;
        static __sidata: u32;
    }

    asm!(
        "ldr r9, ={sdata}",
        "ldr r0, ={sbss}",
        "ldr r1, ={ebss}",
        "bl  {memzero}",
        "ldr r0, ={sdata}",
        "ldr r1, ={edata}",
        "ldr r2, ={sidata}",
        "bl  {memcopy}",
        "bl  {main}",
        sbss = sym __sbss,
        ebss = sym __ebss,
        sdata = sym __sdata,
        edata = sym __edata,
        sidata = sym __sidata,
        memzero = sym memzero,
        memcopy = sym memcopy,
        main = sym main
    );

    loop {}
}

extern "C" fn memzero(start: u32, end: u32) {
    let mut s = start as *mut u8;
    let e = end as *mut u8;
    while s < e {
        unsafe {
            ptr::write_volatile(s, mem::zeroed()); 
            s = s.offset(1);
        }
    }
}

extern "C" fn memcopy(dst_start: u32, dst_end: u32, src_start: u32) {
    let mut dst_s = dst_start as *mut u8;
    let dst_e = dst_end as *mut u8;
    let mut src_s = src_start as *const u8;
    while dst_s < dst_e {
        unsafe {
            ptr::write_volatile(dst_s, ptr::read_volatile(src_s));
            dst_s = dst_s.offset(1);
            src_s = src_s.offset(1);
        }
    }
}

pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

#[link_section = ".vector_table.exceptions"]
#[no_mangle]
pub static __EXCEPTIONS: [Vector; 14] = [
    // Exception 2: Non Maskable Interrupt.
    Vector {
        handler: NonMaskableInt,
    },
    // Exception 3: Hard Fault Interrupt.
    Vector { handler: HardFaultTrampoline },
    // Exception 4: Memory Management Interrupt [not on Cortex-M0 variants].
    #[cfg(not(armv6m))]
    Vector {
        handler: MemoryManagement,
    },
    #[cfg(armv6m)]
    Vector { reserved: 0 },
    // Exception 5: Bus Fault Interrupt [not on Cortex-M0 variants].
    #[cfg(not(armv6m))]
    Vector { handler: BusFault },
    #[cfg(armv6m)]
    Vector { reserved: 0 },
    // Exception 6: Usage Fault Interrupt [not on Cortex-M0 variants].
    #[cfg(not(armv6m))]
    Vector {
        handler: UsageFault,
    },
    #[cfg(armv6m)]
    Vector { reserved: 0 },
    // Exception 7: Secure Fault Interrupt [only on Armv8-M].
    #[cfg(armv8m)]
    Vector {
        handler: SecureFault,
    },
    #[cfg(not(armv8m))]
    Vector { reserved: 0 },
    // 8-10: Reserved
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    // Exception 11: SV Call Interrupt.
    Vector { handler: SVCall },
    // Exception 12: Debug Monitor Interrupt [not on Cortex-M0 variants].
    #[cfg(not(armv6m))]
    Vector {
        handler: DebugMonitor,
    },
    #[cfg(armv6m)]
    Vector { reserved: 0 },
    // 13: Reserved
    Vector { reserved: 0 },
    // Exception 14: Pend SV Interrupt [not on Cortex-M0 variants].
    Vector { handler: PendSV },
    // Exception 15: System Tick Interrupt.
    Vector { handler: SysTick },
];

extern "C" {
    fn HardFault();

    fn NonMaskableInt();

    #[cfg(not(armv6m))]
    fn MemoryManagement();

    #[cfg(not(armv6m))]
    fn BusFault();

    #[cfg(not(armv6m))]
    fn UsageFault();

    #[cfg(armv8m)]
    fn SecureFault();

    fn SVCall();

    #[cfg(not(armv6m))]
    fn DebugMonitor();

    fn PendSV();

    fn SysTick();
}

#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 240] = [{
    extern "C" {
        fn DefaultHandler();
    }

    DefaultHandler
}; 240];

#[allow(non_snake_case)]
#[export_name = "HardFaultTrampoline"]
pub extern "C" fn HardFaultTrampoline() {
    unsafe { HardFault(); }
}

#[allow(non_snake_case)]
#[export_name = "DefaultHandler_"]
pub extern "C" fn DefaultHandler_() {
    loop {}
}
