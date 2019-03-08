#![crate_name = "riscv32i"]
#![crate_type = "rlib"]
#![feature(asm, const_fn, lang_items, global_asm)]
#![no_std]

#[macro_use(register_bitfields, register_bitmasks)]
extern crate kernel;
extern crate riscvregs;

pub mod plic;
pub mod support;
pub mod syscall;

extern "C" {
    // External function defined by the board main.rs.
    fn reset_handler();

    // Where the end of the stack region is (and hence where the stack should
    // start).
    static _estack: u32;

    // Address of _start_trap.
    static _start_trap: u32;

    // Boundaries of the .bss section.
    static mut _szero: u32;
    static mut _ezero: u32;

    // Where the .data section is stored in flash.
    static mut _etext: u32;

    // Boundaries of the .data section.
    static mut _srelocate: u32;
    static mut _erelocate: u32;
}

struct StackFrame {
    ra: u32,
    t0: u32,
    t1: u32,
    t2: u32,
    t3: u32,
    t4: u32,
    t5: u32,
    t6: u32,
    a0: u32,
    a1: u32,
    a2: u32,
    a3: u32,
    a4: u32,
    a5: u32,
    a6: u32,
    a7: u32,
}

// unsafe fn _start_trap2(){
//     let sf = StackFrame{0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// }

/// Entry point of all programs (_start).
///
/// It initializes DWARF call frame information, the stack pointer, the
/// frame pointer (needed for closures to work in start_rust) and the global
/// pointer. Then it calls _start_rust.
#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
global_asm!(
    r#"
.section .riscv.start, "ax"
.globl _start
_start:
  .cfi_startproc
  .cfi_undefined ra

  // Set the global pointer register using the variable defined in the linker
  // script. This register is only set once. The global pointer is a method
  // for sharing state between the linker and the CPU so that the linker can
  // emit code with offsets that are relative to the gp register, and the CPU
  // can successfully execute them.
  //
  // https://gnu-mcu-eclipse.github.io/arch/riscv/programmer/#the-gp-global-pointer-register
  // https://groups.google.com/a/groups.riscv.org/forum/#!msg/sw-dev/60IdaZj27dY/5MydPLnHAQAJ
  // https://www.sifive.com/blog/2017/08/28/all-aboard-part-3-linker-relaxation-in-riscv-toolchain/
  //
  lui gp, %hi(__global_pointer$)
  addi gp, gp, %lo(__global_pointer$)

  // Initialize the stack pointer register. This comes directly from the linker
  // script.
  lui sp, %hi(_estack)
  addi sp, sp, %lo(_estack)

  // Set s0 (the frame pointer) to the start of the stack.
  add s0, sp, zero

  // With that initial setup out of the way, we now branch to the main code,
  // likely defined in a board's main.rs.
  jal zero, reset_handler

  .cfi_endproc
"#
);

/// Setup memory for the kernel.
///
/// This moves the data segment from flash to RAM and zeros out the BSS section.
pub unsafe fn init_memory() {
    // Relocate data segment.
    // Assumes data starts right after text segment as specified by the linker
    // file.
    let mut pdest = &mut _srelocate as *mut u32;
    let pend = &mut _erelocate as *mut u32;
    let mut psrc = &_etext as *const u32;

    if psrc != pdest {
        while (pdest as *const u32) < pend {
            *pdest = *psrc;
            pdest = pdest.offset(1);
            psrc = psrc.offset(1);
        }
    }

    // Clear the zero segment (BSS)
    let pzero = &_ezero as *const u32;
    pdest = &mut _szero as *mut u32;

    while (pdest as *const u32) < pzero {
        *pdest = 0;
        pdest = pdest.offset(1);
    }
}

// enum ValToWrite{
//     Val(u32),
//     Addr(&'static unsafe extern "C" fn() {start_trap_rust})
// }

/// Tell the MCU what address the trap handler is located at.
///
/// The trap handler is called on exceptions and for interrupts.
pub unsafe fn configure_machine_trap_handler() {
    // riscvregs::register::mtvec::write(start_trap_rust as usize, riscvregs::register::mtvec::TrapMode::Direct);
    asm!("csrw 0x305, $0": : "r"(&_start_trap) : : "volatile");
}
/// Tell the MCU what address the trap handler is located at.
///
/// The trap handler is called on exceptions and for interrupts.
pub unsafe fn configure_supervisor_trap_handler() {
    asm!("csrw 0x105, $0": : "r"(&_start_trap) : : "volatile");
    // riscvregs::register::stvec::write(start_trap_rust as usize, riscvregs::register::stvec::TrapMode::Direct);
    // let addr: &'static unsafe extern "C" fn() = start_trap_rust;
    // write_csr(0x005, (start_trap_rust) as u32);
}

pub unsafe fn configure_user_trap_handler() {
    asm!("csrw 0x005, $0": : "r"(&_start_trap) : : "volatile");
    
}

/// Enable all PLIC interrupts so that individual peripheral drivers do not have
/// to manage these.
pub unsafe fn enable_plic_interrupts() {
    plic::disable_all();
    plic::clear_all_pending();
    plic::enable_all();
}

// macro_rules! read_register {
//     #[inline]
//     ($reg_name:tt, $var:ident) => {
//         asm!("sw ")
//     }
// }

/// Trap entry point (_start_trap)
///
/// Saves caller saved registers ra, t0..6, a0..7, calls _start_trap_rust,
/// restores caller saved registers and then returns.
#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
global_asm!(
    r#"
  .section .riscv.trap, "ax"
  .align 4
  .global _start_trap

_start_trap:
  addi sp, sp, -16*4

  // csrr t6, 0x342;

  //csrr t6, 0x343;
  sw ra, 0*4(sp)
  sw t0, 1*4(sp)
  sw t1, 2*4(sp)
  sw t2, 3*4(sp)
  sw t3, 4*4(sp)
  sw t4, 5*4(sp)
  sw t5, 6*4(sp)
  sw t6, 7*4(sp)
  sw a0, 8*4(sp)
  sw a1, 9*4(sp)
  sw a2, 10*4(sp)
  sw a3, 11*4(sp)
  sw a4, 12*4(sp)
  sw a5, 13*4(sp)
  sw a6, 14*4(sp)
  sw a7, 15*4(sp)

  jal ra, _start_trap_rust

  lw ra, 0*4(sp)
  lw t0, 1*4(sp)
  lw t1, 2*4(sp)
  lw t2, 3*4(sp)
  lw t3, 4*4(sp)
  lw t4, 5*4(sp)
  lw t5, 6*4(sp)
  lw t6, 7*4(sp)
  lw a0, 8*4(sp)
  lw a1, 9*4(sp)
  lw a2, 10*4(sp)
  lw a3, 11*4(sp)
  lw a4, 12*4(sp)
  lw a5, 13*4(sp)
  lw a6, 14*4(sp)
  lw a7, 15*4(sp)

  addi sp, sp, 16*4
  mret
"#
);

/// Trap entry point rust (_start_trap_rust)
///
/// mcause is read to determine the cause of the trap. XLEN-1 bit indicates
/// if it's an interrupt or an exception. The result is converted to an element
/// of the Interrupt or Exception enum and passed to handle_interrupt or
/// handle_exception.
// #[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust"]
pub unsafe extern "C" fn start_trap_rust() {
    asm!("csrr t0, 0x342");
    asm!("ret");
    // asm!("j ra");
    // // dispatch trap to handler
    // trap_handler(mcause::read().cause());
    // // mstatus, remain in M-mode after mret
    // unsafe {
    //     mstatus::set_mpp(mstatus::MPP::Machine);
    // }
}

// Make sure there is an abort when linking.
//
// I don't know why we need this, or why cortex-m doesn't seem to have it.
#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
global_asm!(
    r#"
.section .init
.globl abort
abort:
  jal zero, _start
"#
);
