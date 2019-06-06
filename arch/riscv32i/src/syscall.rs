//! Pseudo kernel-userland system call interface.
//!
//! This is for platforms that only include the "Machine Mode" privilege level.
//! Since these chips don't have hardware support for user mode, we have to fake
//! it. This means the apps have to be highly trusted as there is no real separation
//! between the kernel and apps.
//!
//! Note: this really only exists so we can demonstrate Tock running on actual
//! RISC-V hardware. Really, this is very undesirable for Tock as it violates
//! the safety properties of the OS. As hardware starts to exist that supports M
//! and U modes we will remove this.

use core::fmt::Write;
use core::ptr::{read_volatile, write_volatile};

use kernel;

#[allow(improper_ctypes)]
extern "C" {
    pub fn switch_to_user(user_stack: *const u8, process_regs: &mut [usize; 8]) -> *mut u8;
}

/// This holds all of the state that the kernel must keep for the process when
/// the process is not executing.
#[derive(Copy, Clone)]
pub struct RiscvimacStoredState {
    // 31 registers
    regs: [usize; 31],
    pc: usize,
    reason: usize,
}

impl Default for RiscvimacStoredState {
    fn default() -> RiscvimacStoredState {
        RiscvimacStoredState {
            regs: [0; 31],
            pc: 0,
            reason: 0,
        }
    }
}

/// Implementation of the `UserspaceKernelBoundary` for the RISC-V architecture.
pub struct SysCall();

impl SysCall {
    pub const unsafe fn new() -> SysCall {
        SysCall()
    }
}

impl kernel::syscall::UserspaceKernelBoundary for SysCall {
    type StoredState = RiscvimacStoredState;

    /// Get the syscall that the process called.
    unsafe fn get_syscall(&self, stack_pointer: *const usize) -> Option<kernel::syscall::Syscall> {
        None
    }

    unsafe fn set_syscall_return_value(&self, stack_pointer: *const usize, return_value: isize) {
        // riscv has same convention as cortex m
        let sp = stack_pointer as *mut isize;
        write_volatile(sp, return_value);
    }

    unsafe fn pop_syscall_stack_frame(
        &self,
        stack_pointer: *const usize,
        _state: &mut RiscvimacStoredState,
    ) -> *mut usize {
        stack_pointer as *mut usize
    }

    unsafe fn push_function_call(
        &self,
        stack_pointer: *const usize,
        _remaining_stack_memory: usize,
        _callback: kernel::procs::FunctionCall,
        _state: &RiscvimacStoredState,
    ) -> Result<*mut usize, *mut usize> {
        Err(stack_pointer as *mut usize)
    }

    unsafe fn switch_to_process(
        &self,
        stack_pointer: *const usize,
        _state: &mut RiscvimacStoredState,
    ) -> (*mut usize, kernel::syscall::ContextSwitchReason) {
        (
            stack_pointer as *mut usize,
            kernel::syscall::ContextSwitchReason::Fault,
        )
    }

    unsafe fn fault_fmt(&self, writer: &mut Write) {}

    unsafe fn process_detail_fmt(
        &self,
        stack_pointer: *const usize,
        state: &RiscvimacStoredState,
        writer: &mut Write,
    ) {
    }
}
