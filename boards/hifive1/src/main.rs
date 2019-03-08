//! Board file for SiFive HiFive1 RISC-V development platform.
//!
//! - <https://www.sifive.com/products/hifive1/>

#![no_std]
#![no_main]
#![feature(asm)]

extern crate capsules;
#[allow(unused_imports)]
#[macro_use(create_capability, debug, debug_gpio, static_init)]
extern crate kernel;
// registers
extern crate riscvregs;
extern crate e310x;
extern crate riscv32i;
extern crate sifive;

// use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
// use capsules::virtual_i2c::{I2CDevice, MuxI2C};
use capsules::virtual_uart::{UartDevice, UartMux};
use kernel::capabilities;
use kernel::hil;
use kernel::Platform;

pub mod io;

// State for loading and holding applications.

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::procs::FaultResponse = kernel::procs::FaultResponse::Panic;

// RAM to be shared by all application processes.
#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 8192] = [0; 8192];

// Actual memory for holding the active process structures.
static mut PROCESSES: [Option<&'static kernel::procs::ProcessType>; NUM_PROCS] =
    [None, None, None, None];

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

/// A structure representing this platform that holds references to all
/// capsules for this platform.
struct HiFive1 {
    // console: &'static capsules::console::Console<'static, UartDevice<'static>>,
    gpio: &'static capsules::gpio::GPIO<'static, sifive::gpio::GpioPin>,
    // alarm: &'static capsules::alarm::AlarmDriver<
    //     'static,
    //     VirtualMuxAlarm<'static, sam4l::ast::Ast<'static>>,
    // >,
    led: &'static capsules::led::LED<'static, sifive::gpio::GpioPin>,
    // button: &'static capsules::button::Button<'static, sam4l::gpio::GPIOPin>,
    // ipc: kernel::ipc::IPC,
}

/// Mapping of integer syscalls to objects that implement syscalls.
impl Platform for HiFive1 {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            // capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),

            // capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            // capsules::button::DRIVER_NUM => f(Some(self.button)),

            // kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

pub unsafe fn trap_save_cause(mepc: u32) {
    riscvregs::register::mepc::write(mepc as usize);
}


/// Reset Handler.
///
/// This function is called from the arch crate after some very basic RISC-V
/// setup.
#[no_mangle]
#[allow(unused_variables)]
pub unsafe fn reset_handler() {
    // Basic setup of the platform.
    riscv32i::init_memory();
    let PMP_NAPOT = 0x18;
    let PMP_R = 0x1;
    let PMP_W = 0x02;
    let PMP_X = 0x04;
    let pmpc = PMP_NAPOT | PMP_R | PMP_W | PMP_X;
    let save = 0;
    // let tfn = trap_save_cause;
    let PMPCFG_COUNT = 4;
    let PMPADDR_COUNT = 16;
    asm!("
csrw 0x3A0, x0
csrw 0x3A1, x0
csrw 0x3A2, x0
csrw 0x3A3, x0
csrw 0x3B0, x0
csrw 0x3B1, x0
csrw 0x3B2, x0
csrw 0x3B3, x0
csrw 0x3B4, x0
csrw 0x3B5, x0
csrw 0x3B6, x0
csrw 0x3B7, x0
csrw 0x3B8, x0
csrw 0x3B9, x0
csrw 0x3BA, x0
csrw 0x3BB, x0
csrw 0x3BC, x0
csrw 0x3BD, x0
csrw 0x3BE, x0
csrw 0x3BF, x0
");
    // tfn = 0;
    asm!(
        "
lui t0, %hi(0x1f)
addi t0, t0, %lo(0x1f)
csrrw t0, 0x301, t0
lui t0, %hi(0xffffffff)
addi t0, t0, %lo(0xffffffff)
csrw 0x3B0, t0
lui t0, %hi(0x1f)
addi t0, t0, %lo(0x1f)
csrw 0x3A0, t0
.align 2
1: csrw 0x301, t0
" : : : "t0"
    );
    riscv32i::configure_machine_trap_handler();
    riscv32i::configure_supervisor_trap_handler();
    riscvregs::register::mstatus::set_mpp(riscvregs::register::mstatus::MPP::User);

    asm!("
    lui a0, %hi(0x80101000)
    addi a0, a0, %lo(0x80101000)
    csrw 0x341, a0
    mret
");


    // let b = riscvregs::register::mtvec::read();
    // let a = riscvregs::register::mepc::read();
    // debug!(" csr val is: {:?}", a);
    // read mstatus csr
    // write MPP bit as 0
    // write back to csr

//     asm!("
//     // csrr    t0, 0x300
//     // li      t1, (3<<11)
//     // or      t0, t0, t1
//     // xor     t0, t0, t1
//     // li      t1, (1<<11) | (1<<17)
//     // or      t0, t0, t1
//     // csrw    0x300, t0

//     lui a0, %hi(0x80101000)
//     addi a0, a0, %lo(0x80101000)
//     csrw 0x341, a0
//     mret
// ");
    // riscv32i::configure_user_trap_handler();
    // asm!("
    //     // set mepc to 0x20c00000
    //     lui a0, %hi(0x80101000)
    //     addi a0, a0, %lo(0x80101000)
    //     csrw 0x041, a0

    //     // now go to what is in mepc
    //     uret
    //     " ::::);
    // asm!("uret");

    // e310x::watchdog::WATCHDOG.disable();
    // e310x::rtc::RTC.disable();
    // e310x::pwm::PWM0.disable();
    // e310x::pwm::PWM1.disable();
    // e310x::pwm::PWM2.disable();




    // testing some mret jump-around code


    // extern "C" {
    //     /// Beginning of the ROM region containing app images.
    //     ///
    //     /// This symbol is defined in the linker script.
    //     static _sapps: u8;
    // }

    // kernel::procs::load_processes(
    //     board_kernel,
    //     chip,
    //     &_sapps as *const u8,
    //     &mut APP_MEMORY,
    //     &mut PROCESSES,
    //     FAULT_RESPONSE,
    //     &process_mgmt_cap,
    // );
    // board_kernel.kernel_loop(&hifive1, chip, None, &main_loop_cap);
}
