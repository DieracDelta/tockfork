//! Board file for SiFive HiFive1 RISC-V development platform.
//!
//! - <https://www.sifive.com/products/hifive1/>
//!
//!

//This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

//© 2019 Massachusetts Institute of Technology.

//The software/firmware is provided to you on an As-Is basis

//Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.

#![no_std]
#![no_main]
#![feature(asm)]
#![feature(global_asm)]

extern crate capsules;
#[allow(unused_imports)]
#[macro_use(create_capability, debug, debug_gpio, static_init)]
extern crate kernel;
// registers
extern crate e310x;
extern crate riscv32i;
extern crate riscvregs;
extern crate sifive;
use core::fmt::Write;
use kernel::common::StaticRef;
use kernel::debug::get_debug_writer;
use sifive::uart::UartRegisters;

// use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
// use capsules::virtual_i2c::{I2CDevice, MuxI2C};
use capsules::virtual_uart::{UartDevice, UartMux};
use kernel::capabilities;
use kernel::hil;
use kernel::Platform;

pub mod io;

extern "C" {
    static _fake_userland_2: u32;
}

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
    //led: &'static capsules::led::LED<'static, sifive::gpio::GpioPin>,
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
            //capsules::led::DRIVER_NUM => f(Some(self.led)),
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

    //configure_pmp();
    // configure trap handler addresses
    //riscv32i::configure_machine_trap_handler();
    //riscv32i::configure_supervisor_trap_handler();
    // riscv32i::configure_user_trap_handler();

    e310x::watchdog::WATCHDOG.disable();
    e310x::rtc::RTC.disable();
    e310x::pwm::PWM0.disable();
    e310x::pwm::PWM1.disable();
    e310x::pwm::PWM2.disable();
    // enables prci
    e310x::prci::PRCI.set_clock_frequency(sifive::prci::ClockFrequency::Freq18Mhz);

    // enable interrupts
    //riscv32i::enable_plic_interrupts();
    //enable m mode software interrupts
    //riscv32i::plic::enable_all_sources();
    //riscv32i::plic::set_priority_threshold(0);
    //riscv32i::plic::enable_interrupts();
    //riscvregs::register::mstatus::set_mpie();
    //riscv32i::clint::write_mtimecmp1(0x0);

    let process_mgmt_cap = create_capability!(capabilities::ProcessManagementCapability);
    let main_loop_cap = create_capability!(capabilities::MainLoopCapability);
    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));
    kernel::debug::assign_gpios(
        Some(&e310x::gpio::PORT[22]), // Red
        None,
        None,
    );
    let the_chip = static_init!(e310x::chip::E310x, e310x::chip::E310x::new());
    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux = static_init!(
        UartMux<'static>,
        UartMux::new(
            &e310x::uart::UART0,
            &mut capsules::virtual_uart::RX_BUF,
            115200
        )
    );
    hil::uart::UART::set_client(&e310x::uart::UART0, uart_mux);
    uart_mux.initialize();
    let gpio_pins = static_init!([&'static sifive::gpio::GpioPin; 0], []);
    let gpio = static_init!(
        capsules::gpio::GPIO<'static, sifive::gpio::GpioPin>,
        capsules::gpio::GPIO::new(gpio_pins)
    );

    let hifive1 = HiFive1 { gpio: gpio };
    // Create virtual device for kernel debug.
    let debugger_uart = static_init!(UartDevice, UartDevice::new(uart_mux, false));
    debugger_uart.setup();
    let debugger = static_init!(
        kernel::debug::DebugWriter,
        kernel::debug::DebugWriter::new(
            debugger_uart,
            &mut kernel::debug::OUTPUT_BUF,
            &mut kernel::debug::INTERNAL_BUF,
        )
    );
    hil::uart::UART::set_client(debugger_uart, debugger);
    //UartDevice::UART::set_client(debugger_uart );

    let debug_wrapper = static_init!(
        kernel::debug::DebugWriterWrapper,
        kernel::debug::DebugWriterWrapper::new(debugger)
    );
    kernel::debug::set_debug_writer_wrapper(debug_wrapper);

    // for e310x, gpio pin 17 is tx, gpio pin 16 is rx
    // see page 5 https://sifive.cdn.prismic.io/sifive%2Ffeb6f967-ff96-418f-9af4-a7f3b7fd1dfc_fe310-g000-ds.pdf
    e310x::uart::UART0.initialize_gpio_pins(&e310x::gpio::PORT[17], &e310x::gpio::PORT[16]);

    const UART0_BASE: StaticRef<UartRegisters> =
        unsafe { StaticRef::new(0x1001_3000 as *const UartRegisters) };
    let uart0 = &UART0_BASE;
    uart0.div.write(sifive::uart::div::div.val(138));
    uart0.txctrl.modify(sifive::uart::txctrl::txen::SET);
    while (1 == (uart0.txctrl.get() & 0x80000000)) {}
    uart0.txdata.set('y' as u32);
    debug!("hi\r\n");
    riscvregs::register::mstatus::set_mpp(riscvregs::register::mstatus::MPP::Machine);

    //debug!("hello world 1");
    //<e310x::chip::E310x as kernel::Chip>::service_pending_interrupts(&the_chip);
    //debug!("hello world 2");
    //<e310x::chip::E310x as kernel::Chip>::service_pending_interrupts(&the_chip);
    // return into userland code (0x80101000) with supervisor privilege levels
    fake_return_to_userland_2();
}

pub unsafe fn configure_pmp() {
    // set all PMP config and address sections to zero
    // documentation came from riscv volume 2 spec 1.10
    // https://content.riscv.org/wp-content/uploads/2017/05/riscv-privileged-v1.10.pdf
    asm!(
        "
        // config sections to null region/off
        csrw 0x3A0, x0
        csrw 0x3A1, x0
        csrw 0x3A2, x0
        csrw 0x3A3, x0
        // address registers
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
    "
    );

    asm!("
        // update mstatus
        // lui t0, %hi(0x1f)
        // addi t0, t0, %lo(0x1f)
        // csrrw t0, 0x301, t0


        // set last PMP section to largest possible address for perms to entire address space
        lui t0, %hi(0xffffffff)
        addi t0, t0, %lo(0xffffffff)
        csrw 0x3B0, t0

        // set pmpcfg0
        // set A field to TOR
        // set L field to 0
        // set X, W, R fields
        lui t0, %hi(0x1f)
        addi t0, t0, %lo(0x1f)
        csrw 0x3A0, t0
        " : : : "t0"
    );
}

pub unsafe fn fake_return_to_userland() {
    asm!(
        "
        lui a0, %hi(0x80101000)
        addi a0, a0, %lo(0x80101000)
        csrw 0x341, a0
        mret
    "
    );
}

#[export_name = "fake_userland_2"]
pub unsafe extern "C" fn fake_userland_2() {
    // triggering software interrupt works
    //riscv32i::clint::trigger_software_interrupt();

    //asm!("ecall");
    debug!("first hello world");
    //hil::uart::UART::abort_receive(&e310x::uart::UART0);
    debug!("second hello world");
    // triggers timer interrupts every 6000
    //riscv32i::clint::write_mtimecmp1(6000);

    //this works now! calls start_trap_rust
    //riscv32i::clint::trigger_software_interrupt();
}

global_asm!(
    r#"
    .align 4
    .global _fake_userland_2

_fake_userland_2:
    call fake_userland_2
    "#
);

pub unsafe fn fake_return_to_userland_2() {
    asm!(
"
csrw 0x341, $0
mret
" : : "r"(&_fake_userland_2) : : "volatile"
    );
}
