use gpio;
use interrupts;
use kernel;
use riscv32i;
use riscv32i::plic;
use uart;

pub struct E310x {
    userspace_kernel_boundary: riscv32i::syscall::SysCall,
}

impl E310x {
    pub unsafe fn new() -> E310x {
        E310x {
            userspace_kernel_boundary: riscv32i::syscall::SysCall::new(),
        }
    }
}

// this is actually for the riscv32i chip
impl kernel::Chip for E310x {
    type MPU = ();
    type UserspaceKernelBoundary = riscv32i::syscall::SysCall;
    type SysTick = ();

    fn mpu(&self) -> &Self::MPU {
        &()
    }

    fn systick(&self) -> &Self::SysTick {
        &()
    }

    fn userspace_kernel_boundary(&self) -> &riscv32i::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn service_pending_interrupts(&self) {
        unsafe {
            //while plic::has_pending() {
            let trap_id = plic::claim_m_mode();
            // no interrupt to service
            if trap_id == 0 {
                return;
            }
            debug!("Pidx {}", trap_id);

            (match trap_id {
                interrupts::UART0 => uart::UART0.handle_interrupt(),
                index @ interrupts::GPIO0..interrupts::GPIO31 => {
                    gpio::PORT[index as usize].handle_interrupt()
                }
                //_ => debug!("PLIC index not supported by Tock {}", interrupt),
                _ => debug!("Pidx {}", trap_id),
            });
            plic::complete(trap_id);
            //}
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { plic::has_pending() }
    }

    fn sleep(&self) {
        unsafe {
            //riscv32i::support::wfi();
            for i in 0..500000 {
                riscv32i::support::nop();
            }
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        riscv32i::support::atomic(f)
    }
}
