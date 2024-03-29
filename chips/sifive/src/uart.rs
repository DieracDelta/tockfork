use core::cell::Cell;

use gpio;
use kernel::common::cells::OptionalCell;
use kernel::common::cells::TakeCell;
use kernel::common::registers::{ReadOnly, ReadWrite};
use kernel::common::StaticRef;
use kernel::hil;
use kernel::ReturnCode;

#[repr(C)]
pub struct UartRegisters {
    /// Transmit Data Register
    pub txdata: ReadWrite<u32, txdata::Register>,
    /// Receive Data Register
    pub rxdata: ReadWrite<u32, rxdata::Register>,
    /// Transmit Control Register
    pub txctrl: ReadWrite<u32, txctrl::Register>,
    /// Receive Control Register
    pub rxctrl: ReadWrite<u32, rxctrl::Register>,
    /// Interrupt Enable Register
    pub ie: ReadWrite<u32, interrupt::Register>,
    /// Interrupt Pending Register
    pub ip: ReadOnly<u32, interrupt::Register>,
    /// Baud Rate Divisor Register
    pub div: ReadWrite<u32, div::Register>,
}

register_bitfields![u32,
txdata [
    full OFFSET(31) NUMBITS(1) [],
    data OFFSET(0) NUMBITS(8) []
],
rxdata [
    empty OFFSET(31) NUMBITS(1) [],
    data OFFSET(0) NUMBITS(8) []
],
txctrl [
    // morally speaking this hsould be 3 bits but I just want to set it to 1
    txcnt OFFSET(16) NUMBITS(3) [],
    nstop OFFSET(1) NUMBITS(1) [
        OneStopBit = 0,
        TwoStopBits = 1
    ],
    txen OFFSET(0) NUMBITS(1) []
],
rxctrl [
    rxcnt OFFSET(16) NUMBITS(3) [],
    rxen OFFSET(0) NUMBITS(1) []
],
interrupt [
    rxwm OFFSET(1) NUMBITS(1) [],
    txwm OFFSET(0) NUMBITS(1) []
],
div [
    div OFFSET(0) NUMBITS(16) []
]
];

pub struct Uart {
    registers: StaticRef<UartRegisters>,
    client: OptionalCell<&'static hil::uart::Client>,
    stop_bits: Cell<hil::uart::StopBits>,
    buffer: TakeCell<'static, [u8]>,
    len: Cell<usize>,
    index: Cell<usize>,
}

#[derive(Copy, Clone)]
pub struct UartParams {
    pub baud_rate: u32,
}

impl Uart {
    pub const fn new(base: StaticRef<UartRegisters>) -> Uart {
        Uart {
            registers: base,
            client: OptionalCell::empty(),
            stop_bits: Cell::new(hil::uart::StopBits::One),
            buffer: TakeCell::empty(),
            len: Cell::new(0),
            index: Cell::new(0),
        }
    }

    /// Configure GPIO pins for the UART.
    pub fn initialize_gpio_pins(&self, tx: &gpio::GpioPin, rx: &gpio::GpioPin) {
        tx.iof0();
    }

    fn set_baud_rate(&self, baud_rate: u32) {
        let regs = self.registers;

        // Assume that the clock is running at 384 MHz.
        // let clock_speed = 384_000_000 as u32;

        let clock_speed = 32_000_000 as u32;

        //            f_clk
        // f_baud = ---------
        //           div + 1
        let divisor = (clock_speed / baud_rate) - 1;

        regs.div.write(div::div.val(divisor));
    }

    fn enable_tx_interrupt(&self) {
        let regs = self.registers;
        regs.ie.modify(interrupt::txwm::SET);
        regs.txctrl.modify(txctrl::txcnt::SET);
    }

    fn disable_tx_interrupt(&self) {
        let regs = self.registers;
        regs.ie.modify(interrupt::txwm::CLEAR);
    }

    fn enable_rx_interrupt(&self) {
        let regs = self.registers;
        regs.ie.modify(interrupt::rxwm::SET);
    }

    fn disable_rx_interrupt(&self) {
        let regs = self.registers;
        regs.ie.modify(interrupt::rxwm::CLEAR);
    }

    pub fn handle_interrupt(&self) {
        let regs = self.registers;

        // Get a copy so we can check each interrupt flag in the register.
        let pending_interrupts = regs.ip.extract();

        // Determine why an interrupt occurred.
        if pending_interrupts.is_set(interrupt::txwm) {
            // Got a TX interrupt which means the number of bytes in the FIFO
            // has fallen to zero. If there is more to send do that, otherwise
            // send a callback to the client.
            if self.len.get() == self.index.get() {
                // We are done.
                regs.txctrl.write(txctrl::txen::CLEAR);
                self.disable_tx_interrupt();

                // Signal client write done
                self.client.map(|client| {
                    self.buffer.take().map(|buffer| {
                        client.transmit_complete(buffer, hil::uart::Error::CommandComplete);
                    });
                });
            } else {
                // More to send. Fill the buffer until it is full.
                self.buffer.map(|buffer| {
                    for i in self.index.get()..self.len.get() {
                        // Write the byte from the array to the tx register.
                        regs.txdata.write(txdata::data.val(buffer[i] as u32));
                        self.index.set(i + 1);
                        // Check if the buffer is full
                        if regs.txdata.is_set(txdata::full) {
                            // If it is, break and wait for the TX interrupt.
                            break;
                        }
                    }
                });
            }
        }

        //if pending_interrupts.is_set(interrupt::rxwm) {
        //// TODO
        //}
    }
}

impl hil::uart::UART for Uart {
    fn set_client(&self, client: &'static hil::uart::Client) {
        self.client.set(client);
    }

    fn configure(&self, params: hil::uart::UARTParameters) -> ReturnCode {
        // This chip does not support these features.
        if params.parity != hil::uart::Parity::None {
            return ReturnCode::ENOSUPPORT;
        }
        if params.hw_flow_control != false {
            return ReturnCode::ENOSUPPORT;
        }

        // We can set the baud rate.
        self.set_baud_rate(params.baud_rate);

        // We need to save the stop bits because it is set in the TX register.
        self.stop_bits.set(params.stop_bits);

        ReturnCode::SUCCESS
    }

    //fn freebuffer(&self) {
    //self.client.map(|aclient| {
    //self.buffer.take().map(|a| {
    //aclient.transmit_complete(a, kernel::hil::uart::Error::CommandComplete);
    //});
    //});
    //}

    fn transmit(&self, tx_data: &'static mut [u8], tx_len: usize) {
        let regs = self.registers;

        if tx_len == 0 {
            return;
        }

        self.buffer.replace(tx_data);

        // Enable the interrupt so we know when we can keep writing.
        regs.ie.write(interrupt::txwm::SET + interrupt::rxwm::SET);
        //regs.txctrl.modify(txctrl::txcnt::SET);
        let stop_bits = match self.stop_bits.get() {
            hil::uart::StopBits::One => txctrl::nstop::OneStopBit,
            hil::uart::StopBits::Two => txctrl::nstop::TwoStopBits,
        };
        // just to confirm that tx interrupts are actually enabled
        let a = regs.ie.read(interrupt::txwm);
        regs.txctrl
            .write(txctrl::txen::SET + stop_bits + txctrl::txcnt.val(1));
        //regs.rxctrl.write(rxctrl::rxen.val(1) + rxctrl::rxcnt::SET);
        //if a == 0 {
        //regs.txctrl
        //.write(txctrl::txen::SET + stop_bits + txctrl::txcnt.val(1));
        //}

        // Fill the TX buffer until it reports full.
        for i in 0..tx_len {
            // Write the byte from the array to the tx register.
            self.buffer.map(|tx_buffer| {
                regs.txdata.set(tx_buffer[i] as u32);
                self.index.set(i + 1);

                //let a = regs.ip.read(interrupt::txwm);
                //if a == 1 {
                //return;
                //}
                //self.index.set(i + 1);
            });
            //self.index.set(i + 1);
            // Check if the buffer is full
            //if regs.txdata.is_set(txdata::full) {
            //// If it is, break and wait for the TX interrupt.
            //break;
            //}
        }

        //self.client
        //.transmit_complete(tx_data, hil::uart::Error::CommandComplete);

        //self.client.map(|aclient| {
        //self.buffer.take().map(|a| {
        //aclient.transmit_complete(a, kernel::hil::uart::Error::CommandComplete);
        //});
        //});
        //self.client.map(|client| {
        //self.buffer.take().map(|tx_buffer| {
        //client.transmit_complete(tx_buffer, kernel::hil::uart::Error::CommandComplete);
        //});
        //});

        // Save the buffer so we can keep sending it.
        //self.buffer.replace(tx_data);
        //self.len.set(tx_len);

        // Enable transmissions, and wait until the FIFO is empty before getting
        // an interrupt.
        //let stop_bits = match self.stop_bits.get() {
        //hil::uart::StopBits::One => txctrl::nstop::OneStopBit,
        //hil::uart::StopBits::Two => txctrl::nstop::TwoStopBits,
        //};
        //regs.txctrl
        //.write(txctrl::txen::SET + stop_bits + txctrl::txcnt.val(1));
        //.write(txctrl::txen::SET + stop_bits + txctrl::txcnt.val(1));
    }

    fn receive(&self, rx_buffer: &'static mut [u8], rx_len: usize) {
        //self.client.map(|client| {
        //self.buffer.take().map(|rx_buffer| {
        //client.receive_complete(
        //rx_buffer,
        //rx_len,
        //kernel::hil::uart::Error::CommandComplete,
        //);
        //});
        //});
        //client.receive_complete(tx_buffer, 0, kernel::hil::uart::Error::CommandComplete);
        //let regs = self.registers;

        //// truncate rx_len if necessary
        //let truncated_length = core::cmp::min(rx_len, rx_buf.len());

        //self.rx_remaining_bytes.set(truncated_length);

        //self.offset.set(0);
        //self.rx_buffer.replace(rx_buf);

        //let truncated_uart_max_length = core::cmp::min(truncated_length, 255);

        //self.enable_rx_interrupts();
    }

    fn abort_receive(&self) {
        self.client.map(|aclient| {
            self.buffer.take().map(|a| {
                aclient.transmit_complete(a, kernel::hil::uart::Error::CommandComplete);
            });
        });
        //unimplemented!()
    }
}
