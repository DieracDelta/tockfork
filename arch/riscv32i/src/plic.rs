//! Platform Level Interrupt Control

use kernel::common::registers::{ReadOnly, ReadWrite};
use kernel::common::StaticRef;

// memory map as described here: https://sifive.cdn.prismic.io/sifive%2F898b5153-4c06-4085-8d95-2d5fd67e74c6_u54_core_complex_manual_v19_02.pdf
#[repr(C)]
struct PlicRegisters {
    //claim: ReadWrite<u32>,
    _reserved0: u32,
    // Interrupt Priority Register
    priority: [ReadWrite<u32, Priority::Register>; (0x0C00_00D0 - 0x0C00_0004) / 0x4],
    _reserved1: [u32; (0x0C00_1000 - 0x0C00_00D0) / 0x4],
    // Interrupt Pending Register
    pending: [ReadOnly<u32, Pending::Register>; (0x0C00_1008 - 0x0C00_1000) / 0x4],
    _reserved2: [u32; (0x0C00_2000 - 0x0C00_1008) / 0x4],
    // Interrupt machine enable Register
    m_enable: [ReadWrite<u32, Enable::Register>; 2],
    _reserved3: [u32; (0x0C20_0000 - 0x0C00_2008) / 0x4],
    // Interrupt supervisor enable Register
    //s_enable: [ReadWrite<u32, Enable::Register>; (0x0C00_2094 - 0x0C00_2080) / 0x4],
    // Interrupt priority threshold Register
    m_priority_threshold: ReadWrite<u32, PriorityThreshold::Register>,
    // Interrupt claim complete Register
    m_claim_complete: ReadWrite<u32>,
    //_reserved4: [u32; (0x0C20_1000 - 0x0C20_0008) / 0x4],
    // Interrupt priority threshold register
    //s_priority_threshold: ReadWrite<u32, PriorityThreshold::Register>,
    // Interrupt claim complete register
    //s_claim_complete: ReadWrite<u32>,
    //_reserved5: [u32; (0x1000_0000 - 0x0C20_1008) / 0x4],
}

register_bitfields![u32,
Priority [
    PRIORITYBITS OFFSET(0) NUMBITS(3) []
],
Enable [
    ENABLEBITS OFFSET(0) NUMBITS(32) []
],
Pending [
    PENDINGBITS OFFSET(0) NUMBITS(32) []
],
PriorityThreshold [
    THRESHOLDBITS OFFSET(0) NUMBITS(3) []
    //PRIORITYBITS OFFSET(3) NUMBITS(29) [],
]
];

const PLIC_BASE: StaticRef<PlicRegisters> =
    unsafe { StaticRef::new(0x0c00_0000 as *const PlicRegisters) };

pub unsafe fn machine_clear_pending_bit(id: u32) {
    let plic: &PlicRegisters = &*PLIC_BASE;
    let offset = id / 32;
    let bit_index = offset % 32;
    let m_enable = &plic.m_enable[offset as usize];
    m_enable.set(m_enable.get() & (0 << bit_index));
}

/// Clear all pending interrupts.
pub unsafe fn clear_all_pending() {
    //let plic: &PlicRegisters = &*PLIC_BASE;
    //for pending in plic.pending.iter() {
    //pending.set(0);
    //}
}

pub unsafe fn set_priority_threshold(val: u8) {
    let plic: &PlicRegisters = &*PLIC_BASE;
    plic.m_priority_threshold
        .modify(PriorityThreshold::THRESHOLDBITS.val(val as u32));
    // to enable the supervisor level interrupts
    // keeping things simple
    //plic.s_priority_threshold
    //.modify(PriorityThreshold::THRESHOLDBITS.val(0));
}

/// Enable all interrupts.
pub unsafe fn enable_all_sources() {
    let plic: &PlicRegisters = &*PLIC_BASE;
    // don't set the first bit, as RO
    plic.m_enable[0].set(0xFF);
    //plic.m_enable[1].set(0xFFFF_FFFF);
    // only first 20 matter
    //plic.m_enable[2].set(0xFFFF_FFFF);
    //plic.m_enable[3].set(0xFFFF_FFFF);
    //// only first 3 bits matter
    //plic.m_enable[4].set(0x7);

    // Set some default priority for each interrupt. This is not really used
    // at this point.
    // just needs to be nonzero
    for priority in plic.priority.iter() {
        priority.write(Priority::PRIORITYBITS.val(1));
    }
}

/// Disable all interrupts.
pub unsafe fn disable_all_sources() {
    let plic: &PlicRegisters = &*PLIC_BASE;
    for enable in plic.m_enable.iter() {
        enable.set(0);
    }
    //for enable in plic.s_enable.iter() {
    //enable.set(0);
    //}
}

/// Get the index (0-256) of the lowest number pending interrupt, or `None` if
/// none is pending. RISC-V PLIC has a "claim" register which makes it easy
/// to grab the highest priority pending interrupt.
pub unsafe fn claim_m_mode() -> u32 {
    let plic: &PlicRegisters = &*PLIC_BASE;

    plic.m_claim_complete.get()
}

/// Signal that an interrupt is finished being handled. In Tock, this should be
/// called from the normal main loop (not the interrupt handler).
pub unsafe fn complete(index: u32) {
    let plic: &PlicRegisters = &*PLIC_BASE;
    plic.m_claim_complete.set(index);
}

/// Return `true` if there are any pending interrupts in the PLIC, `false`
/// otherwise.
pub unsafe fn has_pending() -> bool {
    let plic: &PlicRegisters = &*PLIC_BASE;
    plic.pending.iter().fold(0, |i, pending| pending.get() | i) != 0
}

pub unsafe fn enable_interrupts() {
    riscvregs::register::mstatus::set_mie();
    //enable software interrupts
    riscvregs::register::mie::set_msoft();
    riscvregs::register::mie::set_ssoft();
    // enable timer interrupts
    riscvregs::register::mie::set_mtimer();
    riscvregs::register::mie::set_stimer();
    // enable external interrupts
    riscvregs::register::mie::set_mext();
    riscvregs::register::mie::set_sext();
    //riscvregs::register::mie::set_lie0();
    //riscvregs::register::mie::set_lie1();
    //riscvregs::register::mie::set_lie2();
    //riscvregs::register::mie::set_lie3();
    //riscvregs::register::mie::set_lie4();
    //riscvregs::register::mie::set_lie5();
    //riscvregs::register::mie::set_lie6();
    //riscvregs::register::mie::set_lie7();
    //riscvregs::register::mie::set_lie8();
    //riscvregs::register::mie::set_lie9();
    //riscvregs::register::mie::set_lie10();
    //riscvregs::register::mie::set_lie11();
    //riscvregs::register::mie::set_lie12();
    //riscvregs::register::mie::set_lie13();
    //riscvregs::register::mie::set_lie14();
    //riscvregs::register::mie::set_lie15();
}

pub unsafe fn disable_interrupts() {
    riscvregs::register::mstatus::clear_mie();
    //enable software interrupts
    riscvregs::register::mie::clear_msoft();
    riscvregs::register::mie::clear_ssoft();
    // enable timer interrupts
    riscvregs::register::mie::clear_mtimer();
    riscvregs::register::mie::clear_stimer();
    // enable external interrupts
    riscvregs::register::mie::clear_mext();
    riscvregs::register::mie::clear_sext();
    riscvregs::register::mie::clear_lie0();
    riscvregs::register::mie::clear_lie1();
    riscvregs::register::mie::clear_lie2();
    riscvregs::register::mie::clear_lie3();
    riscvregs::register::mie::clear_lie4();
    riscvregs::register::mie::clear_lie5();
    riscvregs::register::mie::clear_lie6();
    riscvregs::register::mie::clear_lie7();
    riscvregs::register::mie::clear_lie8();
    riscvregs::register::mie::clear_lie9();
    riscvregs::register::mie::clear_lie10();
    riscvregs::register::mie::clear_lie11();
    riscvregs::register::mie::clear_lie12();
    riscvregs::register::mie::clear_lie13();
    riscvregs::register::mie::clear_lie14();
    riscvregs::register::mie::clear_lie15();
}
