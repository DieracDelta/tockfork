use kernel::common::registers::{ReadOnly, ReadWrite};
use kernel::common::StaticRef;

// memory map as described here: https://sifive.cdn.prismic.io/sifive%2F898b5153-4c06-4085-8d95-2d5fd67e74c6_u54_core_complex_manual_v19_02.pdf
// TODO move this and PLIC somewhere else
#[repr(C)]
struct ClintRegisters {
    //claim: ReadWrite<u32>,
    //_reserved0: u32,
    // Interrupt Priority Register
    //priority: [ReadWrite<u32, Priority::Register>; (0x0C00_0214 - 0x0C00_0004) / 0x4],
    //_reserved1: [u32; (0x0C00_1000 - 0x0C00_0214) / 0x4],
    // Interrupt Pending Register
    misp: ReadWrite<u32, MISP::Register>,
    _reserved0: [u32; (0x200_4000 - 0x200_0004) / 0x4],
    mtimecmp1: ReadWrite<u32, MTIME::Register>,
    mtimecmp2: ReadWrite<u32, MTIME::Register>,
    _reserved1: [u32; (0x200_bff8 - 0x200_4008) / 0x4],
    mtime1: ReadWrite<u32, MTIME::Register>,
    mtime2: ReadWrite<u32, MTIME::Register>,
}

register_bitfields![u32,
    MISP [
        MISPBIT OFFSET(0) NUMBITS(1) []
    ]
];

register_bitfields![u32,
    MTIME [
        MTIMEBITS OFFSET(0) NUMBITS(32) []
    ]
];

const CLINT_BASE: StaticRef<ClintRegisters> =
    unsafe { StaticRef::new(0x200_0000 as *const ClintRegisters) };

//pub unsafe fn read_mtimecmp() -> u64 {
//let clint: &ClintRegisters = &*CLINT_BASE;
//clint.mtimecmp.get()
//}

pub unsafe fn read_mtime() -> u64 {
    let clint: &ClintRegisters = &*CLINT_BASE;
    let a = (clint.mtime2.get() as u64);
    let b = (clint.mtime1.get() as u64);
    a + (b << 32)
}

pub unsafe fn write_mtime(arg: u32) {
    let clint: &ClintRegisters = &*CLINT_BASE;
    clint.mtime1.set(arg);
}

pub unsafe fn write_mtimecmp1(new_bound: u32) {
    let clint: &ClintRegisters = &*CLINT_BASE;
    clint.mtimecmp1.set(new_bound);
}

pub unsafe fn write_mtimecmp2(new_bound: u32) {
    let clint: &ClintRegisters = &*CLINT_BASE;
    clint.mtimecmp2.set(new_bound);
}
pub unsafe fn trigger_software_interrupt() {
    let clint: &ClintRegisters = &*CLINT_BASE;
    clint.misp.write(MISP::MISPBIT::SET);
}
