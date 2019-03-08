MEMORY
{
  rom (rx)  : ORIGIN = 0x80000000, LENGTH = 512M
  prog (rx) : ORIGIN = 0x80100000, LENGTH = 507M
  ram (rwx) : ORIGIN = 0x80200000, LENGTH = 16K
}

MPU_MIN_ALIGN = 8K;

INCLUDE ../kernel_layout.ld
