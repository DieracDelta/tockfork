loop.S:
    .globl _start
_start:
    j l1
l1:
    ecall
    j _start
