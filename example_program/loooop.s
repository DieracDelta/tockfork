	.file	"loooop.c"
	.option nopic
	.text
	.align	1
	.globl	main
	.type	main, @function
main:
	addi	sp,sp,-16
	sw	s0,12(sp)
	addi	s0,sp,16
.L2:
	j	.L2
	.size	main, .-main
	.ident	"GCC: (GNU) 8.2.0"
	.section	.note.GNU-stack,"",@progbits
