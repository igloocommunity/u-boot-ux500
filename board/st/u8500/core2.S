/*
 * secondary_wfe()
 * Copyright (c) 2009 ST Ericsson
 * Author: Srinidhi Kasagar <srinidhi.kasagar@stericsson.com>	
 *
 * puts the secondary core in wfe. This is required for the locked
 * version of ST Ericsson SMP platform to attach the debugger at this stage.
 * This is a GCC generated code.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
.global secondary_startup
secondary_startup:
	dsb					@ WFI may enter a low-power mode
	wfi
	mov	pc, lr

.global secondary_wfe
secondary_wfe:
.L8:	wfe
	ldr	r0, .L9
	ldr	r1, .L9+4
	ldr	r3, [r0, #4080]
	str	r3, [r1, #0]
	ldr	r3, .L9+8
	ldr	r2, [r1, #0]
	cmp	r2, r3
	bne	.L8
	ldr	r3, [r0, #4084]
	str	r3, [r1, #4]
	blx	r3
	b	.L8
.L10:
	.align	2
.L9:
	.word	-2146103296
	.word	.LANCHOR0
	.word	-1577128703
	.size	secondary_wfe, .-secondary_wfe
	.global	handler
	.global	magic_num
	.bss
	.align	2
.LANCHOR0 = . + 0
	.type	magic_num, %object
	.size	magic_num, 4
magic_num:
	.space	4
	.type	handler, %object
	.size	handler, 4
handler:
	.space	4
	.ident	"GCC: (GNU) 4.2.3"
	.section	.note.GNU-stack,"",%progbits
