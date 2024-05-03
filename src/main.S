.syntax unified
.arch armv7e-m

.global app_jump

.text
.thumb
.thumb_func
.align 0
.func app_jump
.type app_jump, function
app_jump:
	mov lr, r0
	bx r0
app_jump_hang:
	bkpt
	b app_jump_hang
.endfunc
