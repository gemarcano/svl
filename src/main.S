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
	# Have LR point to the reset vector of the application
	ldr lr, [r0, 4]
	# Update the stack pointer to that of the application
	ldr sp, [r0, 0]
	# Just jump, don't bother linking LR
	bx lr
.endfunc
