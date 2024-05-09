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

.global Reset_Handler

#define GPIO47 (47)
#define GPIO42 (42)
#define GPIO47_READEN (0x02000000)
#define GPIO42_READEN (0x00020000)
#define GPIO_BASE (0x40010000)
#define PADREGL_OFFSET (0x2C)
#define PADREGK_OFFSET (0x28)
#define PADKEY (0x60)
#define USERCODE_OFFSET (0x10000)

.thumb
.thumb_func
.align 0
.func Reset_Handler
.type Reset_Handler, function
Reset_Handler:
	#Check GPIO47, we need to enable input
	ldr r0, =GPIO_BASE
	#ldr r1, [r0, PADREGL_OFFSET]
	ldr r1, [r0, PADREGK_OFFSET]
	# Set bit to enable GPIO 47 reads
	mov r3, r1
	#orr r1, GPIO47_READEN
	orr r1, GPIO42_READEN
	# Unlock writes to PADCFG
	ldr r2, =0x73
	str r2, [r0, PADKEY]
	# Enable GPIO read
	str r1, [r0, PADREGK_OFFSET]
	#str r1, [r0, PADREGL_OFFSET]
	# Read GPIO
	ldr r1, [r0, 0x84]
	# Disable read
	str r2, [r0, PADKEY]
	#str r3, [r0, PADREGL_OFFSET]
	str r3, [r0, PADREGK_OFFSET]
	# _if gpio47 is set, go to bootloader, else app_jump
	#tst r1, 1<<(GPIO47-32)
	tst r1, 1<<(GPIO42-32)
	beq 1f
    ldr sp, =_sstack
    blx Reset_Handler2
0:
    bkpt
    b 0b
1:
	ldr r0, =USERCODE_OFFSET
	b app_jump
.endfunc

