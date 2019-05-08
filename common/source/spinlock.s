	.arch	armv5te    @ BUGGED: that would be okay for ARM9, but the
	.cpu	arm946e-s  @ spinlock functions are ALSO used on ARM7

	.text
	.arm 
	.global SLasm_Acquire
	.type	SLasm_Acquire STT_FUNC
@---------------------------------------------------------------------------------
SLasm_Acquire:  @ in: r0=sgWifiAp, r1=SPINLOCK_VALUE, out: r0
@---------------------------------------------------------------------------------
	ldr	r2,[r0]
	cmp	r2,#0
	movne	r0,#1
	bxne	lr
	mov	r2,r1
	swp r2,r2,[r0]
	cmp r2,#0
	cmpne r2,r1
	moveq r0,#0
	bxeq lr
	swp r2,r2,[r0]
	mov r0,#1
	bx lr

	.global SLasm_Release   
	.type	SLasm_Release STT_FUNC
@---------------------------------------------------------------------------------
SLasm_Release:  @ in: r0=ptr+sgWifiAp_spinlock, r1=SPINLOCK_VALUE, out: r0     
@---------------------------------------------------------------------------------
	ldr r2,[r0]
	cmp r2,r1
	movne r0,#2
	bxne lr
	mov r2,#0
	swp r2,r2,[r0]
	cmp r2,r1
	moveq r0,#0
	movne r0,#2
	bx lr		                

	.pool
	.end

