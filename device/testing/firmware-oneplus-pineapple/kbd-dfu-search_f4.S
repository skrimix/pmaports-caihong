@ Replacement for sub_408EE0 (touchpad toggle key handler) in KA026_A_1.5.7.
@ Same behaviour as stock, except the press is ignored while Search (0x72) is held.
	.syntax unified
	.thumb
	.cpu cortex-m4
	.org 0
	.equ has_usage,   0x419664	@ (list, usage) -> 1 if usage in pressed list
	.equ pogo_conn,   0x4083F4
	.equ ble_conn,    0x405C20	@ 255 = not connected
	.equ mode_flag,   0x4178D4	@ returns byte at 0x200045F0
	.equ tp_state,    0x40A928	@ 1 = touchpad disabled
	.equ tp_set,      0x40A75C	@ app_tp_disable_by_key(disable)
	.equ BASE,        0x408EE0
	.macro blabs target
	bl (\target - BASE) + start
	.endm
start:
	push	{r4, r5, r6, lr}
	ldr	r4, =0x20002B9D		@ key-down edge state
	ldr	r6, =0x20004644		@ pressed key list
	ldrb	r5, [r4]
	mov	r0, r6
	movs	r1, #0x71
	blabs	has_usage
	cbz	r5, press
	cbnz	r0, ret			@ still held
	strb	r0, [r4]		@ released: state = 0
	b	ret
press:
	cbz	r0, ret
	movs	r0, #1
	strb	r0, [r4]		@ pressed: state = 1
	mov	r0, r6
	movs	r1, #0x72
	blabs	has_usage
	cbnz	r0, ret			@ Search held: no firmware toggle
	blabs	pogo_conn
	cbnz	r0, go
	blabs	ble_conn
	cmp	r0, #0xff
	beq	ret
go:
	blabs	mode_flag
	cbnz	r0, enable
	blabs	tp_state
	cbnz	r0, enable
	movs	r0, #1
	b	call
enable:
	movs	r0, #0
call:
	blabs	tp_set
ret:
	pop	{r4, r5, r6, pc}
	.ltorg
