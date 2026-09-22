;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.2.0 #13081 (Linux)
;--------------------------------------------------------
	.module t8
	.optsdcc -mmcs51 --model-large
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _f
	.globl _f_PARM_3
	.globl _f_PARM_2
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	.area DSEG    (DATA)
_f_sloc0_1_0:
	.ds 4
_f_sloc1_1_0:
	.ds 4
_f_sloc2_1_0:
	.ds 4
_f_sloc3_1_0:
	.ds 4
_f_sloc4_1_0:
	.ds 4
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	.area ISEG    (DATA)
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	.area IABS    (ABS,DATA)
	.area IABS    (ABS,DATA)
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	.area BSEG    (BIT)
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	.area PSEG    (PAG,XDATA)
;--------------------------------------------------------
; external ram data
;--------------------------------------------------------
	.area XSEG    (XDATA)
_f_PARM_2:
	.ds 4
_f_PARM_3:
	.ds 4
_f_x_65536_1:
	.ds 4
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area XABS    (ABS,XDATA)
;--------------------------------------------------------
; external initialized ram data
;--------------------------------------------------------
	.area XISEG   (XDATA)
	.area HOME    (CODE)
	.area GSINIT0 (CODE)
	.area GSINIT1 (CODE)
	.area GSINIT2 (CODE)
	.area GSINIT3 (CODE)
	.area GSINIT4 (CODE)
	.area GSINIT5 (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area CSEG    (CODE)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME    (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area GSINIT  (CODE)
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME    (CODE)
	.area HOME    (CODE)
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CSEG    (CODE)
;------------------------------------------------------------
;Allocation info for local variables in function 'f'
;------------------------------------------------------------
;sloc0                     Allocated with name '_f_sloc0_1_0'
;sloc1                     Allocated with name '_f_sloc1_1_0'
;sloc2                     Allocated with name '_f_sloc2_1_0'
;sloc3                     Allocated with name '_f_sloc3_1_0'
;sloc4                     Allocated with name '_f_sloc4_1_0'
;y                         Allocated with name '_f_PARM_2'
;z                         Allocated with name '_f_PARM_3'
;x                         Allocated with name '_f_x_65536_1'
;a                         Allocated with name '_f_a_65536_2'
;b                         Allocated with name '_f_b_65536_2'
;c                         Allocated with name '_f_c_65536_2'
;d                         Allocated with name '_f_d_65536_2'
;e                         Allocated with name '_f_e_65536_2'
;g                         Allocated with name '_f_g_65536_2'
;h                         Allocated with name '_f_h_65536_2'
;i                         Allocated with name '_f_i_65536_2'
;j                         Allocated with name '_f_j_65536_2'
;k                         Allocated with name '_f_k_65536_2'
;l                         Allocated with name '_f_l_65536_2'
;m                         Allocated with name '_f_m_65536_2'
;------------------------------------------------------------
;	t8.c:2: int16_t f(int32_t x,int32_t y,int32_t z) {
;	-----------------------------------------
;	 function f
;	-----------------------------------------
_f:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
	mov	r7,dpl
	mov	r6,dph
	mov	r5,b
	mov	r4,a
	mov	dptr,#_f_x_65536_1
	mov	a,r7
	movx	@dptr,a
	mov	a,r6
	inc	dptr
	movx	@dptr,a
	mov	a,r5
	inc	dptr
	movx	@dptr,a
	mov	a,r4
	inc	dptr
	movx	@dptr,a
;	t8.c:3: int32_t a=x*7,b=y*7,c=z*7,d=a+b,e=c-d,g=e>>3,h=g+a,i=h-b,j=i+c,k=j-d,l=k+e,m=l-g;
	mov	dptr,#_f_x_65536_1
	movx	a,@dptr
	mov	r4,a
	inc	dptr
	movx	a,@dptr
	mov	r5,a
	inc	dptr
	movx	a,@dptr
	mov	r6,a
	inc	dptr
	movx	a,@dptr
	mov	r7,a
	mov	dptr,#__mullong_PARM_2
	mov	a,r4
	movx	@dptr,a
	mov	a,r5
	inc	dptr
	movx	@dptr,a
	mov	a,r6
	inc	dptr
	movx	@dptr,a
	mov	a,r7
	inc	dptr
	movx	@dptr,a
	mov	dptr,#(0x07&0x00ff)
	clr	a
	mov	b,a
	lcall	__mullong
	mov	r4,dpl
	mov	r5,dph
	mov	r6,b
	mov	r7,a
	mov	dptr,#_f_PARM_2
	movx	a,@dptr
	mov	r0,a
	inc	dptr
	movx	a,@dptr
	mov	r1,a
	inc	dptr
	movx	a,@dptr
	mov	r2,a
	inc	dptr
	movx	a,@dptr
	mov	r3,a
	mov	dptr,#__mullong_PARM_2
	mov	a,r0
	movx	@dptr,a
	mov	a,r1
	inc	dptr
	movx	@dptr,a
	mov	a,r2
	inc	dptr
	movx	@dptr,a
	mov	a,r3
	inc	dptr
	movx	@dptr,a
	mov	dptr,#(0x07&0x00ff)
	clr	a
	mov	b,a
	push	ar7
	push	ar6
	push	ar5
	push	ar4
	lcall	__mullong
	mov	_f_sloc0_1_0,dpl
	mov	(_f_sloc0_1_0 + 1),dph
	mov	(_f_sloc0_1_0 + 2),b
	mov	(_f_sloc0_1_0 + 3),a
	mov	dptr,#_f_PARM_3
	movx	a,@dptr
	mov	r0,a
	inc	dptr
	movx	a,@dptr
	mov	r1,a
	inc	dptr
	movx	a,@dptr
	mov	r2,a
	inc	dptr
	movx	a,@dptr
	mov	r3,a
	mov	dptr,#__mullong_PARM_2
	mov	a,r0
	movx	@dptr,a
	mov	a,r1
	inc	dptr
	movx	@dptr,a
	mov	a,r2
	inc	dptr
	movx	@dptr,a
	mov	a,r3
	inc	dptr
	movx	@dptr,a
	mov	dptr,#(0x07&0x00ff)
	clr	a
	mov	b,a
	lcall	__mullong
	mov	_f_sloc1_1_0,dpl
	mov	(_f_sloc1_1_0 + 1),dph
	mov	(_f_sloc1_1_0 + 2),b
	mov	(_f_sloc1_1_0 + 3),a
	pop	ar4
	pop	ar5
	pop	ar6
	pop	ar7
	mov	a,_f_sloc0_1_0
	add	a,r4
	mov	_f_sloc2_1_0,a
	mov	a,(_f_sloc0_1_0 + 1)
	addc	a,r5
	mov	(_f_sloc2_1_0 + 1),a
	mov	a,(_f_sloc0_1_0 + 2)
	addc	a,r6
	mov	(_f_sloc2_1_0 + 2),a
	mov	a,(_f_sloc0_1_0 + 3)
	addc	a,r7
	mov	(_f_sloc2_1_0 + 3),a
	mov	a,_f_sloc1_1_0
	clr	c
	subb	a,_f_sloc2_1_0
	mov	_f_sloc3_1_0,a
	mov	a,(_f_sloc1_1_0 + 1)
	subb	a,(_f_sloc2_1_0 + 1)
	mov	(_f_sloc3_1_0 + 1),a
	mov	a,(_f_sloc1_1_0 + 2)
	subb	a,(_f_sloc2_1_0 + 2)
	mov	(_f_sloc3_1_0 + 2),a
	mov	a,(_f_sloc1_1_0 + 3)
	subb	a,(_f_sloc2_1_0 + 3)
	mov	(_f_sloc3_1_0 + 3),a
	mov	r0,_f_sloc3_1_0
	mov	a,(_f_sloc3_1_0 + 1)
	swap	a
	rl	a
	xch	a,r0
	swap	a
	rl	a
	anl	a,#0x1f
	xrl	a,r0
	xch	a,r0
	anl	a,#0x1f
	xch	a,r0
	xrl	a,r0
	xch	a,r0
	mov	r1,a
	mov	a,(_f_sloc3_1_0 + 2)
	swap	a
	rl	a
	anl	a,#0xe0
	orl	a,r1
	mov	r1,a
	mov	r2,(_f_sloc3_1_0 + 2)
	mov	a,(_f_sloc3_1_0 + 3)
	swap	a
	rl	a
	xch	a,r2
	swap	a
	rl	a
	anl	a,#0x1f
	xrl	a,r2
	xch	a,r2
	anl	a,#0x1f
	xch	a,r2
	xrl	a,r2
	xch	a,r2
	jnb	acc.4,00103$
	orl	a,#0xe0
00103$:
	mov	r3,a
	mov	a,r4
	add	a,r0
	mov	_f_sloc4_1_0,a
	mov	a,r5
	addc	a,r1
	mov	(_f_sloc4_1_0 + 1),a
	mov	a,r6
	addc	a,r2
	mov	(_f_sloc4_1_0 + 2),a
	mov	a,r7
	addc	a,r3
	mov	(_f_sloc4_1_0 + 3),a
	mov	a,_f_sloc4_1_0
	clr	c
	subb	a,_f_sloc0_1_0
	mov	_f_sloc0_1_0,a
	mov	a,(_f_sloc4_1_0 + 1)
	subb	a,(_f_sloc0_1_0 + 1)
	mov	(_f_sloc0_1_0 + 1),a
	mov	a,(_f_sloc4_1_0 + 2)
	subb	a,(_f_sloc0_1_0 + 2)
	mov	(_f_sloc0_1_0 + 2),a
	mov	a,(_f_sloc4_1_0 + 3)
	subb	a,(_f_sloc0_1_0 + 3)
	mov	(_f_sloc0_1_0 + 3),a
	mov	a,_f_sloc1_1_0
	add	a,_f_sloc0_1_0
	mov	_f_sloc1_1_0,a
	mov	a,(_f_sloc1_1_0 + 1)
	addc	a,(_f_sloc0_1_0 + 1)
	mov	(_f_sloc1_1_0 + 1),a
	mov	a,(_f_sloc1_1_0 + 2)
	addc	a,(_f_sloc0_1_0 + 2)
	mov	(_f_sloc1_1_0 + 2),a
	mov	a,(_f_sloc1_1_0 + 3)
	addc	a,(_f_sloc0_1_0 + 3)
	mov	(_f_sloc1_1_0 + 3),a
	mov	a,_f_sloc1_1_0
	clr	c
	subb	a,_f_sloc2_1_0
	mov	_f_sloc2_1_0,a
	mov	a,(_f_sloc1_1_0 + 1)
	subb	a,(_f_sloc2_1_0 + 1)
	mov	(_f_sloc2_1_0 + 1),a
	mov	a,(_f_sloc1_1_0 + 2)
	subb	a,(_f_sloc2_1_0 + 2)
	mov	(_f_sloc2_1_0 + 2),a
	mov	a,(_f_sloc1_1_0 + 3)
	subb	a,(_f_sloc2_1_0 + 3)
	mov	(_f_sloc2_1_0 + 3),a
	mov	r5,_f_sloc2_1_0
	mov	r7,(_f_sloc2_1_0 + 1)
	mov	r4,_f_sloc3_1_0
	mov	r6,(_f_sloc3_1_0 + 1)
	mov	a,r4
	add	a,r5
	mov	r5,a
	mov	a,r6
	addc	a,r7
	mov	r7,a
	mov	a,r5
	clr	c
	subb	a,r0
	mov	r5,a
	mov	a,r7
	subb	a,r1
	mov	r7,a
;	t8.c:4: return (int16_t)(m+h+i+j+k);
	mov	r4,_f_sloc4_1_0
	mov	r6,(_f_sloc4_1_0 + 1)
	mov	a,r4
	add	a,r5
	mov	r5,a
	mov	a,r6
	addc	a,r7
	mov	r7,a
	mov	r4,_f_sloc0_1_0
	mov	r6,(_f_sloc0_1_0 + 1)
	mov	a,r4
	add	a,r5
	mov	r5,a
	mov	a,r6
	addc	a,r7
	mov	r7,a
	mov	r4,_f_sloc1_1_0
	mov	r6,(_f_sloc1_1_0 + 1)
	mov	a,r4
	add	a,r5
	mov	r5,a
	mov	a,r6
	addc	a,r7
	mov	r7,a
	mov	r4,_f_sloc2_1_0
	mov	r6,(_f_sloc2_1_0 + 1)
	mov	a,r4
	add	a,r5
	mov	r5,a
	mov	a,r6
	addc	a,r7
;	t8.c:5: }
	mov	dpl,r5
	mov	dph,a
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
