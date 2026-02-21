;
; 1.2.asm
;
; Created: 16/10/2025 12:36:15 μμ
; Author : anast
;


.include "m328pbdef.inc"
.def A=r16
.def B=r17
.def C=r18
.def D=r19
.def temp=r20
.def counter=r21
.def F0=r22
.def F1=r23


ldi r24,LOW(RAMEND)
out SPL,r24
ldi r24,HIGH(RAMEND)
out SPH,r24
clr r24


start:
    ldi A,0x51
	ldi B,0x40
	ldi C,0x1F
	ldi D,0xFE
	ldi counter,0x06

	ser r24
	out DDRD,r24;

loop:
	subi A,-1
	subi B,-2
	subi C,-3
	subi D,-4
	mov temp,A
	com temp		;A'
	and temp,B
	mov F0,temp		;F0=A'B
	mov temp,B
	com temp
	and temp,D
	or F0,temp
	com F0			;F0=(A'B+B'D)'
	
	out PORTD,F0

	mov temp,A
	or temp,C
	mov F1,temp		;F1=A+C
	mov temp,B
	or temp,D
	and F1,temp		;F1=(A+C)(B+D)
	
	out PORTD,F1

	dec counter
	brne loop

    rjmp start
