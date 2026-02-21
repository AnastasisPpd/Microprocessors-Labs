;
; 2.2A.asm
;
; Created: 28/10/2025 12:46:15 μμ
; Author : anast
;


.include "m328PBdef.inc"			;Atmega328PB microcontroller definitions 

.equ FOSC_MHZ=16					;Microcontroller frequency- 16MHz
.equ DEL_mS=1000					;Delay in mS 
.equ DEL_NU=FOSC_MHZ*DEL_mS			;delay_mS routine 

;Init Stack Pointer
	ldi r23, LOW(RAMEND)
	out SPL, r24
	ldi r24, HIGH(RAMEND)
	out SPH, r24

;Init PORTC as output
	ser r26
	out DDRC, r26

loop1:
	clr r26
loop2:
	lsl r26
	out PORTC, r26
	lsr r26
	ldi r24, low(DEL_NU)
	ldi r25, high(DEL_NU)			;set delay 
	rcall delay_mS
	
	inc r26
	cpi r26, 31						;compare with 31
	breq loop1
	rjmp loop2 
	
;delay of 1000*f1+6 cycles 
delay_mS:
;total delay of next 4 instruction group= 1+ (249*4-1)=996 cycles
	ldi r23, 249
loop_inn:
	dec r23
	nop
	brne loop_inn
	sbiw r24,1
	brne delay_mS
	ret