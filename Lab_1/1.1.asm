;
; 1.1.asm
;
; Created: 15/10/2025 7:42:11 μμ
; Author : anast
;

.include "m328PBdef.inc"
.equ D=3000		;ms


ldi r24,LOW(RAMEND)
out SPL,r24
ldi r25,HIGH(RAMEND)
out SPH,r24

ser r24
out DDRD,r24
clr r29
out PORTD,r29
jmp main

main:
	ldi r24,LOW(D)
	ldi r25,HIGH(D)
	rcall wait_x_msec	;3 cycles
	com r29
	out portd,r29
	rjmp main

wait_x_msec:
	ldi r26,LOW(15988)	;1 cycle
	ldi r27,HIGH(15988)	;1 cycle

delay:
	sbiw r26,4		;2 cycles
	brne delay		;2 cycles or 1 if last
					;delay total 15987 cycles

;at this stage 15989 cycles

	sbiw r24,1		;2 cycles
	breq if_last	;1 cycle or 2 if branch

;at this stage 15992 cycles

	nop
	nop
	nop
	nop
	nop
	nop				;6 cycles

;at this stage 15998 cycles

	rjmp wait_x_msec ;2 cycles and total 16000 cycles (1 msec)

if_last:			;at this stage 15993 cycles
	ret				;4 cycles

;rcall is 3 cycles so total at this stage 16000 cycles (1 msec)
;so after the completion of the function we have x msec delay