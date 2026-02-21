;
; 1.3.asm
;
; Created: 16/10/2025 1:32:49 μμ
; Author : anast
;

.include "m328pbdef.inc"
.def wagon = r16
.def counter = r17

ldi r24,LOW(RAMEND)
out SPL,r24
ldi r24,HIGH(RAMEND)
out SPH,r24

ser r24
out DDRD,r24
clr r24

ldi wagon, 0x01
ldi r18,0x01
out PORTD,wagon

start:
    ldi counter, 0x07
    BST r18,1				;to MSB: T flag = 0

to_MSB:
	ldi r24,LOW(2000)
	ldi r25,HIGH(2000)
	rcall wait_x_msec		;2 sec delay
	lsl wagon				;move wagon
	out PORTD,wagon
	dec counter
	brne to_MSB
	ldi r24,LOW(1000)
	ldi r25,HIGH(1000)
	rcall wait_x_msec		;another 1 sec delay
	BST r18,0				;to LSB: T flag = 1
	ldi counter, 0x07		;reset counter

to_LSB:
	ldi r24,LOW(2000)
	ldi r25,HIGH(2000)
	rcall wait_x_msec		;2 sec delay
	lsr wagon				;move wagon
	out PORTD,wagon
	dec counter
	brne to_LSB
	ldi r24,LOW(1000)
	ldi r25,HIGH(1000)		;1 sec delay
    rcall wait_x_msec		;another 1 sec delay
	rjmp start

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