;
; 2.2B.asm
;
; Created: 28/10/2025 12:49:51 μμ
; Author : anast
;


.include "m328PBdef.inc"			;Atmega328PB microcontroller definitions 

.equ FOSC_MHZ=16					;Microcontroller frequency- 16MHz
.equ DEL_mS=2000					;Delay in mS 
.equ DEL_NU=FOSC_MHZ*DEL_mS			;delay_mS routine 

.org 0x0
rjmp reset
.org 0x2
rjmp ISR0

reset:
;Init Stack Pointer
	ldi r23, LOW(RAMEND)
	out SPL, r24
	ldi r24, HIGH(RAMEND)
	out SPH, r24

;Init of PORTB as read only
	clr r26							;Set r26 bits to 0
	out DDRB, r26					;Set portD as read only
	ser r26							;set r26 bits to 1 
	out PORTD, r26					;Pull-up resistors so we dont have floating pins

;Init PORTC as output
	ser r26
	out DDRC, r26

;Interrupt on rising edge of INT1 pin
	ldi r24,(1 << ISC01) | ( 1 << ISC00)
	sts EICRA, r24 

;Enable the INT0 interrupt(PD2) 
	ldi r24, (1 << INT0)
	out EIMSK, r24
	sei								;Set the Global interrupt Flag

main:
	rjmp main
		
ISR0:
	ldi r16,0
	ldi r25,4
	in r24,PINB						;r24 reads portb
	com r24							;leds are negative logic
	andi r24,0x1E					;mask r24 so we can read the right pins
	lsr	r24							;shift right to bring PB4-PB1 to PB3-PB0

check:
	lsr	r24							;check for PB1,PB2,PB3,PB4
	brcs counter					;if c=1 then its pressed so go to counter
	
decrease_function:
	dec r25
	brne check						;if r25 is not 0 go to check else you have finished
	out PORTC, r16					;light up bits
	reti

counter: 
	lsl r16							;shift left by 1 
	ori r16, 0x01					;enable that pin
	rjmp decrease_function	 
	
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