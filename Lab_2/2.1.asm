; 2.1.asm
;
; Created: 28/10/2025 12:32:10 μμ
; Author : anast
;

.include "m328PBdef.inc"

;Constant variables that assembler reads first
.equ FOSC_MHZ=16					;Microcontroller op frequency
.equ DEL_mS=500						;Delay in mS(only valid for 1 to 4095)
.equ DEL_miliS=5					;Delay in MS
.equ DEL_NU=FOSC_MHZ*DEL_mS			;delay_mS routine: (1000*DEL_NU + 6)cycles the extra 6 cycles dont bother us
.equ DEL_nanoU=FOSC_MHZ*DEL_miliS	;delay_MS routine:

;Initialize the vector table stored in FLASH
.org 0x0
rjmp reset
.org 0x4
rjmp ISR1

.def intr_counter=r16
.def pause_check=r17

reset:
;Init stack pointer
	ldi r24, LOW(RAMEND)
	out SPL,r24
	ldi r24, HIGH(RAMEND)
	out SPH,r24

	ldi intr_counter, 0				;Initialize interrupt counter to 0 

;Init of PORTD as read only
	clr r18							;Set r18 bits to 0
	out DDRD, r18					;Set portD as read only
	ser r18							;set r18 bits to 1 
	out PORTD, r18					;Pull-up resistors so we dont have floating pins

;Init of PORTB as output only
	out DDRC, r18					;set portC as output only

;Interrupt on rising edge of INT1 pin
	ldi r24,(1 << ISC10) | ( 1 << ISC11)
	sts EICRA, r24 


;Enable the INT1 interrupt(PD3) 
	ldi r24, (1 << INT1)
	out EIMSK, r24
	sei								;Set the Global interrupt Flag

main:
	in r18, PIND					;read portD 
	lsr r18							;logical shift right to check 1st bit
	lsr r18
	brcc wait						;If carry flag is 0 then pd1 is pressed so wait
	sei
	rjmp main

wait:
	cli 
	rjmp main

ISR1:
;set INTF1 bit to 0 
intf1_check:
	ldi r21, (1<<INTF1)				; set intf1 to 0 so we can check
	out EIFR,r21					; Set the bit intf1 to 0
	ldi r24, low(DEL_nU)			
	ldi r25, high(DEL_nU)			;set delay
	rcall delay_MS

;check INTF1 bit
	in r21,EIFR
	lsr r21
	lsr r21
	brcs intf1_check

	lsl intr_counter				;shift intr_counter 1 place to light-up PC1-PC5
	out PORTC, intr_counter
	
	lsr intr_counter				;place intr_counter to first position
	inc intr_counter				;Intrerrupt occured so increase int_counter
	mov r20, intr_counter

	subi r20, 31					;subtract 31 
	breq before_reset				; if carry flag is 0 then jump to reset
	reti

before_reset: 
	ldi intr_counter, 0				;Initialize interrupt counter to 0
	reti

;delay of 1000F1_6 cycles 
delay_mS:
;total delay of next 4 instruction group= 1+(249*4-1)= 996 cycles
	ldi r23, 249
loop_inn:
	dec r23
	nop
	brne loop_inn
	sbiw r24,1
	brne delay_mS
	ret 

;delay of 1000F1_6 cycles 
delay_miliS:
;total delay of next 4 instruction group= 1+(249*4-1)= 996 cycles
	ldi r23, 249
loop_in:
	dec r23
	nop
	brne loop_in
	sbiw r26,1
	brne delay_miliS
	ret 