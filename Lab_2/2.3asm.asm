;
; 2.3Assembly.asm
;
; Created: 23/10/2025 2:20:07 μμ
; Author : anast
;

.include "m328PBdef.inc"

;Initialize the vector table stored in FLASH
.org 0x0
rjmp start
.org 0x4
rjmp ISR1

start:
;Init stack pointer
	ldi r24, LOW(RAMEND)
	out SPL,r24
	ldi r24, HIGH(RAMEND)
	out SPH,r24

;Init of PORTD as read only
	clr r24							;Set r18 bits to 0
	out DDRD, r24					;Set portD as input
	ser r24							;set r18 bits to 1 
	out PORTD, r24					;Pull-up resistors so we dont have floating pins

;Init of PORTB as output only
	ser r24
	out DDRB, r24					;set portB as output only

;Interrupt on rising edge of INT1 pin
	ldi r24,(1 << ISC10) | ( 1 << ISC11)
	sts EICRA, r24 

;Enable the INT1 interrupt(PD3) 
	ldi r24, (1 << INT1)
	out EIMSK, r24

	sei
									;Set the Global interrupt Flag
	clr r24
	clr r25
	clr r26
	clr r27


main:
	clr r16
    out PORTB,r16

loop_renew:
	mov r28,r26
	mov r29,r27
	or r28,r29
	breq loop
	ldi r16, 0b00111111
	out PORTB,r16
	rcall delay_1ms
	sbiw r26,1
	rjmp loop_renew

loop:
	mov r28,r24
	mov r29,r25
	or r28,r29
	breq main
	ldi r16, 0b00001000
	out PORTB,r16
	rcall delay_1ms
	sbiw r24,1
	rjmp loop_renew

ISR1:
	in r20, SREG
	push r20
	push r28
	push r29
rst:
	ldi r20, (1 << INTF1)
	out EIFR, r20           ; Clear INTF1
	rcall delay_1ms
	in r16, EIFR
    sbrc r16, INTF1         ; Skip αν INTF1 = 0
    rjmp rst  

	in r17, PORTB
	cpi r17,0
	brne renew
	clr r26
	clr r27
	ldi r24,LOW(4000)
	ldi r25,HIGH(4000)
	rjmp end_isr
renew:
	ldi r26,LOW(1000)
	ldi r27,HIGH(1000)
	ldi r24,LOW(3000)
	ldi r25,HIGH(3000)
end_isr:
	pop r29
	pop r28
	pop r20
	out SREG, r20
	reti

delay_1ms:
ldi r28,LOW(15992)		;1 cycle
ldi r29,HIGH(15992)		;1 cycle
delay:  
sbiw r28,4				;2 cycle
brne delay				;2 cycle or 1 if last
;at this stage 15993 cycles
reti					;4 cycles
;+3 cycles or rcall so total 16000 cycles = 1 ms delay
