;
; 3.1.asm
;
; Created: 4/11/2025 10:41:27 μμ
; Author : anast
;

.include "m328PBdef.inc"

;Constant variables that assembler reads first
.equ FOSC_MHz=16                  ; Microcontroller operating frequency in MHz
.equ DEL_mS=50                ; Delay in mS (valid number from 1 to 4095)
.equ DEL_NU=FOSC_MHz*DEL_mS      ; delay_mS routine: (1000*DEL_NU+6) cycles
.def DC_VALUE = r27     ; Holds PWM duty cycle value

.org 0x0000
rjmp reset

    DutyTable:
.db 0x05, 0x14, 0x24, 0x33, 0x42, 0x52, 0x61, 0x70,0x80, 0x8F, 0x9E, 0xAD, 0xBD, 0xCC, 0xDB, 0xEB, 0xFA

reset:
    ; Initialize stack pointer
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPH, r24
							
	; Initialize PC4, PC5 as inputs
    ldi r24, 0b11001111
    out DDRC, r24
    ldi r24, 0b00110000		;keep the PC4,PC5 buttons lit on
    out PORTC,r24
    
	;PB1 is our output but we set everything as output
    ser r24					;PB1 is our output but we set everything as output
    out DDRB, r24
	clr r24
	out PORTB, r24			;disable all leds
			
	;set mode of opperation for counter TCNT1 and set it as non-inverting 
	;set frequency for counter TCNT1
    ldi r24, (0<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (0<<COM1A0)
    sts TCCR1A, r24
    ldi r24, (0<<WGM13) |(1<<WGM12) | (1<<CS12)| (0<<CS11)| (0<<CS10)
    sts TCCR1B, r24

    ldi DC_VALUE, 7
    rcall update_PWM

main:
   in r26,PINC
   com r26        			; Reverse logic, so now if pressed = 1
   andi r26, 0b00110000 	; Mask for PC5, PC4
   cpi  r26, 0b00110000
   breq main
   
   sbrc r26, 4 ;
   rjmp increase
   sbrc r26, 5
   rjmp decrease
   rjmp main

increase:
	ldi r26, PINC
	sbrs r26, 4
	rjmp increase
	
 	cpi DC_VALUE, 17		; max 
    breq end_inc            ; αν DC_VALUE max πήγαινε main
    inc DC_VALUE
    ldi r24, low(DEL_NU)
    ldi r25, high(DEL_NU)	; Set delay (number of cycles)
    rcall delay_mS
	rcall update_PWM

end_inc:
	rjmp main

decrease:
	ldi r26, PINC
	sbrs r26, 5
	rjmp decrease

	cpi DC_VALUE, 1       	; min 
    breq end_dec          	; αν DC_VALUE min πήγαινε main
    dec DC_VALUE
	ldi r24, low(DEL_NU)
    ldi r25, high(DEL_NU) 	; Set delay (number of cycles)
    rcall delay_mS
	rcall update_PWM

end_dec:
    	rjmp main

update_PWM:
	ldi ZH, high(DutyTable)		; Load high byte of table address into ZH
	ldi ZL, low(DutyTable)		; Load low byte of table address into ZL
	add ZL, DC_VALUE			; add the index
	lpm r10,Z					; Loads the value from program memory LOW (DutyTABLE) into R10 by default
	sts OCR1AL,r10
	ret

; delay of 1000*F1+6 cycles (almost equal to 1000*F1 cycles)
delay_mS:
; total delay of next 4 instruction group = 1+(249*4+1)=996 cycles
ldi r23, 249                     ; (1 cycle)
loop_inn:
	dec r23                          ; 1 cycle
	nop                              ; 1 cycle
	brne loop_inn                    ; 1 or 2 cycles
	sbiw r24,1                       ; 2 cycles
	brne delay_mS                    ; 1 or 2 cycles
	ret                              ; 4 cycles