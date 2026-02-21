;
; 4.1.asm
;
; Created: 5/11/2025 3:38:16 μμ
; Author : anast
;

.include "m328PBdef.inc"		;ATmega328P microcontroller definition

.equ PD0=0 
.equ PD1=1 
.equ PD2=2 
.equ PD3=3 
.equ PD4=4 
.equ PD5=5 
.equ PD6=6 
.equ PD7=7 
.equ timer_start = 49910		;explained below
.def result = r20				;to store the division result

.org 0x0
    rjmp start
.org 0x1A
    rjmp timer_interupt
.org 0x02A
    rjmp ADC_ready
    
start:
	ldi r29,'0'			;for turning the digits to ASCII characters.
    
	ser r16
	out DDRD,r16		;set DDRD as output
	out DDRB,r16		;set DDRB as output
	clr r16
	out DDRC,r16		;set DDRC as input
	out PORTB,r16		;clear output
	out PORTD,r16		;clear outpout

	out EIMSK,r16		;ensure every other normal interrupt is disabled
	
	rcall lcd_init
	rcall lcd_clear_display
	
	sei					;enable interrupts
	
	ldi r16,LOW(RAMEND)
	out SPL,r16
	ldi r16,HIGH(RAMEND)
	out SPH,r16
	clr r16
	
;pre-building ADC
	ldi r16,0x43		;ADC3 as input
	sts ADMUX,r16
	ldi r16,0x8F		;enable conversion-done interrupt
	sts ADCSRA,r16
	
;initialise counter-timer
	clr r16
	sts TCCR1B,r16		;ensuring counter is not yet counting and is frozen | WGM13,WGM12 -> 0 
	sts TCCR1A,r16		;WGM11,WGM10 -> 0 WE WANT NORMAL FUNCTION...NO PWM
	ldi r16,0x01
	sts TIMSK1,r16		;allowing overflow interrupts
;we choose clk/1024 = 15.625hz and timer has 16 bits so it counts from 0 to 65.535
;to have overflow interrupt every 1 sec we need to start counting from 65.535-1*15.625 = 49910 = timer_start
	
;load TCNT1 with the correct timer_start
	ldi r16,HIGH(timer_start)
	sts TCNT1H,r16
	ldi r16,LOW(timer_start)
	sts TCNT1L,r16
	
	ldi r16,0x05
	sts TCCR1B,r16		;start timer
	
main:
	rjmp main			;wait for overflow interupt


timer_interupt:
	sei
	clr r16
	sts TCCR1B,r16				;stop timer 
	ldi r16,HIGH(timer_start)	;reload TCNT1 to timer_start
	sts TCNT1H,r16
	ldi r16,LOW(timer_start)
	sts TCNT1L,r16
	ldi r16,0x05
	sts TCCR1B,r16				;restart timer
	
	ldi r16,0xCF		;setting ADSC to 1 to start convertion
	sts ADCSRA,r16
	reti				;waiting for the completion interrupt to happen
 
ADC_ready:
	rcall lcd_clear_display
	
	sei

	lds r30 , ADCL
	lds r31 , ADCH
	mov r17,r30
	mov r18,r31
	lsl r30
	rol r31 
	lsl r30
	rol r31				;*4
	
	add r30,r17			;+1
	adc r31,r18			;so we have *5 
	
	ldi result,0
	rcall div_1024		;devide /1024
	mov r24,result
	add r24,r29			;turn result to ASCII character
	rcall lcd_data		;output to LCD

	ldi r24,'.'
	rcall lcd_data		;output the dot for decimals

	mov r17,r30
	mov r18,r31
	lsl r30
	rol r31
	lsl r30
	rol r31
	lsl r30
	rol r31				; x <<  3 = 8*x
	
	add r30,r17			; +x
	adc r31,r18
	
	add r30,r17			; +x
	adc r31,r18			; so we have 8x + x + x = 10x
	
	clr result			;divide with 1024 to find the first decimal
	rcall div_1024
	mov r24,result
	add r24,r29			;turn result to ASCII character
	rcall lcd_data		;output to LCD
	
	mov r17,r30
	mov r18,r31
	lsl r30
	rol r31
	lsl r30
	rol r31
	lsl r30
	rol r31				; x <<  3 = 8*x
	
	add r30,r17			; +x
	adc r31,r18
	
	add r30,r17			; +x
	adc r31,r18			; so we have 8x + x + x = 10x
	
	clr result			;divide with 1024 to find the second decimal
	rcall div_1024
	mov r24,result
	add r24,r29			;turn result to ASCII character
	rcall lcd_data		;output to LCD

	reti

div_1024:
	mov result, r31				;could do 10 right rotations of r30,r31
	andi result, 0b11111100		;or we can keep 6 MSB (/1024)
	lsr result					;and rotate right 2 times
	lsr result
	andi r31, 0b00000011		;we keep the remain
	reti
    
lcd_init:
	ldi r24 ,low(200)
	ldi r25 ,high(200)		; Wait 200 mSec
	rcall wait_msec

	ldi r24 ,0x30			; command to switch to 8 bit mode
	out PORTD ,r24
	sbi PORTD ,PD3			; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250
	ldi r25 ,0				; Wait 250uSec
	rcall wait_usec

	ldi r24 ,0x30			; command to switch to 8 bit mode
	out PORTD ,r24
	sbi PORTD ,PD3			; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250
	ldi r25 ,0				; Wait 250uSec
	rcall wait_usec

	ldi r24 ,0x30			; command to switch to 8 bit mode
	out PORTD ,r24
	sbi PORTD ,PD3			; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250 ;
	ldi r25 ,0				; Wait 250uSec
	rcall wait_usec

	ldi r24 ,0x20			; command to switch to 4 bit mode
	out PORTD ,r24
	sbi PORTD ,PD3			; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250 ;
	ldi r25 ,0				; Wait 250uSec
	rcall wait_usec

	ldi r24 ,0x28			; 5x8 dots, 2 lines
	rcall lcd_command
	ldi r24 ,0x0c			; dislay on, cursor off
	rcall lcd_command
	rcall lcd_clear_display
	ldi r24 ,0x06			; Increase address, no display shift
	rcall lcd_command
	ret
	
write_2_nibbles:
	push r24				; save r24(LCD_Data)
	in r25 ,PIND			; read PIND
	andi r25 ,0x0f
	andi r24 ,0xf0			; r24[3:0] Holds previus PORTD[3:0]
	add r24 ,r25			; r24[7:4] <-- LCD_Data_High_Byte
	out PORTD ,r24

	sbi PORTD ,PD3			; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3

	pop r24					; Recover r24(LCD_Data)
	swap r24
	andi r24 ,0xf0			; r24[3:0] Holds previus PORTD[3:0]
	add r24 ,r25			; r24[7:4] <-- LCD_Data_Low_Byte
	out PORTD ,r24

	sbi PORTD ,PD3			; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ret
		
lcd_data:
	sbi PORTD ,PD2			; LCD_RS=1(PD2=1), Data
	rcall write_2_nibbles	; send data
	ldi r24 ,250
	ldi r25 ,0				; Wait 250uSec
	rcall wait_usec
	ret

lcd_command:
	cbi PORTD ,PD2			; LCD_RS=0(PD2=0), Instruction
	rcall write_2_nibbles	; send Instruction
	ldi r24 ,250
	ldi r25 ,0				; Wait 250uSec
	rcall wait_usec
	ret	

lcd_clear_display: 
	ldi r24 ,0x01			; clear display command
	rcall lcd_command
	ldi r24 ,low(5)
	ldi r25 ,high(5)		; Wait 5 mSec
	rcall wait_msec
	ret

wait_msec:
	push r24				; 2 cycles
	push r25				; 2 cycles
	ldi r24 , low(999)		; 1 cycle
	ldi r25 , high(999)		; 1 cycle
	rcall wait_usec			; 998.375 usec
	pop r25					; 2 cycles
	pop r24					; 2 cycles
	nop						; 1 cycle
	nop						; 1 cycle
	sbiw r24 , 1			; 2 cycles
	brne wait_msec			; 1 or 2 cycles
	ret						; 4 cycle

wait_usec:
	sbiw r24 ,1				; 2 cycles (2/16 usec)
	call delay_8cycles		; 4+8=12 cycles
	brne wait_usec			; 1 or 2 cycles
	ret

delay_8cycles:
	nop
	nop
	nop
	nop
	ret