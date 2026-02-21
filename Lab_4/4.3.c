/*
 * main.c
 *
 * Created: 11/6/2025 1:15:35 PM
 *  Author: anast
 */ 

#define F_CPU 16000000UL	// 16 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define SENSITIVITY 0.0129		// Sensitivity in A/ppm
#define VREF 5.0				// Reference voltage in Volts
#define Vgas0 0.1				// Vgas0 in Volts
#define CO_threshold 75			// Threshold in ppm

uint16_t timer_start = 63972 ;

volatile float V_in = 0.0;
volatile int CO_ppm = 0;
volatile uint8_t leds = 0x00;
volatile int blink = 0;
volatile int first_time = 1;

void write_2_nibbles(uint8_t lcd_data) {
	uint8_t temp;

// Send the high nibble
	temp = (PIND & 0x0F) | (lcd_data & 0xF0);	// Keep lower 4 bits of PIND and set high nibble of lcd_data
	PORTD = temp;								// Output the high nibble to PORTD
	PORTD |= (1 << PD3);						// Enable pulse high
	_delay_us(1);								// Small delay to let the signal settle
	PORTD &= ~(1 << PD3);						// Enable pulse low

// Send the low nibble
	lcd_data <<= 4; 							// Move low nibble to high nibble position
	temp = (PIND & 0x0F) | (lcd_data & 0xF0);	// Keep lower 4 bits of PIND and set high nibble of new lcd_data
	PORTD = temp;								// Output the low nibble to PORTD
	PORTD |= (1 << PD3);						// Enable pulse high
	_delay_us(1);								// Small delay to let the signal settle
	PORTD &= ~(1 << PD3);						// Enable pulse low
}

void lcd_data(uint8_t data)
{
	PORTD |= 0x04;				// LCD_RS = 1, (PD2 = 1) -> For Data
	write_2_nibbles(data);		// Send data
	_delay_ms(1);
	return;
}

void lcd_command(uint8_t data)
{
	PORTD &= 0xFB;				// LCD_RS = 0, (PD2 = 0) -> For Instruction
	write_2_nibbles(data);		// Send data
	_delay_ms(1);
	return;
}

void lcd_clear_display()
{
	uint8_t clear_disp = 0x01;	// Clear display command
	lcd_command(clear_disp);
	_delay_ms(5);				// Wait 5 msec
	return;
}

void lcd_init() {
	_delay_ms(200);				// Wait 5 msec

// Send 0x30 command to set 8-bit mode (three times)
	PORTD = 0x30;				// command to switch to 8-bit mode
	PORTD |= (1 << PD3);		// Enable pulse
	_delay_us(1);
	PORTD &= ~(1 << PD3);		// Clear enable
	_delay_us(250);				// Wait 250 µs

	PORTD = 0x30;              // Repeat command to ensure mode set
	PORTD |= (1 << PD3);
	_delay_us(1);
	PORTD &= ~(1 << PD3);
	_delay_us(250);

	PORTD = 0x30;              // Repeat once more
	PORTD |= (1 << PD3);
	_delay_us(1);
	PORTD &= ~(1 << PD3);
	_delay_us(250);

// Send 0x20 command to switch to 4-bit mode
	PORTD = 0x20;
	PORTD |= (1 << PD3);
	_delay_us(1);
	PORTD &= ~(1 << PD3);
	_delay_us(30);

// Set 4-bit mode, 2 lines, 5x8 dots
	lcd_command(0x28);

// Display ON, Cursor OFF
	lcd_command(0x0C);

// Clear display
	lcd_clear_display();

// Entry mode: Increase cursor, no display shift
	lcd_command(0x06);
}

void gas_detected()
{
	lcd_clear_display();
	lcd_data('G');
	lcd_data('A');
	lcd_data('S');
	lcd_data(' ');
	lcd_data('D');
	lcd_data('E');
	lcd_data('T');
	lcd_data('E');
	lcd_data('C');
	lcd_data('T');
	lcd_data('E');
	lcd_data('D');
	return;
}

void gas_clear()
{
	lcd_clear_display();
	lcd_data('C');
	lcd_data('L');
	lcd_data('E');
	lcd_data('A');
	lcd_data('R');
	return;
}

uint8_t calc_leds(int CO_ppm)
{
	if (CO_ppm <= 15) return 0x00;		// if Cx <= 15ppm, open none
	if (CO_ppm <= 35) return 0x01;		// if Cx <= 35ppm, open PB0
	if (CO_ppm <= 75) return 0x03;		// if Cx <= 75ppm, open PB0-PB1
	if (CO_ppm <= 175) return 0x07;		// if Cx <= 175ppm, open PB0-PB2 -> GAS DETECTED
	if (CO_ppm <= 275) return 0x0F;		// if Cx <= 275ppm, open PB0-PB3
	if (CO_ppm <= 375) return 0x1F;		// if Cx <= 375ppm, open PB0-PB4
	return 0x3F;						// if Cx > 375ppm, open PB0-PB5
}

int calc_CO()
{
    V_in = (ADC * VREF) / 1024.0;                   // normalize ADC to VREF (float)
    CO_ppm = (int)((V_in - Vgas0) / SENSITIVITY);   // convert float to int (from the link provided page 3)
    return CO_ppm;
}

ISR(TIMER1_OVF_vect)
{
	cli();
	TCNT1 = timer_start;
	ADCSRA |= 0x40;							//start ADC
	return;
}

// Interrupt routine for ADC
ISR(ADC_vect)
{
	cli();
	CO_ppm = calc_CO();						// Calculate CO concentration
	leds = calc_leds(CO_ppm);
	
	if (CO_ppm > CO_threshold)
	{
		first_time = 0;
		if(blink==0)gas_detected();
		blink = 1;							// Blink necessary leds until CO_ppm drops below 75ppm
		return;
	}
	else if (first_time==1)
	{
		lcd_clear_display();
		return;
	}
	else
	{
		if(blink==1)gas_clear();
		blink = 0;
		return;
	}
}

int main(){
	lcd_init();

	sei();				//enable interrupts
	DDRB = 0x3F;		//set PB0-PB5 as output
	PORTB = 0x00;

	DDRC = 0x00;		//ADC

	DDRD = 0xFF;		//LCD
	PORTD = 0x00;

//ADC enable
	ADMUX = 0x43;				//input from POT4 and REFS0 = 1 for Voltage reference 
	ADCSRA = 0x8F;				//enable interrupts from ADC

//time set up
	TCCR1B = 0x00;				//freeze timer
	TIMSK1 = 0x01;				//allowing overflow interrupt
	TCNT1 = timer_start;		//prescaler = 1024 so 16 MHZ/1024 = 15625 cycles for 1 s
								// 0,1 * 15625 = 1562,5 cycles for 100 ms 
								// 65535 - 1563 = 63972 (=timer_start)
	TCCR1B = 0x05;				//start timer with 16000000/1024=15.625 hz
	lcd_clear_display();
	while(1)
	{
		while(blink == 0)
		{
			PORTB = leds;                // Steady leds 
			sei();
		}
		while(blink == 1)
		{
			PORTB = leds;                // Blinking leds
			_delay_ms(50);
			PORTB = 0x00;
			_delay_ms(50);
			sei();
		}
	}
}