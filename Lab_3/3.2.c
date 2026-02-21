/*
 * main.c
 *
 * Created: 10/29/2025 6:24:36 PM
 *  Author: anast
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

//Table with the duty cycle values (min 2%, max 98%)
const int DutyTable[] = {5, 20, 36, 51, 66, 82, 97, 112, 128, 143, 158, 173, 189, 204, 219, 235, 250};

int main()
{	
	//set TMR1A in fast PWM 8 bit mode with non-inverted output
	TCCR1A = (1 << WGM10) | (1 << COM1A1);2
	TCCR1B = (1 << WGM12) | (1 << CS10);
	
	//set ADC with Vref = AVcc, ADC0 channel
	ADMUX = (1 << REFS0) | (1 << MUX0);
	// enable ADC, prescaler 128
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	DDRB = 0xFF;		//set PB1 as output
	DDRC = 0x00;		//set PORTC as input
	PORTC = 0xFF;		//pull-up resistrors
	DDRD = 0xFF;		//set PORTD as output
	PORTD = 0x00;		//clear output
	
	int DC_VALUE=8;		//start for 50% DC_VALUE
	uint16_t adc_sum;
	
	while(1)
	{
		adc_sum = 0;
		for(int i=0; i<16; i++){
		OCR1AL = DutyTable[DC_VALUE];	//output PWM to PB1 LED
		if (!(PINC & (1<<4))){			//check if PC4 is pressed
			_delay_ms(100);
			while (!(PINC & (1<<4)));	//debounce
			
			if (DC_VALUE<16){
				DC_VALUE += 1;			//+6%
			}
		}
		if (!(PINC & (1<<5))){			//check if PC5 is pressed
			_delay_ms(100);
			while (!(PINC & (1<<5)));	//debounce
			
			if (DC_VALUE>0){
				DC_VALUE -= 1;			//-6%
			}
		}
		
		ADCSRA |= (1 << ADSC);			//start ADC
		while (ADCSRA & (1 << ADSC));
		adc_sum += ADC;					//sum
		
		}
		adc_sum = adc_sum >> 4;			// sum/16

			PORTD = 0x00;
			if (adc_sum <= 200) {		//output to PORTD
				PORTD |= (1 << PD0);
			} else if (adc_sum <= 400) {
				PORTD |= (1 << PD1);
			} else if (adc_sum <= 600) {
				PORTD |= (1 << PD2);
			} else if (adc_sum <= 800) {
				PORTD |= (1 << PD3);
			} else {
				PORTD |= (1 << PD4);
		    }
	}
}