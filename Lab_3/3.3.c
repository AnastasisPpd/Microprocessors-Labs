/*
 * main.c
 *
 * Created: 10/30/2025 2:26:23 PM
 *  Author: anast
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
//Table with the duty cycle values (min 2%, max 98%)
const int DutyTable[] = {5, 20, 36, 51, 66, 82, 97, 112, 128, 143, 158, 173, 189, 204, 219, 235, 250};
int mode = 0;

void check_mode()
{
	if (!(PIND & (1<<0))){			//check if PD0 pressed
		_delay_ms(50);
		while (!(PIND & (1<<0)));	//debounce
		mode = 0;
	}
	if (!(PIND & (1<<1))){			//check if PD1 pressed
		_delay_ms(50);
		while (!(PIND & (1<<1)));	//debounce
		mode = 1;
	}
	return;
}


int main()
{
	//set TMR1A in fast PWM 8 bit mode with non-inverted output
	TCCR1A = (1 << WGM10) | (1 << COM1A1);
	TCCR1B = (1 << WGM12) | (1 << CS10);
	
	//set ADC with Vref = AVcc, ADC0 channel
	ADMUX = (1 << REFS0);
	// enable ADC, prescaler 128
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	DDRB = 0xFF;		//set PB1 as output and other pins as input
	DDRC = 0x00;		//set PORTC as input
	PORTC = 0xFF;		//pull-up resistrors
	DDRD = 0x00;		//set PORTD as input
	PORTD = 0xFF;		//pull-up resistrors
	
	int DC_VALUE=8;		//start for 50% DC_VALUE
	uint16_t volume;
	
	while(1)
	{
		while(mode==0){
			OCR1AL = DutyTable[DC_VALUE];	//output PWM to PB1 LED
			if (!(PIND & (1<<4))){			//check if PD4 is pressed
				_delay_ms(50);
				while (!(PIND & (1<<4)));	//debounce
				
				if (DC_VALUE<16){
					DC_VALUE += 1;			//+6%
				}
			}
			if (!(PIND & (1<<5))){			//check if PD5 is pressed
				_delay_ms(50);
				while (!(PIND & (1<<5)));	//debounce
				
				if (DC_VALUE>0){
					DC_VALUE -= 1;			//-6%
				}
			}
		check_mode();
		}
		
		while(mode==1){
			ADCSRA |= (1 << ADSC);			//start ADC
			while (ADCSRA & (1 << ADSC));
			volume = ADC >> 2;				//from 10 bit to 8 bit
			OCR1AL = volume;				//output PWM to PB1 LED
			check_mode();
		}
	}
}