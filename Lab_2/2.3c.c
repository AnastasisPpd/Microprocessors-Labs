/*
 * main.c
 *
 * Created: 10/23/2025 1:13:16 PM
 *  Author: anast
 */ 

#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

volatile int counter = 0;

ISR (INT1_vect)		//external INT1 ISR
{
	if(counter == 0) counter = 4000;
	else{
	PORTB = 0b00111111;
	_delay_ms(1000);
	PORTB = 0b00001000;
	counter = 3000;
	}
}

int main(void)
{
	//interrupt on rising edge of INT1 pin
	EICRA = (1<<ISC11) | (1<<ISC10);
	//enable the INT1 interrupt (PD3)
	EIMSK = (1<<INT1);
	//enable global interrupts
	sei();
	
	DDRB = 0xFF;		//set PORTB as output
	
    while(1)
    {
		PORTB = 0x00;
		
		while(counter>0)
		{
			PORTB = 0b00001000;
			counter -= 1;
			_delay_ms(1);
		}
    }
}