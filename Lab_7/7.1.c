#define F_CPU 16000000UL	// Set CPU frequency

#include <avr/io.h>
#include <util/delay.h>

bool one_wire_reset(void) {
	DDRD |= (1 << PD4);      // sbi DDRD, PD4 (Set output)
	PORTD &= ~(1 << PD4);    // cbi PORTD, PD4 (Set low)
	
	_delay_us(480);          // 480us delay

	DDRD &= ~(1 << PD4);     // cbi DDRD, PD4 (Set input)
	PORTD &= ~(1 << PD4);    // cbi PORTD, PD4 (Disable pull-up)

	_delay_us(100);          // 100us delay

	uint8_t port_sample = PIND;	// Save PORTD state

	_delay_us(380);				// 380us delay

	if (!(port_sample & (1 << PD4))) return 1;	// If PD4 = 0  return 1
	else return 0;								// If PD4 = 1  return 0
}

uint8_t one_wire_receive_bit(void) {
	DDRD |= (1 << PD4);      // sbi DDRD, PD4 (Set output)
	PORTD &= ~(1 << PD4);    // cbi PORTD, PD4 (Set low)
	
	_delay_us(2);            // time slot 2 usec

	DDRD &= ~(1 << PD4);     // cbi DDRD, PD4 (Set input)
	PORTD &= ~(1 << PD4);    // cbi PORTD, PD4 (Disable pull-up)

	_delay_us(10);           // wait 10 usec

	uint8_t r24 = 0;         // clr r24
	if (PIND & (1 << PD4)) r24 = 1;	// r24 = PD4
	
	_delay_us(49);           // delay 49usec

	return r24;
}

void one_wire_transmit_bit(uint8_t r24) {
	
	DDRD |= (1 << PD4);      // sbi DDRD, PD4 (Set output)
	PORTD &= ~(1 << PD4);    // cbi PORTD, PD4 (Set low)
	
	_delay_us(2);            // time slot 2 usec
	
	if (r24 & 0x01) PORTD |= (1 << PD4); 
	else PORTD &= ~(1 << PD4);	// PD4 = r24[0]

	_delay_us(58);           // wait 58 usec

	DDRD &= ~(1 << PD4);     // cbi DDRD, PD4 (Set input)
	PORTD &= ~(1 << PD4);    // cbi PORTD, PD4 (Disable pull-up)

	_delay_us(1);            // recovery time 1 usec
}

uint8_t one_wire_receive_byte()
{
	uint8_t received_byte = 0x00;       // Store the byte (8-bit) we received
	for (uint8_t i = 0; i < 8; i++)
	{
		uint8_t received_bit = one_wire_receive_bit();
		received_byte |= (received_bit << i);	// Logical shift left, because DS18B20 send LSB first
		// Logical OR to insert new bit into byte sequence
	}
	return received_byte;
}

void one_wire_transmit_byte(uint8_t byte_to_transmit)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		uint8_t send_bit = (byte_to_transmit >> i) & 0x01;	// Bit to transmit now in position bit 0
		one_wire_transmit_bit(send_bit);
	}
}

int16_t get_temp()
{
	bool connected_device = one_wire_reset();   // Check for connected device
	if (!connected_device) return 0x8000;       // Error, return 0x8000
	
	one_wire_transmit_byte(0xCC);               // Only one device
	one_wire_transmit_byte(0x44);               // Read temperature
	
	while (!one_wire_receive_bit());			// Wait until the above counting terminates
	
	one_wire_reset();                           // Re-initialize
	
	one_wire_transmit_byte(0xCC);
	one_wire_transmit_byte(0xBE);               // Read 16-bit result of temperature value
	
	uint16_t temp = 0;
	temp |= one_wire_receive_byte();     // 8-bit LSB of the total 16-bit value
	temp |= ((uint16_t)one_wire_receive_byte() << 8);	// Get the other 8 bits shifted 8 times left	
	return temp;
}

int get_temp_decoded(uint16_t raw_temp)
{
	// The DS18B20 gives a 16-bit signed integer.
	// The last 4 bits are the decimal part (x 0.0625)
	// The first 5 bits (S) are the sign.
	
	// Simple way to get 1 decimal digit of precision:
	// Multiply by 10 and divide by 16 (>>4)
	// Use cast to (int16_t) so the compiler handles the sign automatically
	
	int16_t signed_raw = (int16_t)raw_temp;
	
	// Calculation: (Raw * 10) / 16
	// Use long to avoid overflow during multiplication before division
	return ((long)signed_raw * 10) / 16;
}

int main()
{
	
}