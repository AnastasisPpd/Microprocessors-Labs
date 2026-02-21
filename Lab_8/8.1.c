/*
 * main.c
 *
 * Created: 12/10/2025 5:42:47 PM
 *  Author: anast
 */ 

#define F_CPU 16000000UL // Set CPU frequency

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define PCA9555_0_ADDRESS 0x40 //A0=A1=A2=0 by hardware
#define TWI_READ 1 // reading from twi device
#define TWI_WRITE 0 // writing to twi device
#define SCL_CLOCK 100000L // twi clock in Hz

//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2
// PCA9555 REGISTERS

typedef enum {
	REG_INPUT_0 = 0,
	REG_INPUT_1 = 1,
	REG_OUTPUT_0 = 2,
	REG_OUTPUT_1 = 3,
	REG_POLARITY_INV_0 = 4,
	REG_POLARITY_INV_1 = 5,
	REG_CONFIGURATION_0 = 6,
	REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;
//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10
//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58
#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)
//initialize TWI clock
void twi_init(void)
{
	TWSR0 = 0; // PRESCALER_VALUE=1
	TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}
// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}
//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}
// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address)
{
	uint8_t twi_status;
	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
	// send device address
	TWDR0 = address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) )
	{
		return 1;
	}
	return 0;
}
// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address)
{
	uint8_t twi_status;
	while ( 1 )
	{
		// send START condition
		TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		// wait until transmission completed
		while(!(TWCR0 & (1<<TWINT)));
		// check value of TWI Status Register.
		twi_status = TW_STATUS & 0xF8;
		if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
		// send device address
		TWDR0 = address;
		TWCR0 = (1<<TWINT) | (1<<TWEN);
		// wail until transmission completed
		while(!(TWCR0 & (1<<TWINT)));
		// check value of TWI Status Register.
		twi_status = TW_STATUS & 0xF8;
		if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) )
		{
			/* device busy, send stop condition to terminate write operation */
			TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
			// wait until stop condition is executed and bus released
			while(TWCR0 & (1<<TWSTO));
			continue;
		}
		break;
	}
}
// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data )
{
	// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
	return 0;
}
// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address)
{
	return twi_start( address );
}
// Terminates the data transfer and releases the twi bus
void twi_stop(void)
{
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value)
{
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}
uint8_t PCA9555_0_read(PCA9555_REGISTERS reg)
{
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	return ret_val;
}
void write_2_nibbles(uint8_t lcd_data) {
	uint8_t temp;
	// Send the high nibble, Keep lower 4 bits of PIND and set high nibble of lcd_data
	temp = (PCA9555_0_read(REG_OUTPUT_0) & 0x0F) | (lcd_data & 0xF0);
	PCA9555_0_write(REG_OUTPUT_0 , temp);
	// Output the high nibble to PORTD, Enable pulse high
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) | (1 << PD3));
	_delay_us(1); // Small delay to let the signal settle
	// Enable pulse low
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) & ~(1 << PD3));
	// Send the low nibble
	lcd_data <<= 4; // Move low nibble to high nibble position
	// Keep lower 4 bits of PIND and set high nibble of new lcd_data
	temp = (PCA9555_0_read(REG_OUTPUT_0) & 0x0F) | (lcd_data & 0xF0);
	PCA9555_0_write(REG_OUTPUT_0 , temp);
	// Output the low nibble to PORTD, Enable pulse high
	PCA9555_0_write(REG_OUTPUT_0 , PCA9555_0_read(REG_OUTPUT_0) | (1 << PD3));
	_delay_us(1); // Small delay to let the signal settle
	// Enable pulse low
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) & ~(1 << PD3));
}
void lcd_data(uint8_t data)
{
	uint8_t current_state = PCA9555_0_read(REG_OUTPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, current_state | 0x04);
	write_2_nibbles(data);
	_delay_us(100);
}
void lcd_command(uint8_t data)
{
	uint8_t current_state = PCA9555_0_read(REG_OUTPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, current_state & 0xFB);
	write_2_nibbles(data);
	_delay_ms(2);
}
void lcd_clear_display()
{
	uint8_t clear_disp = 0x01; // Clear display command
	lcd_command(clear_disp);
	_delay_ms(5); // Wait 5 msec
	return;
}
void lcd_init() {
	_delay_ms(200);
	// Send 0x30 command to set 8-bit mode (three times)
	PCA9555_0_write(REG_OUTPUT_0,0x30); // Set command to switch to 8-bit mode
	// Enable pulse
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) | (1 << PD3));
	_delay_us(1);
	// Clear enable
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) & ~(1 << PD3));
	PCA9555_0_write(REG_OUTPUT_0,0x30); // Repeat command to ensure mode set
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) | (1 << PD3));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) & ~(1 << PD3));
	_delay_us(30);
	PCA9555_0_write(REG_OUTPUT_0,0x30); // Repeat once more
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) | (1 << PD3));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) & ~(1 << PD3));
	_delay_us(30);
	// Send 0x20 command to switch to 4-bit mode
	PCA9555_0_write(REG_OUTPUT_0,0x20);
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) | (1 << PD3));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0,PCA9555_0_read(REG_OUTPUT_0) & ~(1 << PD3));
	_delay_us(30);
	// Set 4-bit mode, 2 lines, 5x8 dots
	lcd_command(0x28);
	// Display ON, Cursor OFF
	lcd_command(0x0C);
	// Clear display
	lcd_clear_display();
	// Entry mode: Increment cursor, no display shift
	lcd_command(0x06);
}
// ????????? ??? ???????? String ???? LCD
void lcd_print(const char *str) {
	while (*str) {
		lcd_data(*str++); // ??????? ??? ????????? ??? ????????? ???? ???????
	}
}
/* Routine: usart_init
Description:
This routine initializes the
usart as shown below.
------- INITIALIZATIONS -------
Baud rate: 9600 (Fck= 8MH)
Asynchronous mode
Transmitter on
Reciever on
Communication parameters: 8 Data ,1 Stop, no Parity
--------------------------------
parameters: ubrr to control the BAUD.
return value: None.*/
void usart_init(unsigned int ubrr){
	UCSR0A=0;
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);
	UBRR0H=(unsigned char)(ubrr>>8);
	UBRR0L=(unsigned char)ubrr;
	UCSR0C=(3 << UCSZ00);
	return;
}

/* Routine: usart_transmit
Description:
This routine sends a byte of data
using usart.
parameters:
data: the byte to be transmitted
return value: None. */
void usart_transmit(uint8_t data){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0=data;
}

// 1. ????????? ??? ???????? String
void usart_print(const char *str) {
	while (*str) {
		usart_transmit(*str++);
	}
}

/* Routine: usart_receive
Description:
This routine receives a byte of data
from usart.
parameters: None.
return value: the received byte */
uint8_t usart_receive(){
	while(!(UCSR0A&(1<<RXC0)));
	return UDR0;
}

void usart_receive_string(char *buf, uint8_t len) {
	char c;
	// ????? ??? ???????? ????????? ??? ??????? ?????????? ?? ????? '\n'
	while ((c = usart_receive()) != '\n') {
		// ?? ??? ????? '\r' ??? ??????? ?????????? ????? (???????? 1 ??? ?? \0)
		if (c != '\r' && --len > 0) {
			*buf++ = c; // ?????????? ????????? ??? ?????????? ??? ?????? ??????? ????
		}
	}
	*buf = '\0'; // ??????????? string (Null terminator)
}

int main(void)
{ 
	 twi_init();
	 PCA9555_0_write(REG_CONFIGURATION_0, 0x00); 
	 lcd_init();
	 lcd_clear_display();

	 
	usart_init(103);
	
	char buf[20];
	char buffer[20];
	
    while(1)
    {
		//char buf[20];
		//char buffer[20];
		 // 1) Send connect command
         usart_print("ESP:connect\r\n");
         usart_receive_string(buf,20);
		 
		 if (strcmp(buf, "Success") == 0) {

			 lcd_print("1.Success");
			 _delay_ms(2000);
			 
			 lcd_clear_display();
		 }
		 else {
		lcd_print("1.Fail");
		 _delay_ms(2000);
		}
		 // 2) Send URL command
		 lcd_clear_display();
	 	 
		 usart_print("ESP:url:\"http://192.168.1.250:5000/data\"\r\n");
		 usart_receive_string(buffer,20);
		 
		 if (strcmp(buffer, "Success") == 0) {
			 lcd_print("2.Success");
			 _delay_ms(2000);
			 
			 lcd_clear_display();
			 break;
		 }
		 else {
			 lcd_print("2.Fail");
		 _delay_ms(2000);
		 
		 lcd_clear_display();
		 }
	}
}