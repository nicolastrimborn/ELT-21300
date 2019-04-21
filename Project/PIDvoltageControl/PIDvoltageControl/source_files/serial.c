/*
 * CFile1.c
 *
 * Created: 20/04/2019 11:46:25 PM
 *  Author: Nick
 */ 
#include <serial.h>

void init_Uart(void) {
	/* UCSR0B */
	// (TXEN0 = 1) (RXEN0 = 1) enable receiver and transmitter
	/* UCSR0C */
	// Asynchronous UART (UMSEL01=0) (UMSEL00=0)
	// 8bit data format 1 Stop BIT (USBS0 = 0), No Parity (UPM01 = 0) (UPM00 = 0)
	
	UBRR0H=(BAUDRATE>>8);
	UBRR0L=BAUDRATE; //set baud rate
	UCSR0C|= (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes// 8bit data format 1 StopBIT (USBS0 = 0), No Parity (UPM01 = 0) (UPM00 = 0)
	UCSR0B|=(1<<TXEN0)|(1<<RXEN0); //enable receiver and transmitter
	UCSR0B|= (1<<RXCIE0); // enable receiver interrupt
	USART_Flush();
	sei();
}
void USART_putstring (char *str)
{
	while (*str != 0x00) { // wait while register is free
		USART_send(*str);
		str++;
	}
}

void USART_send (unsigned char data)
{
	while (!((UCSR0A & (1<<UDRE0)))); // do nothing while buffer is busy
	UDR0 = data;
	
}
// function to receive data
unsigned char USART_recieve (void)
{
	while(!((UCSR0A & (1<<UDRE0)))); // do nothing while buffer is busy
	return UDR0; // return 8-bit data
}


void USART_Flush( void )
{
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0))
	dummy = UDR0;
}