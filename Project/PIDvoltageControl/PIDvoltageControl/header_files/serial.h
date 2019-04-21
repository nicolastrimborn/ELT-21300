#ifndef SERIAL_H
#define SERIAL_H
#endif

#ifndef F_CPU
#define F_CPU 16000000UL // set the CPU clock
#endif

#define BAUD 9600 // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1) // set baudrate value for UBRR

#include <avr/io.h>
#include <util/delay.h>#include <avr/interrupt.h>


void init_Uart(void);
void USART_putstring (char *str);
void USART_send (unsigned char data);
unsigned char USART_recieve (void);
void USART_Flush( void );