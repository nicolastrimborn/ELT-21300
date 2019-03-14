/*
 * ADwithInterrupts.c
 *   Interrupt routine for parallel output AD-converter
 *   Data on ports B and D
 *   Control on port C
 *
 * Created: 28.1.2019 12:36:51
 * Author : jv
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint16_t conversion_result;
volatile uint8_t conversion_done = 0;

ISR(PCINT1_vect)	// Pin change interrupt on port C -> data available
			// NOTE: this code contains an error. This interrupt
			// routine should check that the interrupt was caused by
			// a falling edge. Rising edge should be ignored.
{
	// read data in
	PORTC = PORTC | (1<<PC4);
	conversion_result = (PINB << 8) | PIND;	// high order bits on port B
	// reset read signal (removes also AD-converter conversion ready signal)
	PORTC = PORTC & ~(1<<PC4);
	// reset start conversion pin
	PORTC = PORTC | (1<<PC0);
	// reset chip select
	PORTC = PORTC | (1<<PC1);
}

void start_conversion(void)
{
	// set chip select to low
	PORTC = PORTC & ~(1<<PC1);
	// set start conversion pin to low
	PORTC = PORTC & ~(1<<PC0);
}

void polled_conversion(void)
{
	// set chip select to low
	PORTC = PORTC & ~(1<<PC1);
	// set start conversion pin to low
	PORTC = PORTC & ~(1<<PC0);

	// wait for the conversion to finish
	while(!(PORTC & ~(1<<PC2))) ;

	// read data
	PORTC = PORTC | (1<<PC4);
	conversion_result = (PINB << 8) | PIND;	// high order bits on port B

	// reset read signal
	PORTC = PORTC & ~(1<<PC4);
	// reset start conversion pin
	PORTC = PORTC | (1<<PC0);
	// reset chip select
	PORTC = PORTC | (1<<PC1);
}

void init_system(void)
{
	// ports B and D as input
	DDRB = 0x00;
	DDRD = 0x00;

	// port C as control
	// 	PC0 is output, starts conversion, active low
	//	PC1 is AD-converter chip select, active low
	//	PC2 is input, end of conversion, causes an interrupt, active low
	//	PC3 is output, reset, active low
	//	PC4 is output, read signal, active low
	DDRC = (1<<PC0) | (1<<PC1) | (1<<PC3) | (1<<PC4);

	// start conversion pin to high (inactive)
	PORTC = PORTC | (1<<PC0);
	// AD chip select high (inactive)
	PORTC = PORTC | (1<<PC1);
	// AD read high (inactive)
	PORTC = PORTC | (1<<PC4);

	// reset AD_converter, pulse reser to low
	PORTC = PORTC | (1<<PC3);	// initially low must be set to high
	PORTC = PORTC & ~(1<<PC3);	// low
	PORTC = PORTC | (1<<PC3);	// high

	// comment out if you use only polled data transfer
	// enable pin change interrupts on port C in PCICR bit PCIE1
	PCICR |= (1 << PCIE1);
	// enable pin change interrupt on bit PC2 in PCMSK1 bit PCINT10
	PCMSK1 |= (1 << PCINT10);
	sei();
}

int main(void)
{
	uint32_t cumulative_result = 0;
	init_system();
	while(1) {
		start_conversion();
		// wait for the result

		while(!conversion_done) ; // not a good example as here we poll again
		// use the result
		cumulative_result += conversion_result;
	}
}
