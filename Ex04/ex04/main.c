/*
 * ex04.c
 *
 * Created: 31/01/2019 9:53:36 AM
 * Author : Nick
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

volatile int button_changed = 0;

void init_system(void)
{
	DDRB = 0x00; // ports B as input
	DDRD = 0x02; // port PD1 as output LED
	PORTD = 0x00; // Initialize outputs to 0
	
	//PORTB |= (1<<PB0); // set pin PB0 weqk pullup
	// enable pin change interrupts on port C in PCICR bit PCIE1
	PCICR |= (1 << PCIE0);
	// enable pin change interrupt on bit PC2 in PCMSK1 bit PCINT10
	PCMSK0 |= (1 << PCINT0);
	sei();	
}

ISR(PCINT0_vect)
{
	if (PINB & (1<<PB0)) {
		button_changed = 1;
	}
	
}

int main(void)
{
	init_system();
    /* Replace with your application code */
    while (1) 
    {
		if(button_changed) {
			PORTD ^= (1<<PD1); //toggle LED
			button_changed = 0;
		}
		
    }
}

