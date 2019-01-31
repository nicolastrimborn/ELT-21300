/*
 * Ex03.c
 * Created: 24/01/2019 1:45:53 PM
 * Author : Nick
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL 
#include <util/delay.h>

#define ledPort			PORTB
#define ledDDR			DDRB
#define led1			PORTB0
#define led2            PORTB1

#define buttonPort		PORTD
#define buttonDDR		DDRD
#define button1			1
#define button2			2

void initIO()
{
	//LED Setup
	ledDDR = (1<<PORTB0)|(1<<PORTB1);	//Set ledPins DDR to Output
	PORTB |= (1<<led1);			//initialize Led1 to Off
	PORTB &= ~(1<<led2);		//initialize Led2 to Off
	
	//Button Setup	
	buttonDDR = 0x00;				//Set buttonPort DDR to input
	buttonPort |= (1<<button2);     //Set pullup for button 2 only
		
}

int main(void)
{
	void initIO();
    /* Replace with your application code */
    while (1) 
    {
		_delay_ms(1000);
		PORTB ^= (1<<led1);
		PORTB ^= (1<<led2);
		_delay_ms(1000);
		PORTB ^= (1<<led2);
		PORTB ^= (1<<led1);
    }
}

