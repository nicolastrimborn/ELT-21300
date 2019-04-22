/*
 * PIDvoltageControl.c
 *
 * Created: 20/04/2019 10:12:24 PM
 * Author : Nick
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // set the CPU clock
#endif
#include <serial.h>
#include <adc.h>
#include <avr/io.h>
#include <util/delay.h>#include <avr/interrupt.h>#include <stdio.h> #include <stdlib.h>volatile uint16_t ADC_Value = 0;
#define TABLESIZE 10
#define TIMEINCR 4

volatile uint8_t dPortInput;		// read contents of the D port
volatile uint16_t edgeCounter = 0;	// count of the rising edges of the signal in D0
// variables used only in the interrupt service routine
uint8_t d0Input = 0;		// state of D2
uint8_t previousState = 0;	// previous state of D2
uint8_t pause=0;

// PWM output signal is a ramp 10 - 190
uint8_t table[TABLESIZE] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
uint8_t table_index = 0;

// for counting interrupts
static uint8_t int_count = 0;uint16_t adc_value;void init_IO(void) {
	DIDR0 |= (1 <<ADC0D);  // Disable digital input buffer on utilized ADC Pins.  Reduce Power Consumption
	PORTC |= (1 <<PORTC0); //Write 1 to corresponding output pin
	
	DDRC|= PORTC0;
	DDRB|= (1 << PB5); // Set LED as output
	
	// port D as input with pull-up
	//DDRD &= ~(1 << PD0); // data direction of PD0 as input (0)
	DDRD = 0x00;
	PORTD |= (1 << PD2); // enable pull-up on PD2 (PUD in MCUCR is enabled (0) by default)
	
	// enable pin change interrupts on port D in PCICR bit PCIE2 (address 0x68 )
	PCICR |= (1 << PCIE2);
	// enable pin change interrupt on bit PD2 in PCMSK2 bit PCINT18 (address 0x6d )
	PCMSK2 |= (1 << PCINT18);
	
}

void init_PWM()
{
	TCCR0B |= 0b10;			// pre-scaler 8
	TIMSK0 |= (1<<TOIE0);	// Timer overflow interrupt enable
	TCCR0A |= (1<<COM0A1);	// clear IO pin on match, set at BOTTOM
	TCCR0A |= (1<<WGM00) | (1<<WGM01);	// Fast PWM mode WGM[2:0] = 0b011
	DDRD |= (1<<PD6);		// enable output buffer of PIN 6 on port D
	OCR0A = table[TABLESIZE/2];	// initial value for signal
}



void menuHander(unsigned char command) {
	//Using decimal values for checking ASCII char received over UART
	switch (command)
	{
		case 49:
			USART_putstring("1 is pressed \r\n");
			printADChannel(0);
			break;
		case 50:
			USART_putstring("2 is pressed \r\n");
			USART_WriteNum(edgeCounter);
			break;
		case 51:
			USART_putstring("3 is pressed \r\n");
			break;
		case 52:
			USART_putstring("4 is pressed \r\n");
			break;
		case 53: 
			USART_putstring("5 is pressed \r\n");
			break;
	 	default:
			USART_putstring("command not understood \r\n");
	}
	
}

void buttonHandler() {
	if(pause == 1) {
		USART_putstring("Un-Paused\r\n");
		pause = 0;
		togglePauseLED();
	} else {
		USART_putstring("Paused\r\n");
		pause = 1;
		togglePauseLED();
	}
}

void togglePauseLED()
{
	PORTB ^= (1 << PB5); // Toggle the LED 
}

ISR(USART_RX_vect)
{
	unsigned char recievedChar = USART_recieve();
	menuHander(recievedChar);
	//if (recievedChar == 'H') {
	//	USART_putstring("H is pressed \r\n");
	//} else if (recievedChar == 'L') {
	//	USART_putstring("L is pressed \r\n");
	//}
}

ISR( ADC_vect )
{
	//ADC_Value = ( 255 * ADC ) / 1023; //scale to fit in 8bit value
	//ADC_Value = ADCH;
	ADC_Value = ADCL | (ADCH << 8); //If using 10 bit presion, must read low first
	ADCSRA |= (1 << ADSC); // Start conversion after interrupt complete;
}

ISR (TIMER0_OVF_vect)
{
	if(int_count++ >= TIMEINCR) {		// update signal every TIMEINCR cycle
		OCR0A = table[table_index++];
		if(table_index >=TABLESIZE)
		table_index=0;				// back to the start of the table
		int_count = 0;					// reset cycle counter
	}
	
}

ISR(PCINT2_vect)							// Pin change interrupt flag register PCIFR (0x3b)
{
	d0Input = PIND & (1<<PD2);				// read port D pin 0 (mask all other bits)
	if(previousState==0) {					// if previous state was 0 then the change must be a rising edge
		// action
		buttonHandler();
		dPortInput = PIND;					// read port D to the buffer variable
		edgeCounter++;
		
		//USART_WriteNum(edgeCounter);
								// count rising edges
	}
	previousState=d0Input;
}



int main(void)
{
	init_IO();
	init_Uart();
	init_ACD();
	init_PWM();
	sei();
    while (1) 
    {
		//potval=ReadADC(0);
		//USART_WriteNum(ADC_Value);
		//USART_WriteNum(table_index);
		//ReadADC(0);
		//printf("Vbg = %4.2fV\n", vbg);
		//_delay_ms(500);
		//printADChannel(0);              
		//printADCInterupt(ADC_Value);
		//_delay_ms(100);    			  
					  //This two lines are to tell to the terminal to change line             //You can tweak this value to have slower or faster readings or for max speed remove this line
			
	}
}



