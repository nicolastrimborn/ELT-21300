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
#include <avr/io.h>
#include <util/delay.h>#include <avr/interrupt.h>volatile uint8_t ADC_Value = 0;
#define TABLESIZE 10
#define TIMEINCR 4

// PWM output signal is a ramp 10 - 190
uint8_t table[TABLESIZE] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
uint8_t table_index = 0;

// for counting interrupts
static uint8_t int_count = 0;
void init_IO(void) {
	DIDR0 |= (1 <<ADC0D);  // Disable digital input buffer on utilized ADC Pins.  Reduce Power Consumption
	PORTC |= (1 <<PORTC0); //Write 1 to corresponding output pin
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

void init_ACD(void) {
	
	ADMUX |= (1 << REFS0); // AVcc with external capacitor on AREF pin. MUX 3...0 0x00 = ADC0
	ADMUX |= (1 << ADLAR); // Use only high byte ADCSRA[ADLAR] -> 1
	ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2); //ADC pre-scaler 128 (16M/128  = 125k) ADPS2:0 = 111
	ADCSRB = 0x00; //Set to Free running mode
	PRR &= ~(1<<PRADC); // enable AD-Converter in Power Reduction Register
	ADCSRA |= (1 << ADEN); // Enable the ADC
	ADCSRA |= (1 << ADIE); // Enable interrupts
	ADCSRA |= (1 << ADSC); // Start the ADC conversion
	sei();
	
}

void menuHander(unsigned char command) {
	switch (command)
	{
		case 1:
			USART_putstring("1 is pressed \r\n");
			break;
		case 2:
			USART_putstring("2 is pressed \r\n");
			break;
		case 3:
			USART_putstring("3 is pressed \r\n");
			break;
		case 4:
			USART_putstring("4 is pressed \r\n");
			break;
		case 5: 
			USART_putstring("5 is pressed \r\n");
			break;
	 	default:
			USART_putstring("command not understood \r\n");
	}
	
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
	ADC_Value = ( 255 * ADC ) / 1023; //scale to fit in 8bit value
	//ADC_Value = ADCH;
	//ADC_Value = ADCL | (ADCH << 8); //If using 10 bit presion, must read low first
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

int main(void)
{
	init_IO();
	init_Uart();
	init_ACD();
	init_PWM();
	sei();
    /* Replace with your application code */
    while (1) 
    {
		//USART_WriteNum(ADC_Value);
		USART_WriteNum(table_index);
		_delay_ms(100);	
	}
}



