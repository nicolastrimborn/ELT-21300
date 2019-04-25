/*
 * CFile1.c
 *
 * Created: 22/04/2019 10:44:45 PM
 *  Author: Nick
 */ 

#include "adc.h"

/*  Free running interrupt based ADC 
void init_ACD(void) {
	ADMUX |= (1 << REFS0); // AVCC with	external capacitor on Vref
	//ADMUX |= (1 << ADLAR); // Use only high byte ADCSRA[ADLAR] -> 1
	ADMUX |= (1 << ADC0D);
	ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2); //ADC pre-scaler 128 (16M/128  = 125k) ADPS2:0 = 111
	ADCSRB = 0x00; //Set to Free running mode
	PRR &= ~(1<<PRADC); // enable AD-Converter in Power Reduction Register
	ADCSRA |= (1 << ADEN); // Enable the ADC
	ADCSRA |= (1 << ADIE); // Enable interrupts
	ADCSRA |= (1 << ADSC); // Start the ADC conversion
}
*/


/* single conversion settings*/
void init_ACD(void) {
	  ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
	  ADMUX |= (1<<REFS0);                //Voltage 4 from Avcc (5v)
	  ADCSRA |= (1<<ADEN);                //Turn on ADC
	  ADCSRA |= (1<<ADSC);                //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}


// Function for single conversion
uint16_t ReadADC(uint8_t ADCchannel)
{
	ADMUX &= 0xF0;                    //Clear the older channel that was read
	ADMUX |= ADCchannel;                //Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                //Starts a new conversion
	while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
	return ADCW;                    //Returns the ADC value of the chosen channel
}

void printADChannel(uint8_t channel)
{
	volatile uint16_t adc_value = 0;
	float voltage;
	if (channel <=6 ) {
		USART_putstring("Reading channel ");
		USART_send('0' );            //This is a nifty trick when we only want to send a number between 0 and 9
		USART_putstring(" : ");            //Just to keep things pretty
		adc_value = ReadADC(channel);
		itoa(adc_value, buffer, 10);        //Convert the read value to an ascii string
		USART_putstring(buffer);        //Send the converted value to the terminal
		USART_putstring("  ");
		USART_putstring("Voltage : ");
		voltage = (float)VREF/1024*adc_value;
		dtostrf(voltage, 6, 3, buffer2);
		USART_putstring(buffer2);
		//Some more formatting
		USART_send('\r');
		USART_send('\n');
	}
}

void printADCInterupt(uint16_t adc_value)
{
	float voltage;
	USART_putstring("Reading channel ");
	USART_send('0' );            //This is a nifty trick when we only want to send a number between 0 and 9
	USART_putstring(" : ");            //Just to keep things pretty
	itoa(adc_value, buffer, 10);        //Convert the read value to an ascii string
	USART_putstring(buffer);        //Send the converted value to the terminal
	USART_putstring("  ");
	USART_putstring("Voltage : ");
	voltage = (float)VREF/1024*adc_value;
	dtostrf(voltage, 6, 3, buffer2);
	USART_putstring(buffer2);
	//Some more formatting
	USART_send('\r');
	USART_send('\n');
	
}