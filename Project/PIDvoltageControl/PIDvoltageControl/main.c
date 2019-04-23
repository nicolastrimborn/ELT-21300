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
#include <util/delay.h>#include <avr/interrupt.h>#include <stdio.h> #include <stdlib.h>#include <pid.h>volatile uint16_t ADC_Value = 0;
#define TABLESIZE 11
#define TIMEINCR 4
#define PWMTOP 255
#define FALSE 0
#define TRUE 1



volatile uint8_t dPortInput;		// read contents of the D port
volatile uint16_t edgeCounter = 0;	// count of the rising edges of the signal in D0
// variables used only in the interrupt service routine
uint8_t d0Input = 0;		// state of D2
uint8_t previousState = 0;	// previous state of D2
uint8_t pause=FALSE;

// PWM output signal is a ramp 10 - 190
uint8_t table[TABLESIZE] = {0,10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
uint8_t pwmTable[TABLESIZE] = {0,25, 51, 77, 102, 128, 153, 179, 204, 239, 255};
uint8_t table_index = 0;

// for counting interrupts
static uint8_t int_count = 0;uint16_t adc_value;/* ##############################################  PID Parameters #############################################*/  /*P, I and D parameter values
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand */
 //#define K_P 1.00
 //#define K_I 0.00
 //#define K_D 0.00
 
 uint8_t K_P = 1;
 uint8_t K_I = 0;
 uint8_t K_D = 0;
//flags for status information
struct GLOBAL_FLAGS {
	//! True when PID control loop should run one time
	uint8_t pidTimer : 1;
	uint8_t dummy : 7;
} gFlags = {0, 0};		uint16_t SetpointADCVal = 693;/*################################################  PID Parameters #############################################*///! Parameters for regulator
struct PID_DATA pidData;
/* Sampling Time Interval
 *
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255
 */
#define TIME_INTERVAL 157void init_PID(void){	//pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
	// Set up timer, enable timer/counter 0 overflow interrupt
	TCCR2B |= (1 << CS21); // clock source to be used by the Timer/Counter clkI/O (8 Prescaler)
	//TCCR2B |= (1 << CS20); // clock source to be used by the Timer/Counter clkI/O (No Prescaler)
	//TCCR2B |= (1 << CS22)|(1 << CS21); // clock source to be used by the Timer/Counter clkI/O (256 Prescaler)
	TIMSK2 = (1 << TOIE2); //TOIE2 – Timer/Counter2 Overflow Interrupt Enable
	TCNT2  = 0;}/* brief Read reference value.
 * This function must return the reference value. May be constant or varying*/
int16_t Get_Reference(void)
{
	return SetpointADCVal;
}

/* Read system process value
 * This function must return the measured data */
int16_t Get_Measurement(void)
{
	return ReadADC(0);
}/* Set control input to system
 * Set the output from the controller as input to system. */void Set_Input(int16_t inputValue)
{
	USART_putstring("Control Val: ");
	USART_WriteNum(inputValue);
	USART_putstring("\r\n");
	printADChannel(0);
}void init_IO(void) {
	DIDR0 |= (1 <<ADC0D);  // Disable digital input buffer on utilized ADC Pins.  Reduce Power Consumption
	PORTC |= (1 <<PORTC0); //Write 1 to corresponding output pin
	PRR &= ~(1 << PRTIM0);
	PRR &= ~(1 << PRTIM2);	
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
	TCCR0B |= (1 << CS01);	// pre-scaler 8. PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ]
	//TCCR0B |= (1 << CS01)|(1 << CS00); // pre-scaler 64. PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ] 
	//TCCR0B |= 0b10;			// pre-scaler 8
	TIMSK0 |= (1<<TOIE0);	// Timer overflow interrupt enable
	TCCR0A |= (1<<COM0A1);	// clear IO pin on match, set at BOTTOM. none-inverting mode
	TCCR0A |= (1<<WGM00) | (1<<WGM01);	// Fast PWM mode WGM[2:0] = 0b011, TOP=0xFF
	DDRD |= (1<<PD6);		// enable output buffer of PIN 6 on port D
	OCR0A = table[0];	// initial value for signal
	//OCR0A = table[TABLESIZE/2];	// initial value for signal
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
			USART_putstring("PWM DC%: ");
			USART_WriteNum(table[table_index]);
			USART_putstring("OCR0AVal: ");
			USART_WriteNum(pwmTable[table_index]);
			USART_putstring("\r\n");
			OCR0A = pwmTable[table_index++];
			if(table_index >=TABLESIZE) {
				table_index=0;
			}				// back to the start of the table			
			break;
		case 52:
			USART_putstring("4 is pressed \r\n");
			USART_putstring("Increasing P Term: ");
			USART_WriteNum(K_P++);
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 53: 
			USART_putstring("5 is pressed \r\n");
			USART_putstring("Increasing I Term: ");
			USART_WriteNum(K_I++);
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 54:
			USART_putstring("6 is pressed \r\n");
			USART_putstring("Increasing D Term: ");
			USART_WriteNum(K_D++);
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 55:
			USART_putstring("7 is pressed \r\n");
			USART_putstring("Reset PID Vals ");
			K_P = 0; K_I = 0; K_D = 0;
			USART_putstring("P: ");
			USART_WriteNum(K_P);
			USART_putstring(" I: ");
			USART_WriteNum(K_I);
			USART_putstring(" D: ");
			USART_WriteNum(K_D);
			USART_putstring("\r\n");
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
	 	default:
			USART_putstring("command not understood \r\n");
	}
	
}

void buttonHandler() {
	if(pause == 1) {
		USART_putstring("Un-Paused\r\n");
		pause = FALSE;
		OCR0A = pwmTable[table_index];
		togglePauseLED();
	} else {
		USART_putstring("Paused\r\n");
		OCR0A = pwmTable[0];
		pause = TRUE;
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
		////	USART_putstring("H is pressed \r\n");
		////} else if (recievedChar == 'L') {
		////	USART_putstring("L is pressed \r\n");
	//}
}

ISR( ADC_vect )
{
	//ADC_Value = ( 255 * ADC ) / 1023; //scale to fit in 8bit value
	//ADC_Value = ADCH;
	ADC_Value = ADCL | (ADCH << 8); //If using 10 bit presicion, must read low first
	ADCSRA |= (1 << ADSC); // Start conversion after interrupt complete;
}

ISR(TIMER0_OVF_vect)
{
	static uint16_t i = 0;
	//USART_putstring("Timer 1.. running");
//   	if(int_count++ >= TIMEINCR) {		// update signal every TIMEINCR cycle
//   		OCR0A = table[table_index++];
//   		if(table_index >=TABLESIZE)
//   		table_index=0;				// back to the start of the table
//   		int_count = 0;					// reset cycle counter
//   	}
//
	//if (i < TIME_INTERVAL) {
		//i++;
		//} else {
		////USART_putstring("Timer 1 running");
		////USART_putstring("\r\n");
		//gFlags.pidTimer = 1;
		//i               = 0;
	//}

}

ISR(TIMER2_OVF_vect)
{
	static uint16_t i = 0;
	//USART_putstring("Timer 2.. running");

	if (i < TIME_INTERVAL) {
		i++;
	} else {
		//USART_putstring("Timer 2 running");
		//USART_putstring("\r\n");
		gFlags.pidTimer = TRUE;
		i               = 0;
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
	int16_t referenceValue, measurementValue, inputValue;
	init_IO();
	init_Uart();
	init_ACD();
	init_PWM();
	init_PID();
	sei();
    while (1) {
		if (gFlags.pidTimer == TRUE) {
			USART_putstring("in pid handler\r\n");
			referenceValue   = Get_Reference();
			measurementValue = Get_Measurement();
			inputValue = pid_Controller(referenceValue, measurementValue, &pidData);
			Set_Input(inputValue);
			gFlags.pidTimer = FALSE;
		}
		//USART_putstring("not in pid handler\r\n");
		//potval=ReadADC(0);
		//USART_WriteNum(ADC_Value);
		//USART_WriteNum(table_index);
		//ReadADC(0);
		//printf("Vbg = %4.2fV\n", vbg);
		//_delay_ms(500);
		//printADChannel(0);              
		//printADCInterupt(ADC_Value);
		//_delay_ms(200);    			  
					  //This two lines are to tell to the terminal to change line             //You can tweak this value to have slower or faster readings or for max speed remove this line
		
	}
}



