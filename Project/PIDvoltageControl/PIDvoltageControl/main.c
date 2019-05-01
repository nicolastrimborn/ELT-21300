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
#include <util/delay.h>#include <avr/interrupt.h>#include <stdio.h> #include <stdlib.h>#include <pid.h>
#define TABLESIZE 11
#define TIMEINCR 4
#define PWMTOP 255
#define FALSE 0
#define TRUE 1

// variables used only in the interrupt service routine
volatile uint16_t ADC_Value;
volatile uint8_t dPortInput;		// read contents of the D port

uint8_t d0Input = 0;		// state of D2
uint8_t previousState = 0;	// previous state of D2
uint8_t pause=FALSE;

// PWM output signal is a ramp 0 - 100 DutyC
uint8_t table[TABLESIZE] = {0,10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
uint8_t pwmTable[TABLESIZE] = {0,25, 51, 77, 102, 128, 153, 179, 204, 239, 255};
	
uint8_t SP_index = 0;	
//uint8_t SP[6] = {0,1,2,3,4,5};
const uint16_t SP_ADC_VAL[6] = {13,210,410,607,807,1001};

		
uint8_t table_index = 0;

// for counting interrupts
static uint8_t int_count = 0;uint16_t adc_value;/* ##############################################  PID Parameters #############################################*/  /*P, I and D parameter values */
//* Good values Kp = 0.3 Ki=0.1 Kd = 0
 volatile float K_P = 0.3;
 volatile float K_I = 0.1;
 volatile float K_D = 0.0;
 
//flags for status information
typedef struct GLOBAL_FLAGS {
	//! True when PID control loop should run one time
	volatile uint8_t  pidTimer : 1;
	volatile uint8_t dummy : 7;
} GLOBAL_FLAGS; GLOBAL_FLAGS gFlags = {.pidTimer = 0, .dummy = 0};
//! Parameters for regulator
struct PID_DATA pidData;

/* Sampling Time Interval
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255 */
 #define TIME_INTERVAL 127
 
 volatile uint16_t SetpointADCVal;
 volatile uint16_t paused_SetpointADCVal;
 
/*################################################  PID Parameters #############################################*/void init_PID(void){	SetpointADCVal = SP_ADC_VAL[3];	SP_index = 3;	//pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
	// Set up timer, enable timer/counter 0 overflow interrupt
	//TCCR2B |= (1 << CS21); // clock source to be used by the Timer/Counter clkI/O (8 Prescaler)
	TCCR2B |= (1 << CS20); // clock source to be used by the Timer/Counter clkI/O (No Prescaler)
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
	
	if((int32_t)OCR0A + inputValue > 255) {
		OCR0A = 255;
	} else if ((int32_t)OCR0A + inputValue < 0) {
		OCR0A = 0;
	} else {
		OCR0A = OCR0A + inputValue;
	}
	char buf[80];
	sprintf(buf,"SP: %01dV ctrlVal: %04d PwmVal: %04d SPADCVal: %04d  Curr ADC: %04d Kp= %2.1f Ki= %1.1f Kd= %2.1f \r\n",
			SP_index,inputValue, OCR0A, SetpointADCVal, ReadADC(0), K_P, K_I, K_D);
	//sprintf(buf,"SP: %02d ctrlVal: %04d PwmVal: %04d SPADCVal: %04d  Curr ADC: %04d Kp= %2.1f Ki= %1.1f Kd= %2.1f \r\n",
	//SP[SP_index], inputValue, OCR0A, SetpointADCVal, ReadADC(0), K_P, K_I, K_D);
	USART_putstring(buf);
	
}void printValues() {	//char buf[80];	//sprintf(buf,"PwmVal: %04d SP_ADCVal: %04d Curr ADC: %04d Kp= %1.1f Ki= %1.1f Kd= %2.1f \r\n",
			//OCR0A, SetpointADCVal, ReadADC(0), K_P, K_I, K_D);	
	//USART_putstring(buf);	//sprintf(buf,"ctrlVal: %04d PwmVal: %04d SPADCVal: %04d  Curr ADC: %04d Kp= %2.1f Ki= %1.1f Kd= %2.1f \r\n",
	//inputValue, OCR0A, SetpointADCVal, ReadADC(0), K_P, K_I, K_D);}void init_IO(void) {
	DIDR0 |= (1 <<ADC0D);  // Disable digital input buffer on utilized ADC Pins.  Reduce Power Consumption
	PORTC |= (1 <<PORTC0); //Write 1 to corresponding output pin
	PRR &= ~(1 << PRTIM0); // Disable Timer0 Power Reduction
	PRR &= ~(1 << PRTIM2);	// Disable Timer0 Power Reduction
	
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
	TCCR0B |= (1 << CS01);	// pre-scaler 8. PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ] approx 7.8kHz
	//TCCR0B |= (1 << CS01)|(1 << CS00); // pre-scaler 64. PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ] 
	TIMSK0 |= (1<<TOIE0);	// Timer overflow interrupt enable
	TCCR0A |= (1<<COM0A1);	// clear IO pin on match, set at BOTTOM. none-inverting mode
	TCCR0A |= (1<<WGM00) | (1<<WGM01);	// Fast PWM mode WGM[2:0] = 0b011, TOP=0xFF
	DDRD |= (1<<PD6);		// enable output buffer of PIN 6 on port D
	//OCR0A = table[6];	// initial value for signal
}

void buttonHandler(void) {
	if(pause == 1) {
		USART_putstring("Un-Paused\r\n");
		pause = FALSE;
		//OCR0A = pwmTable[table_index];
		SetpointADCVal = paused_SetpointADCVal;
		togglePauseLED();
	} else {
		USART_putstring("Paused\r\n");
		//OCR0A = pwmTable[0];
		paused_SetpointADCVal = SetpointADCVal;
		SetpointADCVal = SP_ADC_VAL[0];
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
}

ISR( ADC_vect )
{
	ADC_Value = ADCL | (ADCH << 8); //using 10 bit precision, must read low first
	ADCSRA |= (1 << ADSC); // Start conversion after interrupt complete;
}

ISR(TIMER0_OVF_vect)
{

}

ISR(TIMER2_OVF_vect)
{
	static uint16_t i = 0;
	//USART_putstring("Timer 2.. running");
	if (i < TIME_INTERVAL) {
		i++;
	} else {
		gFlags.pidTimer = 1;
		i               = 0;
		//printValues();
	}
}

ISR(PCINT2_vect)							// Pin change interrupt flag register PCIFR (0x3b)
{
	
	d0Input = PIND & (1<<PD2);				// read port D pin 0 (mask all other bits)
	if(previousState==0) {					// if previous state was 0 then the change must be a rising edge
		// action
		buttonHandler();
		dPortInput = PIND;					// read port D to the buffer variable
	}
	previousState=d0Input;
}

void menuHander(unsigned char command) {
	//Using decimal values for checking ASCII char received over UART
	switch (command)
	{
		case 48:
			USART_putstring("0 is pressed \r\n");
			USART_putstring("SP: 0V");
			SetpointADCVal = SP_ADC_VAL[0];
			SP_index = 0;
			break;
		case 49:
			USART_putstring("1 is pressed \r\n");
			USART_putstring("SP: 1V");
			SetpointADCVal = SP_ADC_VAL[1];
			SP_index = 1;
			break;
		case 50:
			USART_putstring("2 is pressed \r\n");
			USART_putstring("SP: 2V");
			SetpointADCVal = SP_ADC_VAL[2];
			SP_index = 2;
			break;
		case 51:
			USART_putstring("3 is pressed \r\n");
			USART_putstring("SP: 3V");
			SetpointADCVal = SP_ADC_VAL[3];
			SP_index = 3;
			break;
		case 52:
			USART_putstring("4 is pressed \r\n");
			USART_putstring("SP: 4V");
			SetpointADCVal = SP_ADC_VAL[4];
			SP_index = 4;
			break;
		case 53:
			USART_putstring("5 is pressed \r\n");
			USART_putstring("SP: 5V");
			SetpointADCVal = SP_ADC_VAL[5];
			SP_index = 5;
			break;
		case 54:
			USART_putstring("6 is pressed \r\n");
			USART_putstring("Increasing P Term: ");
			K_P = K_P+0.1;
			//pidData.P_Factor = K_P; pidData.sumError = 0;
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 55:
			USART_putstring("7 is pressed \r\n");
			USART_putstring("Increasing I Term: ");
			K_I = K_I+0.1;
			//pidData.I_Factor = K_I; //pidData.sumError = 0;
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 56:
			USART_putstring("8 is pressed \r\n");
			USART_putstring("Increasing D Term: ");
			K_D = K_D+0.1;
			//pidData.D_Factor = K_D;  //pidData.sumError = 0;
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 57:
			USART_putstring("9 is pressed \r\n");
			USART_putstring("Reset PID Vals ");
			K_P = 0; K_I = 0; K_D = 0;
			//pidData.I_Factor = K_I; pidData.D_Factor = K_D; pidData.P_Factor = K_P;
			pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			
			//USART_putstring("P: ");
			//USART_WriteNum(K_P);
			//USART_putstring(" I: ");
			//USART_WriteNum(K_I);
			//USART_putstring(" D: ");
			//USART_WriteNum(K_D);
			//USART_putstring("\r\n");
			//pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
			break;
		case 61:
			USART_putstring("+ is pressed \r\n");
			USART_putstring("PWM DC%: ");
			USART_WriteNum(table[table_index]);
			USART_putstring("OCR0AVal: ");
			USART_WriteNum(pwmTable[table_index]);
			USART_putstring("\r\n");
			OCR0A = pwmTable[table_index++];
			if(table_index >=TABLESIZE) {
				table_index=0; // back to the start of the table
			}				
			break;
		case 65:
			USART_putstring("A is pressed \r\n");
			USART_putstring("PWM DC%: ");
			break;
		default:
			USART_putstring(command);
			USART_putstring(" command not understood \r\n");
	}
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
		if (gFlags.pidTimer == 1) {
			//USART_putstring("in pid handler\r\n");
			referenceValue   = Get_Reference();
			measurementValue = Get_Measurement();
			inputValue = pid_Controller(referenceValue, measurementValue, &pidData);
			Set_Input(inputValue);
			gFlags.pidTimer = 0;
		}
	}
}