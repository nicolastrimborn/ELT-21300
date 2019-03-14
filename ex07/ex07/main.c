/*
 * ex07.c
 *
 * Created: 21/02/2019 4:51:56 PM
 * Author : Nick
 */ 

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL;

#define EN1 PD5
#define EN2 PD7
#define forward 1
#define backward 0
#define run 1
#define brake 0

volatile uint8_t DC_motor_speed = 0;
bool runMotor = false;

void init_pwm_timer0(void);
void init_io(void);
void init_adc(void);
void setDirection(int direction);
void motorState(int state);


void setDirection(int direction)
{
	if(runMotor) {
		if(direction == forward) {
			PORTD &= ~(1 << EN1); // forward
			PORTD |= (1 << EN2); 
		
		} else {
			PORTD &= ~(1 << EN2); // forward
			PORTD |= (1 << EN1);
		} 
	}
	
}

void motorState(int state) 
{
	if(!runMotor) {
		runMotor = false;
		PORTD |= (1 << EN1)|(1 << EN2);		
	} else {
		runMotor = true;
	}
}

int main(void)
{
	init_io();
	init_pwm_timer0();
    /* Replace with your application code */
    while (1) 
    {
		OCR0A = DC_motor_speed;
		setDirection(forward);
		motorState(run);
    }
}

void init_pwm_timer0(void) 
{
	
	OCR0A = 128;  //50% duty cycle
	TCCR0A |= (1<<COM0A1);	// clear IO pin on match, set at BOTTOM
	TCCR0A |= (1<<WGM00) | (1<<WGM01);	// Fast PWM mode WGM[2:0] = 0b011
	//PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ]
	TCCR0B |= 0b11;		// 16000000/ 64 * (1+255) (16Mhz/1024*256) = 1khz)
}

void init_io(void)
{
	DDRD |= (1 << PD5)|(1 << PD6)|(1 << PD7);
}

void init_adc(void)
{
	ADMUX |= (1 << REFS0); // AVcc with external capacitor on AREF pin. MUX 3...0 0x00 = ADC0
	ADCSRA |= (1 << ADEN); // Enable the ADC
	ADCSRA |= (1 << ADIE); // Enable interrupts
	ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2); //ADC prescaler 128 (16M/128  = 125k)  
	sei();
	ADCSRA |= (1 << ADSC); // Start the ADC conversion
}


ISR( ADC_vect )
{
	DC_motor_speed = ( 255 * ADC ) / 1023; //scale to fit in 8bit value
	ADCSRA |= 1 << ADSC;
}

