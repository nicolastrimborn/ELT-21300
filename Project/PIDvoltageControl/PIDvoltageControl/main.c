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
#include <util/delay.h>#include <avr/interrupt.h>
void initIO(void) {
	
	
}

void initACD(void) {
	
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


int main(void)
{
	init_Uart();
    /* Replace with your application code */
    while (1) 
    {
		
	}
}



