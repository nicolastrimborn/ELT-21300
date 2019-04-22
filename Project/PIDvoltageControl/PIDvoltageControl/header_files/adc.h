/*
 * adc.h
 *
 * Created: 22/04/2019 10:45:22 PM
 *  Author: Nick
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#define VREF 5


void init_ACD(void);
uint16_t ReadADC(uint8_t ADCchannel);
void printADChannel(uint8_t channel);
char buffer[5];char buffer2[10];



#endif /* ADC_H_ */