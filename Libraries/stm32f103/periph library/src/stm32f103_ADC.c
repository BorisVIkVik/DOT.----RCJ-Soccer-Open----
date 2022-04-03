#include "stm32f103_ADC.h"

bool calibrated[ADCQ];
int channelQ[ADCQ];
uint8_t sq[ADCQ][16];

void resetADC(void){
	ADC1->CR2 &= ~ADC_CR2_ADON;	
	ADC2->CR2 &= ~ADC_CR2_ADON;	
	ADC3->CR2 &= ~ADC_CR2_ADON;	
	
	for(int i = 0; i < ADCQ; i++){
		calibrated[i] = 0;
		channelQ[i] = 0;
		for(int a = 0; a < 16; a++) sq[i][a] = 0;
	}
}
	
void initADC(unsigned int adcNum, uint8_t channel, uint16_t prt, uint16_t pin, bool contMode){						//regular mode
	
	if(channelQ[adcNum] == 16) return;					//if all channels are in use
	
	ADC_TypeDef * adc;
	switch(adcNum){
		case 0: adc = ADC1; break;
		case 1: adc = ADC2; break;
		case 2: adc = ADC3; break;
	}
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;													//set clk
	
	if(adc == ADC1)	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;					//enable adc
	else if(adc == ADC2)	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;					//set adc clk under 14MHz
		
	initPin(prt, pin, INPUTAN);									//init pin
	
	adc->CR2 |= ADC_CR2_ADON;										//turn on adc
	
	if(!calibrated[adcNum]){										//calibration
		adc->CR2 |= ADC_CR2_CAL;
		while(!(adc->CR2 & ADC_CR2_CAL));
		calibrated[adcNum] = 1;
	}
	
	if(contMode)adc->CR2 |= ADC_CR2_CONT;				//continuous mode
	else adc->CR2 &= ~ADC_CR2_CONT;
	
	
	adc->CR2 |= ADC_CR2_EXTSEL;									//select SWSTART as external event
	adc->CR2 |= ADC_CR2_EXTTRIG;								//enable external trigger
	
	sq[adcNum][channelQ[adcNum]] = channel;
	adc->SQR1 |= ADC_SQR1_L & channelQ[adcNum];									//update total number channels in the sequence
	switch(channelQ[adcNum]){																		//place in the sequence
		case 0: adc->SQR3 |= ADC_SQR3_SQ1 & channel; break;
		case 1: adc->SQR3 |= ADC_SQR3_SQ2 & channel; break;
		case 2: adc->SQR3 |= ADC_SQR3_SQ3 & channel; break;
		case 3: adc->SQR3 |= ADC_SQR3_SQ4 & channel; break;
		case 4: adc->SQR3 |= ADC_SQR3_SQ5 & channel; break;
		case 5: adc->SQR3 |= ADC_SQR3_SQ6 & channel; break;
		case 6: adc->SQR2 |= ADC_SQR2_SQ7 & channel; break;
		case 7: adc->SQR2 |= ADC_SQR2_SQ8 & channel; break;
		case 8: adc->SQR2 |= ADC_SQR2_SQ9 & channel; break;
		case 9: adc->SQR2 |= ADC_SQR2_SQ10 & channel; break;
		case 10: adc->SQR2 |= ADC_SQR2_SQ11 & channel; break;
		case 11: adc->SQR2 |= ADC_SQR2_SQ12 & channel; break;
		case 12: adc->SQR1 |= ADC_SQR1_SQ13 & channel; break;
		case 13: adc->SQR1 |= ADC_SQR1_SQ14 & channel; break;
		case 14: adc->SQR1 |= ADC_SQR1_SQ15 & channel; break;
		case 15: adc->SQR1 |= ADC_SQR1_SQ16 & channel; break;
	} 
	
	switch(channel){																						//select minimum sample time
		case 0: adc->SMPR2 &= ~ADC_SMPR2_SMP0; break;
		case 1: adc->SMPR2 &= ~ADC_SMPR2_SMP1; break;
		case 2: adc->SMPR2 &= ~ADC_SMPR2_SMP2; break;
		case 3: adc->SMPR2 &= ~ADC_SMPR2_SMP3; break;
		case 4: adc->SMPR2 &= ~ADC_SMPR2_SMP4; break;
		case 5: adc->SMPR2 &= ~ADC_SMPR2_SMP5; break;
		case 6: adc->SMPR2 &= ~ADC_SMPR2_SMP6; break;
		case 7: adc->SMPR2 &= ~ADC_SMPR2_SMP7; break;
		case 8: adc->SMPR2 &= ~ADC_SMPR2_SMP8; break;
		case 9: adc->SMPR2 &= ~ADC_SMPR2_SMP9; break;
		case 10: adc->SMPR1 &= ~ADC_SMPR1_SMP10; break;
		case 11: adc->SMPR1 &= ~ADC_SMPR1_SMP11; break;
		case 12: adc->SMPR1 &= ~ADC_SMPR1_SMP12; break;
		case 13: adc->SMPR1 &= ~ADC_SMPR1_SMP13; break;
		case 14: adc->SMPR1 &= ~ADC_SMPR1_SMP14; break;
		case 15: adc->SMPR1 &= ~ADC_SMPR1_SMP15; break;
	}
	
	channelQ[adcNum]++;
	
}

uint16_t readADC(unsigned int adcNum, uint8_t channel){ ///////gdsdfsdfsd
	
	ADC_TypeDef * adc;
	switch(adcNum){
		case 0: adc = ADC1; break;
		case 1: adc = ADC2; break;
		case 2: adc = ADC3; break;
	}
	
	adc->CR2 |= ADC_CR2_SWSTART;
	
	int i = -1;
	do{
		i++;
		while(!(adc->SR & ADC_SR_EOC));
	}while(sq[adcNum][i] != channel && i < 17);
	
	return adc->DR;
	
}
