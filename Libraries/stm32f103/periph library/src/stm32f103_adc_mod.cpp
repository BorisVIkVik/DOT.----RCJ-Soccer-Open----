#include "stm32f103_adc_mod.h"

const int8_t PL_ADC::channels[16] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5};

PL_ADC::PL_ADC(ADC_TypeDef* adc) {
	pinMode(PA0, INPUTAN);
	
	this->adc = adc;
	
	if (adc == ADC1) index = 1;
	if (adc == ADC2) index = 2;
	
	usedCount = 0;
	for (int i = 0; i < 4; i++) jdr[i] = -1;
}

bool PL_ADC::add(int8_t pin) {
	if (usedCount >= 3) {
		ERROR("All injected channels in use");
		return false;
	}
	
	int8_t chn = -1;
	for (int i = 0; i < 16; i++) {
		if (channels[i] == pin) {
			chn = i;
			break;
		}
	}
	
	if (chn == -1) {
		ERROR("You cannot use ADC on that pin");
		return false;
	}
	/*
	printUART(_UART1, chn);
	writeStrUART(_UART1, "\r\n");
	*/
	pinMode(pin, INPUTAN);
	adc->JSQR |= (uint32_t)(3) << 20; // Set maximum count of injected channels by default
	adc->JSQR |= (uint32_t)(chn) << (5 * usedCount); // Set current channel
	
	jdr[usedCount] = pin;
	usedCount++;
	
	return true;
}

void PL_ADC::waitUpdate() {
	for(int k = 0; k < 256; k++);
}

unsigned PL_ADC::read(int8_t pin, bool withDelay) {
	if (withDelay) waitUpdate();
	
	if (jdr[0] == pin) return adc->JDR1;
	if (jdr[1] == pin) return adc->JDR2;
	if (jdr[2] == pin) return adc->JDR3;
	if (jdr[3] == pin) return adc->JDR4;
	
	ERROR("Pin are not initialized as ADC pin");
	return 42;
}

void PL_ADC::init() {
	
	//RCC config
	if (index == 1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	if (index == 2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	
	//Start ADC for calibration
	adc->CR2 |= ADC_CR2_ADON;
	adc->CR2 |= ADC_CR2_SWSTART;
	
	for(unsigned long i = 0; i < 100000; i++) asm("nop");
	
	//Calibration
	adc->CR2 |= ADC_CR2_CAL;
	while (!(adc->CR2 & ADC_CR2_CAL)){};
	
	//Stop ADC
	adc->CR2 &= ~ADC_CR2_SWSTART;
	adc->CR2 &= ~ADC_CR2_ADON;
	
	//Set 239 cycles reading delay
	adc->SMPR2 |= (ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1
        | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4
        | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7); // 14 cycles wait
	
	//Enable automatic start of injected channels
	adc->CR1 |= ADC_CR1_JAUTO;
	adc->CR1 |= ADC_CR1_SCAN;

	
	//Enable only one regular group channel
	adc->SQR1 &= ~(ADC_SQR1_L_2 | ADC_SQR1_L_1 | ADC_SQR1_L_0);
	
	
}

void PL_ADC::start() {
	adc->CR2 |= ADC_CR2_ADON;
	adc->CR2 |= ADC_CR2_SWSTART;
}

void PL_ADC::stop() {
	adc->CR2 &= ~ADC_CR2_SWSTART;
	adc->CR2 &= ~ADC_CR2_ADON;
}
