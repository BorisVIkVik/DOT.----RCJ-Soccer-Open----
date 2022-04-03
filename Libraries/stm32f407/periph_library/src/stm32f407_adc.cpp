#include "stm32f407_adc.h"

PL_ADC::PL_ADC(ADC_TypeDef* adc) {
	this->adc = adc;
	
	if (adc == ADC1) index = 1;
	if (adc == ADC2) index = 2;
	if (adc == ADC3) index = 3;
	
	usedCount = 0;
	for (int i = 0; i < 4; i++) jdr[i] = -1;
}

bool PL_ADC::add(int8_t pin) {
	if (usedCount > 3) {
		//ERROR("All injected channels in use");
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
		//ERROR("You cannot use ADC on that pin");
		return false;
	}
	//printUART(_UART6, chn);
	//writeStrUART(_UART6, "\r\n");
	
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
	
	//ERROR("Pin are not initialized as ADC pin");
	return 42;
}

void PL_ADC::init() {
	if (index == 1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	if (index == 2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	if (index == 3) RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
	
	/* Set ADC prescaler to 8 */
	ADC->CCR |= ADC_CCR_ADCPRE_0;
	ADC->CCR |= ADC_CCR_ADCPRE_1;
	
	/* Set ADC trigger source */
	adc->CR2 |= ADC_CR2_JEXTSEL;
	
	/* Enable continious mode */
	adc->CR2 |= ADC_CR2_CONT;
	
	/* Set 12 bit resolution */
	adc->CR1 &= ((~ADC_CR1_RES_0) & (~ADC_CR1_RES_1));
	
	/* Maybe multiply channel support TODO: Check it */
	adc->CR1 |= ADC_CR1_SCAN;
	
	/* Enable automatic start of injected channels */
	adc->CR1 |= ADC_CR1_JAUTO;
}

void PL_ADC::start() {
	adc->CR2 |= ADC_CR2_ADON;
	adc->CR2 |= ADC_CR2_SWSTART;
}
void PL_ADC::stop() {
	adc->CR2 &= ~ADC_CR2_SWSTART;
	adc->CR2 &= ~ADC_CR2_ADON;
}

const int8_t PL_ADC::channels[16] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5};
