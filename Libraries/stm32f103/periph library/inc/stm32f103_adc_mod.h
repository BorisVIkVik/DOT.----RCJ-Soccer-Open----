#ifndef STM32F103_ADC_MOD
#define STM32F103_ADC_MOD

#include "stm32f10x.h"
#include "stdbool.h"
#include "math.h"

#include "stm32f103_wrappers.h"
#include "stm32f103_pinList.h"


#define powerOnADC(adc) adc->CR2|=ADC_CR2_ADON
#define fastStart(adc) adc->CR2|=ADC_CR2_ADON;adc->CR2|=ADC_CR2_SWSTART
#define waitEndOfConversion(adc) while(!(adc->SR&ADC_SR_EOC)){};asm("nop")
#define waitEndOfInjectedConversion(adc) while(!(adc->SR&ADC_SR_JEOC)){};asm("nop")
#define fastRead(adc, chn) (adc->chn)
#define fastCheatRead(adc) (adc->DR)


class PL_ADC {
	public:
		int8_t index;
		int8_t usedCount; // Count of used injected channels
	
		PL_ADC(ADC_TypeDef* adc = ADC1);
		
		void waitUpdate();
		unsigned read(int8_t pin, bool withDelay = true/*set true if you need to wait next conversation, waiting ~6ns*/);
		
		bool add(int8_t pin);
		
		void init();
		void start();
		void stop();

	private:
		ADC_TypeDef* adc;
		const static int8_t channels[16]; // Channel to pin array converter
		int8_t jdr[4]; // pin to JDRx array converter
};


#endif
