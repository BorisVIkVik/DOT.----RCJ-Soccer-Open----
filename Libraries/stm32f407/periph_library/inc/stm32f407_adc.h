#ifndef STM32F407_ADC
#define STM32F407_ADC

#include "stm32f4xx.h"
#include "stdbool.h"
#include "math.h"

#include "stm32f407_wrappers.h"
#include "stm32f407_pinList.h"


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
