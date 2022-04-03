#ifndef stm32f103_ADC
#define stm32f103_ADC

#include "stm32f10x.h"
#include "stm32f103_pin.h"


#define ADCQ 3

#define _ADC1 0
#define _ADC2 1
#define _ADC3 2

void resetADC(void);
void initADC(unsigned int adcNum, uint8_t channel, uint16_t prt, uint16_t pin, bool contMode = 0);
uint16_t readADC(unsigned int adcNum, uint8_t channel);

#endif
