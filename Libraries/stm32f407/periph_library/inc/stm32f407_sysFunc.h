#ifndef stm32f407_sysFunc
#define stm32f407_sysFunc

#include "stm32f4xx.h"


void sysStart(void);

void initRCC(void);

unsigned long millis();
unsigned long micros();
void delay(uint32_t t);
void delayMicros(uint32_t t);

#endif
