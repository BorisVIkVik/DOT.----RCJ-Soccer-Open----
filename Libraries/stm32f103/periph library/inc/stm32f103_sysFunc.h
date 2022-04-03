#ifndef stm32f103_sysFunc
#define stm32f103_sysFunc

#include "stm32f10x.h"


void sysStart(void);

void initRCC(void);

unsigned long millis();
unsigned long micros();
void delay(uint32_t t);
void delayMicros(uint32_t t);

#endif
