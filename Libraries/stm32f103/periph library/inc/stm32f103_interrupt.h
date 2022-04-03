#ifndef stm32f103_interrupt
#define stm32f103_interrupt

#include "stm32f10x.h"
#include "stm32f103_pin.h"
#include "stdbool.h"


#define A 0
#define B 1
#define C 2
#define D 3

#define RISING 0
#define FALLING 1
#define RISING_FALLING 2

void enableInterrupt(uint8_t prt, uint8_t pin, uint8_t mode, void (*func)());
void disableInterrupt(uint8_t prt, uint8_t pin);
void callInterrupt(uint8_t ch);

#endif
