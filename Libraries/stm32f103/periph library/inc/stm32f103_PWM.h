#ifndef stm32f103_PWM
#define stm32f103_PWM

#include "stm32f10x.h"
#include "stm32f103_pin.h"


#define _TIM1 1
#define _TIM2 2
#define _TIM3 3
#define _TIM4 4


void initPWM(unsigned int timN, uint8_t channel, GPIO_TypeDef * prt, uint16_t pinNum, unsigned int freq, unsigned int clk = 72);
void initServoPWM(unsigned int timN, uint8_t channel, GPIO_TypeDef * prt, uint16_t pinNum, unsigned int freq, unsigned int clk = 72);
void setPWM(unsigned int timN, uint8_t channel, unsigned int value);
void setServoPWM(unsigned int timN, uint8_t channel, uint8_t value);

#endif
