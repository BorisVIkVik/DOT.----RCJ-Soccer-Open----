#ifndef stm32f407_PWM
#define stm32f407_PWM

#include "stm32f4xx.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_sysFunc.h"
#include "stdbool.h"



#define TIMN 14
#define CHN 4

#define _TIM1 1
#define _TIM2 2
#define _TIM3 3
#define _TIM4 4
#define _TIM5 5
#define _TIM8 8
#define _TIM9 9
#define _TIM10 10
#define _TIM11 11
#define _TIM12 12
#define _TIM13 13
#define _TIM14 14

#define _CH1 1
#define _CH2 2
#define _CH3 3
#define _CH4 4


#define SERVO_FREQ -1

void initPWM(unsigned int timN, uint8_t channel, uint16_t pin, int freq);
void setPWM(unsigned int timN, uint8_t channel, unsigned int value);

#endif
