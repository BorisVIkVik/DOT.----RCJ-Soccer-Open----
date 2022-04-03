#ifndef STM32F407_PIN
#define STM32F407_PIN

#include "stm32f4xx.h"
#include "stdbool.h"
#include "math.h"


#define A GPIOA
#define B GPIOB
#define C GPIOC
#define D GPIOD
#define E GPIOE

#define OUTPUTPP 0
#define OUTPUTOD 1
#define AFPP 2
#define AFOD 3
#define INPUTAN 4
#define INPUT 5

#define FL 0
#define PU 1
#define PD 2


void initPin(GPIO_TypeDef * port, uint16_t num, uint8_t mode, uint8_t pull = FL, uint8_t af = 20);

inline void setPin(GPIO_TypeDef * port, uint16_t num, bool state) {
	if (state) port->ODR |= ((uint16_t)0x0001 << num);
	else port->ODR &= ~((uint16_t)0x0001 << num);
}

inline void invertPin(GPIO_TypeDef * port, uint16_t num) {
	port->ODR ^= ((uint16_t)0x0001 << num);
}

inline bool readPin(GPIO_TypeDef * port, uint16_t num) {
	return port->IDR & ((uint16_t)0x0001 << num);
}

#endif
