#ifndef stm32f103_pin
#define stm32f103_pin

#include "stm32f10x.h"
#include "stdbool.h"
#include "math.h"


#define A GPIOA
#define B GPIOB
#define C GPIOC
#define D GPIOD

#define OUTPUTPP 0
#define OUTPUTOD 1
#define OUTPUTAFPP 2
#define OUTPUTAFOD 3
#define INPUTAN 4
#define INPUTFL 5
#define INPUTPD 6
#define INPUTPU 7



void initPin(GPIO_TypeDef * port, uint16_t num, uint8_t mode);

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
