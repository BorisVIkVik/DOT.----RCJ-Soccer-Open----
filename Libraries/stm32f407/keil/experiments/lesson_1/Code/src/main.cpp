#include "main.h"


void delay(uint32_t time) {
	uint32_t i;
	
	for (i = 0; i < time; i++);
}

int main() {	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODER6_1;
	GPIOA->MODER |= GPIO_MODER_MODER6_0;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_6;
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0;
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6_1;
	
	
	while(1) 
	{
		GPIOA->ODR ^= GPIO_ODR_ODR_6;
		delay(3000000);
	}
	
}
