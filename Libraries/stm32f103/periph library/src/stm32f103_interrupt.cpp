#include "stm32f103_interrupt.h"


void (*_interruptFunction[16])();


void enableInterrupt(uint8_t prt, uint8_t pin, uint8_t mode, void (*func)()){
	
	//enable clk
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;									
	
	//enable interrupt
	EXTI->IMR |= 1 << pin;															
	
	//select mode (rising, falling or both)
	if(mode == RISING) EXTI->RTSR |= 1 << pin;					
	else if(mode == FALLING) EXTI->FTSR |= 1 << pin;
	else if(mode == RISING_FALLING) EXTI->RTSR |= 1 << pin, EXTI->FTSR |= 1 << pin;
	
	//connect choosen pin
	AFIO->EXTICR[pin  / 4] &= ~(0xF << (4 * (pin % 4)));
	AFIO->EXTICR[pin  / 4] |= prt << (4 * (pin % 4));		
	
	//enable IRQ
	if(pin == 0)NVIC_EnableIRQ(EXTI0_IRQn);							
	else if(pin == 1)NVIC_EnableIRQ(EXTI1_IRQn);
	else if(pin == 2)NVIC_EnableIRQ(EXTI2_IRQn);
	else if(pin == 3)NVIC_EnableIRQ(EXTI3_IRQn);
	else if(pin == 4)NVIC_EnableIRQ(EXTI4_IRQn);
	else if(pin <= 9)NVIC_EnableIRQ(EXTI9_5_IRQn);
	else if(pin <= 15)NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	_interruptFunction[pin] = func;
}

void disableInterrupt(uint8_t prt, uint8_t pin){
		
	//disable interrupt
	EXTI->IMR &= ~(1 << pin);											
	
	//clear mode
	EXTI->RTSR &= ~(1 << pin);										
	EXTI->FTSR &= ~(1 << pin);
		
	//diconnect pin
	AFIO->EXTICR[pin / 4] &= ~(0xF << (4 * (pin % 4)));		
	
}


void callInterrupt(uint8_t ch){
		
	EXTI->SWIER |= 1 << ch;
	
}


void _handler(uint8_t num){				//actual interrupt handler
	
	_interruptFunction[num]();
	
}


/**************************************** interrupt handlers ****************************************/

#ifdef __cplusplus
extern "C"{
#endif
	
void EXTI0_IRQHandler(void){
	_handler(0);
	EXTI->PR |= EXTI_PR_PR0;
}	
	
void EXTI1_IRQHandler(void){
	_handler(1);
	EXTI->PR |= EXTI_PR_PR1;
}	
	
void EXTI2_IRQHandler(void){
	_handler(2);
	EXTI->PR |= EXTI_PR_PR2;
}
	
void EXTI3_IRQHandler(void){
	_handler(3);
	EXTI->PR |= EXTI_PR_PR3;
}

void EXTI4_IRQHandler(void){
	_handler(4);
	EXTI->PR |= EXTI_PR_PR4;
}

void EXTI9_5_IRQHandler(void){
	
	if(EXTI->PR & EXTI_PR_PR5){
		_handler(5);
		EXTI->PR |= EXTI_PR_PR5;
	}
	else if(EXTI->PR & EXTI_PR_PR6){
		_handler(6);
		EXTI->PR |= EXTI_PR_PR6;
	}
	else if(EXTI->PR & EXTI_PR_PR7){
		_handler(7);
		EXTI->PR |= EXTI_PR_PR7;
	}
	else if(EXTI->PR & EXTI_PR_PR8){
		_handler(8);
		EXTI->PR |= EXTI_PR_PR8;
	}
	else if(EXTI->PR & EXTI_PR_PR9){
		_handler(9);
		EXTI->PR |= EXTI_PR_PR9;
	}
	
}

void EXTI15_10_IRQHandler(void){
	
	if(EXTI->PR & EXTI_PR_PR10){
		_handler(10);
		EXTI->PR |= EXTI_PR_PR10;
	}
	else if(EXTI->PR & EXTI_PR_PR11){
		_handler(11);
		EXTI->PR |= EXTI_PR_PR11;
	}
	else if(EXTI->PR & EXTI_PR_PR12){
		_handler(12);
		EXTI->PR |= EXTI_PR_PR12;
	}
	else if(EXTI->PR & EXTI_PR_PR13){
		_handler(13);
		EXTI->PR |= EXTI_PR_PR13;
	}
	else if(EXTI->PR & EXTI_PR_PR14){
		_handler(14);
		EXTI->PR |= EXTI_PR_PR14;
	}
	else if(EXTI->PR & EXTI_PR_PR15){
		_handler(15);
		EXTI->PR |= EXTI_PR_PR15;
	}
	
}

#ifdef __cplusplus
}
#endif
