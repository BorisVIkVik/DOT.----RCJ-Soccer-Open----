#include "stm32f103_PWM.h"


#define _SERVO_CNT_180 2800
#define _SERVO_CNT_0 650
const uint16_t _SERVO_K = (_SERVO_CNT_180 - _SERVO_CNT_0) / 180;


void initPWM(unsigned int timN, uint8_t channel, GPIO_TypeDef * prt, uint16_t pinNum, unsigned int freq, unsigned int clk){		//freq [Hz] (min 11Hz), clk [MHz], 		FOR ACCURATE FREQUANCY CLK[Hz]/FREQ[Hz] MUST BE INTEGER!!!
	
	//selrct timer register name
	TIM_TypeDef * tim;													
	switch(timN){
		case 1:
			tim = TIM1;
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
			break;
		case 2:
			tim = TIM2;
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
			break;
		case 3:
			tim = TIM3;
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
			break;
		case 4:
			tim = TIM4;
			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
			break;
	}
	
	//init pin
	initPin(prt, pinNum, OUTPUTAFPP);						
	
	if(tim == TIM1 || tim == TIM8){
		tim->BDTR |= TIM_BDTR_MOE;				//*enable writing to registers
		if(tim == TIM1) freq = 1000;			//TIM1 PSC must be kept at 72 - 1 for correct work of delay functions		
	}							
	
	
	tim->PSC = (clk * 1000000 / freq / 100) - 1;			//calculate frequency
	tim->ARR = 100;																	//timer counts from 0 to 100
	tim->CR1 |= TIM_CR1_ARPE;
	
	switch(channel){
		case 1:
			tim->CCR1 = 0;																				//start with 0%
			tim->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;			//select mode
			tim->CCER |= TIM_CCER_CC1E;															//enable output
			break;
		
		case 2:
			tim->CCR2 = 0;
			tim->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
			tim->CCER |= TIM_CCER_CC2E;
			break;
		
		case 3:
			tim->CCR3 = 0;
			tim->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
			tim->CCER |= TIM_CCER_CC3E;
			break;
		
		case 4:
			tim->CCR4 = 0;
			tim->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
			tim->CCER |= TIM_CCER_CC4E;
		
			break;
	}
	
	
	//enable timer
	tim->CR1 |= TIM_CR1_CEN;				
	
}


void setPWM(unsigned int timN, uint8_t channel, unsigned int value){				//value from 0 to 100
	
	//select name
	TIM_TypeDef * tim;											
	switch(timN){
		case 1: tim = TIM1; break;
		case 2: tim = TIM2; break;
		case 3: tim = TIM3; break;
		case 4: tim = TIM4; break;
	}
	
	//write value
	switch(channel){												
		case 1: tim->CCR1 = value; break;
		case 2: tim->CCR2 = value; break;
		case 3: tim->CCR3 = value; break;
		case 4: tim->CCR4 = value; break;
	}
	
}


void initServoPWM(unsigned int timN, uint8_t channel, GPIO_TypeDef * prt, uint16_t pinNum, unsigned int freq, unsigned int clk){		//freq [Hz], clk [MHz]
	
	//selrct timer register name
	TIM_TypeDef * tim;													
	switch(timN){
		case 1:
			tim = TIM1;
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
			break;
		case 2:
			tim = TIM2;
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
			break;
		case 3:
			tim = TIM3;
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
			break;
		case 4:
			tim = TIM4;
			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
			break;
	}
	
	//init pin
	initPin(prt, pinNum, OUTPUTAFPP);						
	
	if(tim == TIM1 || tim == TIM8){
		tim->BDTR |= TIM_BDTR_MOE;				//*enable writing to registers
		if(tim == TIM1) freq = 1000;			//TIM1 PSC must be kept at 72 - 1 for correct work of delay functions		
	}							
	
	
	tim->PSC = (clk * 1000000 / freq / 5000) - 1;			//calculate frequency
	tim->ARR = 5000;																	//timer counts from 0 to 1000	
	tim->CR1 |= TIM_CR1_ARPE;
	
	switch(channel){
		case 1:
			tim->CCR1 = 1500;																				//start with 50%
			tim->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;			//select mode
			tim->CCER |= TIM_CCER_CC1E;															//enable output
			break;
		
		case 2:
			tim->CCR2 = 1500;
			tim->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
			tim->CCER |= TIM_CCER_CC2E;
			break;
		
		case 3:
			tim->CCR3 = 1500;
			tim->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
			tim->CCER |= TIM_CCER_CC3E;
			break;
		
		case 4:
			tim->CCR4 = 1500;
			tim->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
			tim->CCER |= TIM_CCER_CC4E;
		
			break;
	}
	
	
	//enable timer
	tim->CR1 |= TIM_CR1_CEN;				
	
}


void setServoPWM(unsigned int timN, uint8_t channel, uint8_t value){
	
	//select name
	TIM_TypeDef * tim;											
	switch(timN){
		case 1: tim = TIM1; break;
		case 2: tim = TIM2; break;
		case 3: tim = TIM3; break;
		case 4: tim = TIM4; break;
	}
	
	if(value > 180) value = 180;
	
	//write value
	switch(channel){												
		case 1: tim->CCR1 = _SERVO_CNT_0 + _SERVO_K * value; break;
		case 2: tim->CCR2 = _SERVO_CNT_0 + _SERVO_K * value; break;
		case 3: tim->CCR3 = _SERVO_CNT_0 + _SERVO_K * value; break;
		case 4: tim->CCR4 = _SERVO_CNT_0 + _SERVO_K * value; break;
	}
	
}
