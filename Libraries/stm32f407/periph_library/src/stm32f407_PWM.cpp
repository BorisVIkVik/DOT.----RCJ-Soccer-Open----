#include "stm32f407_PWM.h"


uint8_t timerType [TIMN] = {0};		//0 - undefined; 1 - usual pwm; 2 - servo pwm


void initPWM(unsigned int timN, uint8_t channel, uint16_t pin, int freq){		//freq [Hz]		FOR ACCURATE FREQUANCY CLK[Hz]/FREQ[Hz] MUST BE INTEGER!!!
	
	int APB1CLK = 84 * 1000000;
	int APB2CLK = 168 * 1000000;
	int clk = 0, afNum = 20;
	
	//select timer register name
	TIM_TypeDef * tim;													
	switch(timN){
		case 1:
			tim = TIM1;
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
			clk = APB2CLK;
			afNum = 1;
			break;
		case 2:
			tim = TIM2;
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
			clk = APB1CLK;
			afNum = 1;
			break;
		case 3:
			tim = TIM3;
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
			clk = APB1CLK;
			afNum = 2;
			break;
		case 4:
			tim = TIM4;
			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
			clk = APB1CLK;
			afNum = 2;
			break;
		case 5:
			tim = TIM5;
			RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
			clk = APB1CLK;
			afNum = 2;
			break;
		case 8:
			tim = TIM8;
			RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
			clk = APB2CLK;
			afNum = 3;
			break;
		case 9:
			tim = TIM9;
			RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
			clk = APB2CLK;
			afNum = 3;
			break;
		case 10:
			tim = TIM10;
			RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
			clk = APB2CLK;
			afNum = 3;
			break;
		case 11:
			tim = TIM11;
			RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
			clk = APB2CLK;
			afNum = 3;
			break;
		case 12:
			tim = TIM12;
			RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
			clk = APB1CLK;
			afNum = 9;
			break;
		case 13:
			tim = TIM13;
			RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
			clk = APB1CLK;
			afNum = 9;
			break;
		case 14:
			tim = TIM14;
			RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
			clk = APB1CLK;
			afNum = 9;
			break;
	}
	
	//init pin
	initPin(pin, AFPP, 0, afNum);						
	
	if(tim == TIM1 || tim == TIM8) {
		tim->BDTR |= TIM_BDTR_MOE;				//enable writing to registers	
	}							
	
																	
	if(tim == TIM7) {							//TIM7 ARR and PSC must be kept at 1000 and 84-1 for correct work of delay functions
		TIM7->ARR = 1000;
		TIM7->PSC = 84 - 1;
	}
	else{
		
		if(timerType[timN] == 0){					//if timer was not configured yet
			
			if(freq == SERVO_FREQ) {
				timerType[timN] = 2;
				tim->PSC = (clk / 50 / 3600) - 1;				//servo frequency is 1/20ms = 50Hz, timer count to 180deg / 1ms * 20ms = 3600counts 
				tim->ARR = 3600;
			}
			else {
				timerType[timN] = 1;
				tim->PSC = (clk / freq / 100) - 1;			//calculate frequency, timer counts from 0 to 100
				tim->ARR = 100;
			}
			
			tim->CR1 |= TIM_CR1_ARPE;
		}
	}
	
	switch(channel){
		case 1:
			tim->CCR1 = 0;																					//start with 0%
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


void setPWM(unsigned int timN, uint8_t channel, unsigned int value){				//value from 0 to 100 (for servo from 0 to 180)
	
	//select name
	TIM_TypeDef * tim;											
	switch(timN){
		case 1: tim = TIM1; break;
		case 2: tim = TIM2; break;
		case 3: tim = TIM3; break;
		case 4: tim = TIM4; break;
		case 5: tim = TIM5; break;
		case 8: tim = TIM8; break;
		case 9: tim = TIM9; break;
		case 10: tim = TIM10; break;
		case 11: tim = TIM11; break;
		case 12: tim = TIM12; break;
		case 13: tim = TIM13; break;
		case 14: tim = TIM14; break;

	}
	
	if(timerType[timN] == 2) {												//keep servo signal from 1ms to 2ms (180 counts = 1ms)
			value = 180 + value;
			if(value > (180 + 180)) value = 180 + 180;
	}
	
	//write value
	switch(channel){												
			case 1: tim->CCR1 = value; break;
			case 2: tim->CCR2 = value; break;
			case 3: tim->CCR3 = value; break;
			case 4: tim->CCR4 = value; break;
	}	
	
}


