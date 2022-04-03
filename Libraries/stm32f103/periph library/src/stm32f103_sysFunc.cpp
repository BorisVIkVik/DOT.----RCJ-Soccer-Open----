#include "stm32f103_sysFunc.h"


uint64_t _millis = 0;


void sysStart(void){
	
	initRCC();
	
	//TIM1 configuration for delay functions
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		
	TIM1->PSC = 72 - 1;
	TIM1->ARR = 1000;
	TIM1->CR1 |= TIM_CR1_ARPE;
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 &= ~TIM_CR1_DIR;
	TIM1->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	//start counting
	TIM1->CR1 |= TIM_CR1_CEN;				
	
}


void initRCC(void){
	
	RCC->CFGR &= ~RCC_CFGR_SW;															//clear
	RCC->CFGR |= RCC_CFGR_SW_HSI;														//select source SYSCLK = HSI
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);	//wait till HSI is used
	
	RCC->CR ^= RCC_CR_PLLON;																//turn PLL off to configure it
	while((RCC->CR & RCC_CR_PLLRDY));												//wait till PLL is off
	
	RCC->CR ^= RCC_CR_HSEON;
	RCC->CR |= RCC_CR_HSEON;															//enable HSE
	while(!(RCC->CR & RCC_CR_HSERDY));										//wait till HSE is ready
	
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;		//flash clock
	
	RCC->CFGR &= ~RCC_CFGR_HPRE;													//CLEAR
	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;											//AHB = SYSCLK / 1
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;											//APB1 = HCLK / 2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;											//APB2 = HCLK / 1
	
	RCC->CFGR &= ~RCC_CFGR_PLLMULL;												//CLEAR
	RCC->CFGR &= ~RCC_CFGR_PLLSRC;
	RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;

	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE;											//SOURCE HSE
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE;										//SOURCE HSE = 8 MHz
	RCC->CFGR |= RCC_CFGR_PLLMULL9;												//PLL * 9: CLOCK = 8 MHz * 9 = 72 MHz
	
	RCC->CR |= RCC_CR_PLLON;															//enable PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));										//wait till PLL is ready
	
	RCC->CFGR &= ~RCC_CFGR_SW;														//clear
	RCC->CFGR |= RCC_CFGR_SW_PLL;													//select source SYSCLK = PLL
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1);	//wait till PLL is used
	
}

#define ULONG_MAX 4294967295UL
unsigned long millis() {
	return _millis % ULONG_MAX;
}


unsigned long micros(){
	return _millis * 1000 + TIM1->CNT;
}


void delay(uint32_t t){
	
	unsigned long timer = millis();
	while(millis() - timer < t);
	
}


void delayMicros(uint32_t t){
	
	unsigned long timer = micros();
	while(micros() - timer < t);
	
}




/*********************************** interrupt handlers ***********************************/


#ifdef __cplusplus
extern "C"{
#endif

void TIM1_UP_IRQHandler(void){			//handler for delay functions
	
	if(TIM1->SR & TIM_SR_UIF){
		TIM1->SR &= ~TIM_SR_UIF;
		_millis++;
	}
	
}

#ifdef __cplusplus
}
#endif
