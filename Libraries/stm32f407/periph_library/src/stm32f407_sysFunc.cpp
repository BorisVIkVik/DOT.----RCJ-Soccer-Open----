#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"


unsigned long _millis = 0;


void sysStart(void){
	
	initRCC();
	
	//TIM7 configuration for delay functions
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;		
	TIM7->PSC = 84 - 1;
	TIM7->ARR = 1000;
	TIM7->CR1 |= TIM_CR1_ARPE;
	TIM7->CR1 &= ~TIM_CR1_CMS;
	TIM7->CR1 &= ~TIM_CR1_DIR;
	TIM7->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 1);
	//start counting
	TIM7->CR1 |= TIM_CR1_CEN;				
	
}


void initRCC(void){
	
	RCC->CFGR &= ~RCC_CFGR_SW;									//clear
	RCC->CFGR |= RCC_CFGR_SW_HSI;								//select source SYSCLK = HSI
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);		//wait till HSI is used
	
	RCC->CR &= ~RCC_CR_PLLON;					//turn PLL off to configure it
	while((RCC->CR & RCC_CR_PLLRDY));			//wait till PLL is off
	
	RCC->CR |= RCC_CR_HSEON;					//enable HSE
	while(!(RCC->CR & RCC_CR_HSERDY));			//wait till HSE is ready
		
	RCC->CFGR &= ~RCC_CFGR_HPRE;						//CLEAR
	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;				//AHB = SYSCLK / 1
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;				//APB1 = HCLK / 4
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;				//APB2 = HCLK / 2
	
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;				//CLEAR
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;				//SOURCE HSE
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2;																							//SOURCE HSE = 8 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2;																							//8 / 4 = 2MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7;		//2 * 168 = 336MHz
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; 																							//336 / 2 = 168MHz
	
	RCC->CR |= RCC_CR_PLLON;								//enable PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));			//wait till PLL is ready
	
	RCC->CFGR &= ~RCC_CFGR_SW;					//clear
	RCC->CFGR |= RCC_CFGR_SW_PLL;				//select source SYSCLK = PLL
	while((RCC->CFGR & RCC_CFGR_SWS) == 2);		//wait till PLL is used
	
	//////////
	/*RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;			//enable MCO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	initPin(A, 8, AFPP, PU);
	
	//GPIOA->CRH &= ~GPIO_CRH_CNF8_0;				//configure MCO
	//GPIOA->CRH |= GPIO_CRH_CNF8_1;
	//GPIOA->CRH |= GPIO_CRH_MODE8;
	
	RCC->CFGR |= RCC_CFGR_MCO1_0;					//select PLL / 2 as MCO source
	RCC->CFGR |= RCC_CFGR_MCO1_1;
	
	RCC->CFGR |= RCC_CFGR_MCO1PRE_0;
	//////////
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;			//enable MCO
	
	GPIOA->CRH &= ~GPIO_CRH_CNF8_0;				//configure MCO
	GPIOA->CRH |= GPIO_CRH_CNF8_1;
	GPIOA->CRH |= GPIO_CRH_MODE8;
	
	RCC->CFGR &= ~RCC_CFGR_MCO;					//select PLL / 2 as MCO source
	RCC->CFGR |= RCC_CFGR_MCO_PLL;
	*/
}


unsigned long millis(){
	return _millis;
}


unsigned long micros(){
	return _millis * 1000 + TIM7->CNT;
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

void TIM7_IRQHandler(void){			//handler for delay functions
	
	if(TIM7->SR & TIM_SR_UIF){
		TIM7->SR &= ~TIM_SR_UIF;
		_millis++;
	}
}

#ifdef __cplusplus
}
#endif
