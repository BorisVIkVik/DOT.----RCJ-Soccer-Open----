#ifndef __PWM_H_RND__
#define __PWM_H_RND__

int __pwm = 593;

void initPWM()
{
	initPin(E, 13, OUTPUTPP);
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->PSC = 55;
	TIM5->ARR = 22;
	TIM5->CR1 |= TIM_CR1_ARPE;
	TIM5->CR1 &= ~TIM_CR1_CMS;
	TIM5->CR1 &= ~TIM_CR1_DIR;
	TIM5->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM5_IRQn, 15);
	NVIC_EnableIRQ(TIM5_IRQn);
	//start counting
	TIM5->CR1 |= TIM_CR1_CEN;	
}

uint64_t __cnt = 0;

bool stp = false;
uint64_t cc = 0;

void setPWM(double percents)
{
	__pwm = 196.8 + (percents / 100.0) * 13.055;
}

extern "C" {
// 168 MHz
void TIM5_IRQHandler(void) 
{	
	// pwm handler
  __cnt = ++__cnt % 2440; // 4200 it per 21256 us => 1 it per 5.06095 us on psc=32
	// 8400 it per 19535 us => 1 it per 2.3256 us on psc = 16
	// 1 it per 7.6598 us on psc = 55
	if (__cnt == 0)
	{
		cc++;
		setPin(E, 13, 1);
	}
	
	if (__cnt == __pwm) //int(390.0 + __pwm * 4 + 0.5))
	{
		setPin(E, 13, 0);
	}

	TIM5->SR &= ~TIM_SR_UIF;
}

}


#endif