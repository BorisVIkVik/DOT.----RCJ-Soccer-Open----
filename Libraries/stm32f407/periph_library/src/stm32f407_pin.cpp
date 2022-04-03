#include "stm32f407_pin.h"


void initPin(GPIO_TypeDef * port, uint16_t num, uint8_t mode, uint8_t pull, uint8_t af) {
	
	//select gpio register name & set clk
	if (port == A)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;								
	else if (port == B)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	else if (port == C)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	else if (port == D)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	else if (port == E)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	

	unsigned int N = pow((double)4, (double)num);
		
	switch(mode){
		case OUTPUTPP:
			port->MODER |= ((uint32_t)0x00000001 * N);
			port->MODER &= ~((uint32_t)0x00000002 * N);    			//MODE = 01
			port->OTYPER &= ~((uint32_t)0x00000001 << num);			//OTYPER = 0
			break;
		
		case OUTPUTOD:
			port->MODER |= ((uint32_t)0x00000001 * N);
			port->MODER &= ~((uint32_t)0x00000002 * N);    			//MODE = 01
			port->OTYPER |= ((uint32_t)0x00000001 << num);			//OTYPER = 1
			break;
		
		case AFPP:
			port->MODER &= ~((uint32_t)0x00000001 * N);
			port->MODER |= ((uint32_t)0x00000002 * N);    			//MODE = 10
			port->OTYPER &= ~((uint32_t)0x00000001 << num);			//OTYPER = 0
			break;
		
		case AFOD:
			port->MODER &= ~((uint32_t)0x00000001 * N);
			port->MODER |= ((uint32_t)0x00000002 * N);    			//MODE = 10
			port->OTYPER &= ~((uint32_t)0x00000001 << num);			//OTYPER = 1
			break;
		
		case INPUTAN:
			port->MODER |= ((uint32_t)0x00000003 * N);    			//MODE = 11
			port->PUPDR &= ~((uint32_t)0x00000003 * N);				//PUPDR = 00
			break;
		
		case INPUT:
			port->MODER &= ~((uint32_t)0x00000003 * N);    			//MODE = 00
			break;

		default:
			break;
	}
	
	
	//set pull up or pull down
	if(pull != 0 && mode != INPUTAN) {
		
		if (pull == PU) {
			port->PUPDR |= ((uint32_t)0x00000001 * N);				
			port->PUPDR &= ~((uint32_t)0x00000002 * N); 				//PUPDR = 01
		}
		else if (pull == PD) {
			port->PUPDR &= ~((uint32_t)0x00000001 * N);				
			port->PUPDR |= ((uint32_t)0x00000002 * N); 				//PUPDR = 10
		}
		
	}
	else {
		port->PUPDR &= ~((uint32_t)0x00000003 * N); 					//PUPDR = 00
	}
	
	
	//set speed
	port->OSPEEDR |= ((uint32_t)0x00000003 * N); 				//OSPEEDR = 11
	
	
	//select alternative function
	if (af != 20){
		
		N = num;
		if(num > 7)
			N = num - 8;
		
		port->AFR[num / 8] |= af << (4 * N);
	}	
	
}


