#include "stm32f103_pin.h"



void initPin(GPIO_TypeDef * port, uint16_t num, uint8_t mode) {
	
	//select gpio register name & set clk
	if(port == A)
			RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;								
	else if(port == B)
			RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(port == C)
			RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	else if(port == D)
			RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	

	//choose CRL or CRH
	uint32_t N = num;
	if(N > 7){																						
		N -= 8;
		N = pow((double)16, (double)N);
		
		switch(mode){
			case OUTPUTPP:
				port->CRH &= ~((uint32_t)0x0000000C * N);				//CNF = 00
				port->CRH |= ((uint32_t)0x00000003 * N);    		//MODE = 11
				break;
			
			case OUTPUTOD:
				port->CRH &= ~((uint32_t)0x00000008 * N);				//CNF = 01
				port->CRH |= ((uint32_t)0x00000004 * N);
				port->CRH |= ((uint32_t)0x00000003 * N);				//MODE = 11
				break;
			
			case OUTPUTAFPP:
				RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;							//enable alternative function clock
				port->CRH |= ((uint32_t)0x00000008 * N);				//CNF = 10
				port->CRH &= ~((uint32_t)0x00000004 * N);
				port->CRH |= ((uint32_t)0x00000003 * N);    		//MODE = 11
				break;
			
			case OUTPUTAFOD:
				RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;							//enable alternative function clock
				port->CRH |= ((uint32_t)0x0000000C * N);				//CNF = 11
				port->CRH |= ((uint32_t)0x00000003 * N);				//MODE = 11
				break;
			
			case INPUTAN:
				port->CRH &= ~((uint32_t)0x0000000C * N);				//CNF = 00
				port->CRH &= ~((uint32_t)0x00000003 * N);   		//MODE = 00
				break;
			
			case INPUTFL:
				port->CRH &= ~((uint32_t)0x00000008 * N);				//CNF = 01
				port->CRH |= ((uint32_t)0x00000004 * N);
				port->CRH &= ~((uint32_t)0x00000003 * N);				//MODE = 00
				break;
			
			case INPUTPD:
				port->CRH |= ((uint32_t)0x00000008 * N);				//CNF = 10
				port->CRH &= ~((uint32_t)0x00000004 * N);
				port->CRH &= ~((uint32_t)0x00000003 * N);				//MODE = 00
				port->ODR &= ~((uint16_t)0x0001 << num);				//ODR = 0
				break;
			
			case INPUTPU:
				port->CRH |= ((uint32_t)0x00000008 * N);				//CNF = 10
				port->CRH &= ~((uint32_t)0x00000004 * N);
				port->CRH &= ~((uint32_t)0x00000003 * N);				//MODE = 00
				port->ODR |= ((uint16_t) 0x0001 << num);				//ODR = 1
				break;
			
			default:
				break;
		}	
	}
	else {
		N = pow((double)16, (double)N);
		
		switch(mode){
			case OUTPUTPP:
				port->CRL &= ~((uint32_t)0x0000000C * N);				//CNF = 00
				port->CRL |= ((uint32_t)0x00000003 * N);   			//MODE = 11
				break;
			
			case OUTPUTOD:
				port->CRL &= ~((uint32_t)0x00000008 * N);				//CNF = 01
				port->CRL |= ((uint32_t)0x00000004 * N);
				port->CRL |= ((uint32_t)0x00000003 * N);				//MODE = 11
				break;
			
			case OUTPUTAFPP:
				RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;							//enable alternative function clock
				port->CRL |= ((uint32_t)0x00000008 * N);				//CNF = 10
				port->CRL &= ~((uint32_t)0x00000004 * N);
				port->CRL |= ((uint32_t)0x00000003 * N);    		//MODE = 11
				break;
			
			case OUTPUTAFOD:
				RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;							//enable alternative function clock
				port->CRL |= ((uint32_t)0x0000000C * N);				//CNF = 11
				port->CRL |= ((uint32_t)0x00000003 * N);				//MODE = 11
				break;
			
			case INPUTAN:
				port->CRL &= ~((uint32_t)0x0000000C * N);				//CNF = 00
				port->CRL &= ~((uint32_t)0x00000003 * N);  		  //MODE = 00
				break;
			
			case INPUTFL:
				port->CRL &= ~((uint32_t)0x00000008 * N);				//CNF = 01
				port->CRL |= ((uint32_t)0x00000004 * N);
				port->CRL &= ~((uint32_t)0x00000003 * N);				//MODE = 00
				break;
			
			case INPUTPD:
				port->CRL |= ((uint32_t)0x00000008 * N);				//CNF = 10
				port->CRL &= ~((uint32_t)0x00000004 * N);
				port->CRL &= ~((uint32_t)0x00000003 * N);				//MODE = 00
				port->ODR &= ~((uint16_t)0x0001 << num);				//ODR = 0
				break;
			
			case INPUTPU:
				port->CRL |= ((uint32_t)0x00000008 * N);				//CNF = 10
				port->CRL &= ~((uint32_t)0x00000004 * N);
				port->CRL &= ~((uint32_t)0x00000003 * N);				//MODE = 00
				port->ODR |= ((uint16_t) 0x0001 << num);				//ODR = 1
				break;
			
			default:
				break;
		}
	}
}
