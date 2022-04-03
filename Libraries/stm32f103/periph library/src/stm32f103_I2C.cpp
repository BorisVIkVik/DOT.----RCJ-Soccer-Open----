#include "stm32f103_I2C.h"


void initI2C(unsigned int num, int address1, int address2, uint8_t freq, bool lengthMode) {
	
	I2C_TypeDef * i2c;
	uint8_t prt;
	uint8_t sda, scl;
	
	//select i2c register name
	switch(num) {																
	case _I2C1:
		i2c = I2C1;
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; 				//enable clk
		prt = B, sda = 7, scl = 6;						 	//choose pins
		break;
		
	case _I2C2:
		i2c = I2C2;
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		prt = B, sda = 11, scl = 10;
		break;
	}
	
	//init pins
	initPin(prt, sda, OUTPUTAFOD);
	initPin(prt, scl, OUTPUTAFOD);
	
	
	if (address1 != -1) {		//slave 
		
		//set slave address
		i2c->OAR1 = I2C_OAR1_ADD1_7 & address1;
		//dual addressing mode
		if(address2 != -1) {
			i2c->OAR2 |= I2C_OAR2_ENDUAL;
			i2c->OAR2 |= I2C_OAR2_ADD2 & address2;
		}
		
	}
	else {						//master
		
		i2c->CR2 |= I2C_CR2_FREQ & freq;
		
	}
	
	
	
	
}

void initI2C(unsigned int num, int address1, uint8_t freq, bool lengthMode) {
	
	initI2C(num, address1, -1, freq, lengthMode);
	
}


void initI2C(unsigned int num, uint8_t freq, bool lengthMode) {
	
	initI2C(num, -1, -1, freq, lengthMode);
	
}
