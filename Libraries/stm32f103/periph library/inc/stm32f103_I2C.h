#ifndef stm32f103_I2C
#define stm32f103_I2C

#include "stm32f10x.h"
#include "stm32f103_pin.h"
#include "stdbool.h"


#define I2CQ 2

#define _I2C1 0
#define _I2C2 1

#define SLOW (bool)0
#define FAST (bool)1
#define BIT_7 (bool)0
#define BIT_10 (bool)1


void initI2C(unsigned int num, int address1, int address2, uint8_t freq = 36, bool lengthMode = BIT_7); 		//for slave with dual addressing capability
void initI2C(unsigned int num, int address1, uint8_t freq = 36, bool lengthMode = BIT_7); 		//for slave
void initI2C(unsigned int num, uint8_t freq = 36, bool lengthMode = BIT_7); 		//for master

#endif
