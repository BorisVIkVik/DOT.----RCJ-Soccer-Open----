#ifndef stm32f103_SPI
#define stm32f103_SPI

#include "stm32f10x.h"
#include "stm32f103_pin.h"
#include "stdbool.h"


#define _SPIQ 2

#define _SPI1 0
#define _SPI2 1

#define MASTER 0
#define SLAVE 1


void initSPI(unsigned int num, bool mode, uint8_t dataFrameFormat, uint16_t clkPrescaler = 32);
uint16_t writeSPI(unsigned int num, uint16_t TxData);										//for master
void writeTxBufSPI(unsigned int num, uint16_t TxData);									//for slave
uint16_t readRxBufSPI(unsigned int num);																//for slave
bool SPIAvailable(unsigned int num);

#endif
