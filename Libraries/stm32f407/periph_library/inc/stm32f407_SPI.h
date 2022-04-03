#ifndef stm32f407_SPI
#define stm32f407_SPI

#include "stm32f4xx.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stdbool.h"

#define _SPIQ 2

#define _SPI1 0
#define _SPI2 1
#define _SPI3 2

#define MASTER 0
#define SLAVE 1


void initSPI(unsigned int num, bool mode, uint8_t dataFrameFormat, uint16_t clkPrescaler = 32, int nss = -1, int sck = -1, int miso = -1, int mosi = -1);
uint16_t writeSPI(unsigned int num, uint16_t TxData);								//for master
void writeTxBufSPI(unsigned int num, uint16_t TxData);						//for slave
uint16_t readRxBufSPI(unsigned int num);								//for slave
bool SPIAvailable(unsigned int num);

#endif
