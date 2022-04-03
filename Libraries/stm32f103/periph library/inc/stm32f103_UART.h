#ifndef stm32f103_UART
#define stm32f103_UART

#include "stm32f10x.h"
#include "stm32f103_pin.h"
#include "stdbool.h"
#include "math.h"


#define UARTQ 3

#define _UART1 0
#define _UART2 1
#define _UART3 2

#define NO_PARITY 0 
#define EVEN_PARITY 1
#define ODD_PARITY 2

#define UARTbufSize 32


void initUART(unsigned int num, uint32_t baudrate,  uint8_t wordLength = 8, float _stopBits = 1, uint8_t parity = 0, unsigned int clk = 72);
void writeUART(unsigned int num, char data);
void writeStrUART(unsigned int num, char* data);
uint8_t UARTAvailable(unsigned int num);
char readUART(unsigned int num);
void tryTransmit(unsigned int num);


/********************************** functions for send variable like ASCII number **********************************/
void printUART(unsigned int num, char data);
void printUART(unsigned int num, int data);
void printUART(unsigned int num, unsigned int data);
void printUART(unsigned int num, long data);
void printUART(unsigned int num, unsigned long data);
void printUART(unsigned int num, double data);
void printUART(unsigned int num, float data);

#endif
