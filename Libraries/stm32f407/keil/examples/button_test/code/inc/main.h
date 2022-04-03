#include "stm32f4xx.h"
#include "stdbool.h"
#include "math.h"


/********************************* periph lib *********************************/

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_UART.h"

/********************************* code *********************************/

#define ERROR(x) (while(0);) // Надо забацать какую-нибудь функцию для логов ошибок

#define BUTTON_1 1
#define BUTTON_2 2
#define BUTTON_3 3
#define BUTTON_ALL 4

uint8_t getButtonPin(uint8_t index)
{
	if (index == BUTTON_1) return PE5;
	if (index == BUTTON_2) return PE4;
	if (index == BUTTON_3) return PE3;
	return 0;
}

bool readButton(uint8_t index)
{
	int pin = getButtonPin();
	if (pin == 0) ERROR("Wrong button index in readButton func");
	return 
}

void initButton(uint8_t index)
{
	int pin = getButtonPin(index);
	initPin(A, 0, INPUT, PD);
}
	