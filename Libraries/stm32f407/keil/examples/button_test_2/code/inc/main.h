#include "stm32f4xx.h"
#include "stdbool.h"
#include "math.h"


/********************************* periph lib *********************************/

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_UART.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_adc.h"

/************************************* code ***********************************/


/* BUTTONS */
#define BUTTON_1 1
#define BUTTON_2 2
#define BUTTON_3 3
#define BUTTON_ALL 4

uint16_t getButtonPin(uint8_t index)
{
	if (index == BUTTON_1) return PE5;
	if (index == BUTTON_2) return PE4;
	if (index == BUTTON_3) return PE3;
	ERROR("Wrong button index in getButtonPin func");
	return 0;
}

bool readButton(uint8_t index)
{
	uint16_t pin = getButtonPin(index);
	if (pin == 0) return 0;
	return digitalRead(pin);
}

void initButton(uint8_t index)
{
	if (index == BUTTON_ALL) 
	{
		pinMode(getButtonPin(BUTTON_1), INPUT, FL);
		pinMode(getButtonPin(BUTTON_2), INPUT, FL);
		pinMode(getButtonPin(BUTTON_3), INPUT, FL);
	}
	else pinMode(getButtonPin(index), INPUT, FL);
}


/* LEDS */
#define LED_1 1
#define LED_2 2
#define LED_3 3
#define LED_ALL 4

#define ON 1
#define OFF 0

uint16_t getLEDPin(uint8_t index)
{
	if (index == BUTTON_1) return PE5;
	if (index == BUTTON_2) return PE4;
	if (index == BUTTON_3) return PE3;
	ERROR("Wrong LED index in getLEDPin func");
	return 0;
}

void setLED(uint8_t index, bool state) 
{
	uint16_t pin = getLEDPin(index);
	if (pin == 0) return;
	digitalWrite(index, state);
}

void initLED(uint8_t index)
{
	if (index == BUTTON_ALL) 
	{
		pinMode(getLEDPin(LED_1), OUTPUTPP);
		pinMode(getLEDPin(LED_2), OUTPUTPP);
		pinMode(getLEDPin(LED_3), OUTPUTPP);
	}
	else pinMode(getLEDPin(index), OUTPUTPP);
}
