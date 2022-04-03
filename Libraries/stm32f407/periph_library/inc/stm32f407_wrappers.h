#ifndef STM32F407_WRAPPERS
#define STM32F407_WRAPPERS

#include "stm32f407_pin.h"
#include "stm32f407_UART.h"
//#include "connectionList.h"



#define ERROR(msg) writeStrUART(DEBUG_UART, msg);writeStrUART(DEBUG_UART, "\r\n")

inline void pinMode(uint16_t pin, uint8_t mode, uint8_t pull = FL, uint8_t af = 20) { // Wrapper for initPin function
	if (pin < 16) initPin(A, pin % 16, mode, pull, af);
	else if (pin < 32) initPin(B, pin % 16, mode, pull, af);
	else if (pin < 48) initPin(C, pin % 16, mode, pull, af);
	else if (pin < 64) initPin(D, pin % 16, mode, pull, af);
	else initPin(E, pin % 16, mode, pull, af);
}

inline bool digitalRead(uint16_t pin)
{	
	if (pin < 16) return readPin(A, pin % 16);
	else if (pin < 32) return readPin(B, pin % 16);
	else if (pin < 48) return readPin(C, pin % 16);
	else if (pin < 64) return readPin(D, pin % 16);
	else return readPin(E, pin % 16);
}

inline void digitalWrite(uint16_t pin, bool state)
{	
	if (pin < 16) setPin(A, pin % 16, state);
	else if (pin < 32) setPin(B, pin % 16, state);
	else if (pin < 48) setPin(C, pin % 16, state);
	else if (pin < 64) setPin(D, pin % 16, state);
	else setPin(E, pin % 16, state);
}

////////////////////////////////ilya pidor///////////////////////////////

inline void initPin(uint16_t pin, uint8_t mode, uint8_t pull = FL, uint8_t af = 20) { // Wrapper for initPin function
	if (pin < 16) initPin(A, pin % 16, mode, pull, af);
	else if (pin < 32) initPin(B, pin % 16, mode, pull, af);
	else if (pin < 48) initPin(C, pin % 16, mode, pull, af);
	else if (pin < 64) initPin(D, pin % 16, mode, pull, af);
	else initPin(E, pin % 16, mode, pull, af);
}

inline bool readPin(uint16_t pin)
{	
	if (pin < 16) return readPin(A, pin % 16);
	else if (pin < 32) return readPin(B, pin % 16);
	else if (pin < 48) return readPin(C, pin % 16);
	else if (pin < 64) return readPin(D, pin % 16);
	else return readPin(E, pin % 16);
}

inline void setPin(uint16_t pin, bool state)
{	
	if (pin < 16) setPin(A, pin % 16, state);
	else if (pin < 32) setPin(B, pin % 16, state);
	else if (pin < 48) setPin(C, pin % 16, state);
	else if (pin < 64) setPin(D, pin % 16, state);
	else setPin(E, pin % 16, state);
}

inline void invertPin(uint16_t pin)
{	
	if (pin < 16) invertPin(A, pin % 16);
	else if (pin < 32) invertPin(B, pin % 16);
	else if (pin < 48) invertPin(C, pin % 16);
	else if (pin < 64) invertPin(D, pin % 16);
	else invertPin(E, pin % 16);
}

#endif
