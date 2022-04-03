#ifndef STM32F103_WRAPPERS
#define STM32F103_WRAPPERS

#include "stm32f103_pin.h"
#include "stm32f103_UART.h"



#define ERROR(msg) while(0);//writeStrUART(_UART1, msg);writeStrUART(_UART1, "\r\n")

inline void pinMode(uint16_t pin, uint8_t mode) { // Wrapper for initPin function
	if (pin < 16) initPin(A, pin % 16, mode);
	else if (pin < 32) initPin(B, pin % 16, mode);
	else if (pin < 48) initPin(C, pin % 16, mode);
	else if (pin < 64) initPin(D, pin % 16, mode);
}

inline void initPin(uint16_t pin, uint8_t mode) { // Wrapper for initPin function
	if (pin < 16) initPin(A, pin % 16, mode);
	else if (pin < 32) initPin(B, pin % 16, mode);
	else if (pin < 48) initPin(C, pin % 16, mode);
	else if (pin < 64) initPin(D, pin % 16, mode);
}

inline bool readPin(uint16_t pin)
{	
	if (pin < 16) return readPin(A, pin % 16);
	else if (pin < 32) return readPin(B, pin % 16);
	else if (pin < 48) return readPin(C, pin % 16);
	else if (pin < 64) return readPin(D, pin % 16);
}

inline void setPin(uint16_t pin, bool state)
{	
	if (pin < 16) setPin(A, pin % 16, state);
	else if (pin < 32) setPin(B, pin % 16, state);
	else if (pin < 48) setPin(C, pin % 16, state);
	else if (pin < 64) setPin(D, pin % 16, state);
}

inline void invertPin(uint16_t pin)
{	
	if (pin < 16) invertPin(A, pin % 16);
	else if (pin < 32) invertPin(B, pin % 16);
	else if (pin < 48) invertPin(C, pin % 16);
	else if (pin < 64) invertPin(D, pin % 16);
}

#endif
