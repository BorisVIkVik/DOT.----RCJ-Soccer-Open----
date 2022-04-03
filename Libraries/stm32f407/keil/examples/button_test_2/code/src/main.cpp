#include "main.h"

int main(){
	
	sysStart();
	initUART(_UART2, 9600, 8, 1, 0, 42);
	
	initButton(BUTTON_ALL);
	initLED(LED_ALL);
	
	
	while (1)
	{
		setLED(LED_1, readButton(BUTTON_1));
		setLED(LED_2, readButton(BUTTON_2));
		setLED(LED_3, readButton(BUTTON_3));
	}
}

