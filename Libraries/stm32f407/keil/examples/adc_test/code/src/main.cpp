#include "main.h"

#define DEBUG_UART _UART3

#define DELAY 1
/* Pins */


int main(){
	
	sysStart();
	initUART(DEBUG_UART, 115200, 8, 0, 0, 42);
	delay(1000);
	writeStrUART(DEBUG_UART, "\r\nStart\r\n");
	
	initPin(PE8, OUTPUTPP);
	setPin(PE8, 1);
	initPin(PA15, OUTPUTPP);
	setPin(PA15, 1);
	initPin(PA12, OUTPUTPP);
	setPin(PA12, 1);
	initPin(PA8, OUTPUTPP);
	setPin(PA8, 1);
	initPin(PA11, OUTPUTPP);
	setPin(PA11, 1);
	initPin(PC9, OUTPUTPP);
	setPin(PC9, 1);
	delay(10);
		
	PL_ADC ADC_1 = *(new PL_ADC(ADC1));
	ADC_1.init();
	ADC_1.add(PB0);
	ADC_1.add(PB1);
	ADC_1.add(PC0);
	ADC_1.add(PC1);
	ADC_1.start();
	
	PL_ADC ADC_2 = *(new PL_ADC(ADC2));
	ADC_2.init();
	ADC_2.add(PC2);
	ADC_2.add(PC3);
	ADC_2.start();
	
	writeStrUART(DEBUG_UART, "\r\nADCs configed\r\n");
	
	while(1) {
		writeStrUART(DEBUG_UART, "ball 1: ");
		printUART(DEBUG_UART, ADC_1.read(PB1));
		delay(DELAY);
		writeStrUART(DEBUG_UART, "	 ball 2: ");
		printUART(DEBUG_UART, ADC_1.read(PB0));
		delay(DELAY);
		writeStrUART(DEBUG_UART, "	 cell 1: ");
		printUART(DEBUG_UART, ADC_2.read(PC2) * 3.3 * (1200 + 3300) / (4096 * 3300));
		delay(DELAY);
		writeStrUART(DEBUG_UART, "	 cell 2: ");
		printUART(DEBUG_UART, ADC_1.read(PC1) * 3.3 * (5600 + 3300) / (4096 * 3300));
		delay(DELAY);
		writeStrUART(DEBUG_UART, "	 cell 3: ");
		printUART(DEBUG_UART, ADC_1.read(PC0) * 3.3 * (9100 + 2700) / (4096 * 2700));
		delay(DELAY);
		writeStrUART(DEBUG_UART, "	 cell 4: ");
		printUART(DEBUG_UART, ADC_2.read(PC3)* 3.3 * (13000 + 2700) / (4096 * 2700));
		delay(DELAY);
		writeStrUART(DEBUG_UART, "\r\n");
		
		delay(50);
	}
}
