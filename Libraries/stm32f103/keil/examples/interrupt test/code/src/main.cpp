#include "main.h"


int main(){
	
	sysStart();
	initUART(_UART1, 115200);
	initPin(C, 13, OUTPUTPP);
	
	initPin(A, 3, INPUTPD);
	enableInterrupt(A, 3, RISING, handler1);
	initPin(B, 8, INPUTPU);
	enableInterrupt(B, 8, FALLING, handler2);
	initPin(C, 15, INPUTPU);
	enableInterrupt(C, 15, RISING_FALLING, handler3);
	
	delay(100);
	writeStrUART(_UART1, "\r\nStart\r\n");

	
	while(1){
		
		if(UARTAvailable(_UART1)){
			readUART(_UART1);
			writeStrUART(_UART1, "call interrupt\r\n");
			callInterrupt(8);
		}
		
		setPin(C, 13, 1);
		
	}
	
}


void handler1(){
	
	setPin(C, 13, 0);
	writeStrUART(_UART1, "interrupt 1\r\n");
	
}

void handler2(){
	
	setPin(C, 13, 0);
	writeStrUART(_UART1, "interrupt 2\r\n");
	
}

void handler3(){
	
	setPin(C, 13, 0);
	writeStrUART(_UART1, "interrupt 3\r\n");
	
}
