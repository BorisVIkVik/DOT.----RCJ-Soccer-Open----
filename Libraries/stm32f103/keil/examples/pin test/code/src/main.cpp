#include "main.h"


int main(){
	
	sysStart();
	//initUART(_UART1, 115200);
/*	
	initPin(C, 13, OUTPUTPP);
	

	initPin(B, 10, INPUTFL);

	delay(100);
	writeStrUART(_UART1, "\r\nStart\r\n");
	
	while(1){
		 
		setPin(C, 13, readPin(B, 10));

	}*/
	
	initPin(B, 12, OUTPUTPP);
	initPin(B, 13, OUTPUTPP);
	
	setPin(B, 13, 1);

	delay(100);
	//writeStrUART(_UART1, "\r\nStart\r\n");
	
	while(1){
		/*
		delay(500);
		invertPin(B, 10);
		invertPin(B, 13);
		*/
		setPin(B, 12, (millis()/500)%2);
	}
	
}
	