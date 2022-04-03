#include "main.h"


int main(){
	
	sysStart();
	
	
	//initUART(_UART2, 115200, 8, 1, NO_PARITY, 42);
	delay(100);
	
	
	initPin(E, 11, OUTPUTPP);
	//initPin(A, 0, INPUT, PD);

	delay(100);
	//writeStrUART(_UART2, "\r\nStart\r\n");
	
	while(1){/*
		 
	setPin(A, 6, readPin(A, 0));
		
	//	if(!readPin(E, 3)){
			invertPin(A, 7);
			delay(5000);
	//	}
		*/
		
		invertPin(E, 11);
		delay(500);
		//setPin(A, 7, 1);

	}
	
}
