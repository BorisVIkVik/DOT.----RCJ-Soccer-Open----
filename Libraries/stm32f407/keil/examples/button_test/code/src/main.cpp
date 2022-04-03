#include "main.h"

int main(){
	
	sysStart();
	initUART(_UART2, 9600, 8, 1, 0, 42);
	
	initPin(A, 6, OUTPUTPP);
	initPin(A, 7, OUTPUTPP);
	

	initPin(E, 3, INPUT, PU);
	initPin(A, 0, INPUT, PD);

	delay(100);
	writeStrUART(_UART2, "\r\nStart\r\n");
	
	
	writeUART(_UART2, 48);
		
	while(1){
		 
	setPin(A, 6, readPin(A, 0));
		
	//	if(!readPin(E, 3)){
			invertPin(A, 7);
			delay(5000);
	//	}
		
	}
	
}
