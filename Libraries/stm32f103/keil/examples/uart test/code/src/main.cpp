#include "main.h"

int main(){
	
	sysStart();
	initUART(_UART1, 115200);
	initUART(_UART3, 115200);
	
	initPin(C, 13, OUTPUTPP);
	
	delay(100);
	writeStrUART(_UART1, "\r\nStart\r\n");
	
	while(1){
		
		char data = 0;
		
		if(UARTAvailable(_UART3) > 0)
			while(UARTAvailable(_UART3) > 0){
				setPin(C, 13, 0);
				data = readUART(_UART3);
				printUART(_UART1, data);
				writeStrUART(_UART1, "\r\n");
			}
		else if(UARTAvailable(_UART1) > 0)
			while(UARTAvailable(_UART1) > 0){
				setPin(C, 13, 0);
				data = readUART(_UART1);
				printUART(_UART3, data);
			}
		else 
			setPin(C, 13, 1);
				
	}
	
}
