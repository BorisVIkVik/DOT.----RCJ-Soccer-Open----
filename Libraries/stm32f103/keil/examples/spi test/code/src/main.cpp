// SPI1 (master) connected to SPI2 (slave)

#include "main.h"


#define SPI2_TX_DATA 148

int main(){
	
	sysStart();
	initUART(_UART1, 115200);
	
	initPin(A, 4, OUTPUTPP);			//slave select
	initSPI(_SPI1, MASTER, 8, 128);
	initSPI(_SPI2, SLAVE, 8);
	
	delay(100);
	writeStrUART(_UART1, "\r\nStart\r\n");
	
	writeStrUART(_UART1, "SLAVE write: ");
	printUART(_UART1, SPI2_TX_DATA);
	writeStrUART(_UART1, "\r\n");
	writeTxBufSPI(_SPI2, SPI2_TX_DATA);
	
	while(1){
		
		//master
		if(UARTAvailable(_UART1)){												
			char c = readUART(_UART1);
			writeStrUART(_UART1, "MASTER write ");
			printUART(_UART1, c);
			writeStrUART(_UART1, ", MASTER read: ");
			setPin(A, 4, 0);
			printUART(_UART1, writeSPI(_SPI1, c));
			setPin(A, 4, 1);
			writeStrUART(_UART1, "\r\n");
		}
		
		//slave
		if(SPIAvailable(_SPI2)){													
			writeStrUART(_UART1, "SLAVE read: ");
			printUART(_UART1, readRxBufSPI(_SPI2));
			writeStrUART(_UART1, "\r\n");
			writeTxBufSPI(_SPI2, SPI2_TX_DATA);
		}
		
	}
	
}
