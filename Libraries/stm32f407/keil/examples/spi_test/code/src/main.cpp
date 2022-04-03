// SPI2 (master) connected to SPI3 (slave)

#include "main.h"


#define SPI3_TX_DATA 148

inline void slaveInit(int index) {
	if (index == _SPI1) initPin(A, 4, OUTPUTPP);
	if (index == _SPI2) initPin(B, 12, OUTPUTPP);
	if (index == _SPI3) initPin(A, 15, OUTPUTPP);
}

inline void slaveEnable(int index) {
	if (index == _SPI1) setPin(A, 4, 0);
	if (index == _SPI2) setPin(B, 12, 0);
	if (index == _SPI3) setPin(A, 15, 0);
}

inline void slaveDisable(int index) {
	if (index == _SPI1) setPin(A, 4, 1);
	if (index == _SPI2) setPin(B, 12, 1);
	if (index == _SPI3) setPin(A, 15, 1);
}

int counter = 0;

int main() {
	
	sysStart();
	//initUART(_UART1, 115200);
	
	delay(1000);
	
	initSPI(_SPI1, MASTER, 8, 32);
	pinMode(PE10, OUTPUTPP);
	setPin(PE10, 1);
	
	delay(1000);
	
	while(1)
	{
		setPin(PE10, 0);
		writeSPI(_SPI1, 3);
		setPin(PE10, 1);
		
		delay(2);
	}
	/*
	slaveInit(_SPI3);
	//initSPI(_SPI3, MASTER, 8, 2);
	initSPI(_SPI1, MASTER, 8, 2);
	initSPI(_SPI2, SLAVE, 8);
	
	delay(1000);
	writeStrUART(_UART1, "\r\nStart_complete\r\n");
	
	while (1) {
		
		//update slave tx buffer
		writeTxBufSPI(_SPI2, SPI3_TX_DATA); 
		
		//master
		if(UARTAvailable(_UART1)) {	
			char c = readUART(_UART1);
			writeStrUART(_UART1, "MASTER write ");
			printUART(_UART1, c);
			writeStrUART(_UART1, ", MASTER read: ");
			counter++;
			//if (counter % 2) {
				slaveEnable(_SPI3);
				printUART(_UART1, writeSPI(_SPI1, c));
				slaveDisable(_SPI3);
			//} else {
			//	slaveEnable(_SPI2);
			//	printUART(_UART1, writeSPI(_SPI2, c));
			//	slaveDisable(_SPI2);
			//}
			writeStrUART(_UART1, "\r\n");
		}
		
		//slave
		if(SPIAvailable(_SPI2)) {													
			writeStrUART(_UART1, "SLAVE read: ");
			printUART(_UART1, readRxBufSPI(_SPI2));
			writeStrUART(_UART1, "\r\n");
		}
		
	}
	*/
}
