#define BOUND 0.5

#include "main.h"

#define DEBUGGING 0

uint8_t getLineCount()
{
	uint8_t counter = 0;
	for(int i = 0; i < SENSORS_COUNT; i++)
		if (value[i] > BOUND) counter++;
	
	printUART(DEBUG_UART, int(counter));
	writeStrUART(DEBUG_UART, "\r\n");
	
	return 1 + max(int(counter) - 1, 0);
}

int main() 
{
	sysStart();
	
	initUART(_UART1, 115200, 8, 1, 0, 72);
	initSPI(MOTHERBOARD_SPI, SLAVE, 8);
	
	delay(100);
	writeStrUART(DEBUG_UART, "\r\nStart\r\n");
	
	initPeriph();
	
	loadCalibration();
	
	if (!calibrationIsCorrect())
	{
		clearCalibration();
		oneShotCalibration();
		
		saveCalibration();
	}
	
	
	uint8_t counter;
	
	while (true) 
	{
		readSensors();
		
		counter = getLineCount();
		if (millis() % 10 < 2 && counter != 1)
		{
			printUART(DEBUG_UART, counter);
			writeStrUART(DEBUG_UART, "\r\n");
		}
		
		setPin(LED, counter != 1);

		if (SPIAvailable(MOTHERBOARD_SPI))
		{
			uint8_t flag = readRxBufSPI(MOTHERBOARD_SPI);
			writeTxBufSPI(MOTHERBOARD_SPI, counter);
			//writeStrUART(DEBUG_UART, "\r\nasdf\r\n");
			
			clearValues();
			if (flag != 0xBB)
			{
				//printUART(DEBUG_UART, int(flag));
				//writeStrUART(DEBUG_UART, "\r\n");
				
				// CALIBRATION START 
				if (flag == CALIBRATION_START) 
				{
					writeStrUART(DEBUG_UART, "\r\nCalibration begin\r\n");
					clearCalibration();
					while (flag != CALIBRATION_END) // wait for 0x02 flag to stop calibration
					{
						oneShotCalibration();
						
						if (SPIAvailable(MOTHERBOARD_SPI)) 
						{
							flag = readRxBufSPI(MOTHERBOARD_SPI);
							writeTxBufSPI(MOTHERBOARD_SPI, getLineCount());
							
							clearValues();
						}
						
						setPin(LED, millis() % 200 < 100);
					}
					
					saveCalibration();
					
					writeStrUART(DEBUG_UART, "\r\nCalibration end\r\n");
				}
				
				
				if (flag == SENSORS_ON)
					turnSensorsOn();
				if (flag == SENSORS_OFF)
					turnSensorsOff();
				
				//writeStrUART(DEBUG_UART, "\r\nsend\r\n");
			}
		}
		
		
		/*  DEBUGGING */
		#if DEBUGGING
		for (int i = 0; i < SENSORS_COUNT / 2; i++)
		{
			printUART(_UART1, data[i]);
			writeStrUART(DEBUG_UART, "\t");
		}
		
		writeStrUART(DEBUG_UART, "|\t");
		
		for (int i = SENSORS_COUNT / 2; i < SENSORS_COUNT; i++)
		{
			printUART(_UART1, data[i]);
			writeStrUART(DEBUG_UART, "\t");
		}
		writeStrUART(DEBUG_UART, "\r\n");
	
		delay(500);
		#endif
	}
		
}
