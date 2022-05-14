#ifndef CAMERA
#define CAMERA

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"
#include "connectionList.h"
#include "Errors.h"
#include "tools.h"


class Camera
{
	public:
		Camera();
		void init(unsigned int spi, uint16_t ss);
		unsigned int update();
		uint8_t objects;
		pair<int, int> ball, blue, yellow;
	
	private:
		unsigned int spi;
		uint16_t ss;	
		long long int lastRecieve;
};

Camera::Camera()
{
	objects = 0;
	ball.X = 0;
	ball.Y = 0;
	blue.X = 0;
	blue.Y = 0;
	yellow.X = 0;
	yellow.Y = 0;
	lastRecieve = millis() - 200;
}

void Camera::init(unsigned int spi, uint16_t ss)
{
	this->spi = spi;
	this->ss = ss;
		
	initPin(ss, OUTPUTPP);
	setPin(ss, 1);
}


unsigned int Camera::update()
{
	
	unsigned int error = 0;
	long long int timeout = millis();
	//objects = 0;
	setPin(this->ss, 0);
	uint8_t startByte = writeSPI(spi, 47);
	setPin(this->ss, 1);
	while(startByte != 0xBB)
	{
		if( millis() - timeout > 10)
		{
			//error = CAMERA_CONNECTION_ERROR;
			break;
		}
		setPin(this->ss, 0);
		startByte = writeSPI(spi, 47);
		setPin(this->ss, 1);
	}
	if(error == CAMERA_CONNECTION_ERROR)
		{
		setPin(LED_2, 0);		
		return error;
	}

	setPin(this->ss, 0);
	uint8_t msgLen = writeSPI(spi, 47);
	msgLen = 7;
	uint8_t rxData[10];
	rxData[0] = 0xBB;
	rxData[1] = msgLen;
	for(int iter = 0; iter < msgLen; iter++)
		rxData[iter + 2] = writeSPI(spi, 47);
	uint8_t recvCrc8 = writeSPI(spi, 47);
	setPin(this->ss, 1);
	
	if(crc8(rxData, msgLen + 2) == recvCrc8)
	{
		setPin(LED_2, 1);
		objects = rxData[2];
		ball.X = (rxData[3] * 2) - 240;
		ball.Y = (rxData[4] * 2) - 240;
		yellow.X = (rxData[5] * 2) - 240;
		yellow.Y = (rxData[6] * 2) - 240;
		blue.X = (rxData[7] * 2) - 240;
		blue.Y = (rxData[8] * 2) - 240;
		lastRecieve = millis();
	}
	else
	{
		setPin(LED_2, 0);				
		error |= CAMERA_DATA_ERROR;
	}
	
	if(millis() - lastRecieve > 200)
		error |= CAMERA_CONNECTION_ERROR;
	
	return error;
//	if(ballX == 46)
//	{
//		setPin(this->ss, 0);
//		ballY = writeSPI(spi, 47);
//		setPin(this->ss, 1);
//		if(ballY == 254)
//		{
//			setPin(LED_2, 1);	
//		}	
//	}
//	else
//	{
//		setPin(LED_2, 0);
//	}
	
//	static const int MSG_SIZE = 6;
//	static int ptr = 0, upperBound = MSG_SIZE, msg[MSG_SIZE];
//	static bool inSeq = false;
//	static uint32_t lastUseTime = millis();
//	
//		lastUseTime = millis();
//		setPin(this->ss, 0);
//		msg[0] = writeSPI(CAMERA_SPI, 47) * 2 - 139;
////		msg[1] = writeSPI(CAMERA_SPI, 47) * 2 - 139;
////		msg[2] = writeSPI(CAMERA_SPI, 47) * 2 - 139;
////		msg[3] = writeSPI(CAMERA_SPI, 47) * 2 - 139;
////		msg[4] = writeSPI(CAMERA_SPI, 47) * 2 - 139;
////		msg[5] = writeSPI(CAMERA_SPI, 47) * 2 - 139;
//		setPin(this->ss, 1);
//		if (symb == 0xBB && !inSeq) // if start of message detected
//		{
//			inSeq = true;
//			ptr = 0;
//		}
//		
//		if (inSeq)
//		{
//			msg[ptr++] = symb;
//			
//			if (ptr == upperBound) // message completed
//			{
//				char crc = crc8(msg, MSG_SIZE - 1);
//				if (crc == msg[MSG_SIZE - 1])
//				{
//					ballX = msg[1];
//					b allY = msg[2]; 
//					yGoalX = msg[3];
//					yGoalY = msg[4];
//				}
//				else
//				{
//					GLOBAL_ERROR |= CAMERA_DATA_ERROR;
//				}
//				
//				ptr = 0;
//				inSeq = false;
//			}
//		}
//	
//	
//	if (millis() - lastUseTime > 100) 
//	{
//		GLOBAL_ERROR |= CAMERA_CONNECTION_ERROR;
//	}
//	
//	if (ptr >= MSG_SIZE) 
//	{
//		inSeq = false;
//		ptr = 0;
//	}
	
//	
	
	
	//OLD VIM UPDATING CODE
	/*
	static const int MSG_SIZE = 11;
	static int ptr = 0, upperBound = MSG_SIZE, msg[MSG_SIZE];
	static bool inSeq = false;
	static uint32_t lastUseTime = millis();
	
	if (UARTAvailable(VIM_UART))
	{
		lastUseTime = millis();
		
		int symb = ((int)(readUART(VIM_UART)) + 256) % 256;
		
		if (symb == 0xBB && !inSeq) // if start of message detected
		{
			inSeq = true;
			ptr = 0;
		}
		
		if (inSeq)
		{
			msg[ptr++] = symb;
			
			if (ptr == upperBound) // message completed
			{
				char crc = crc8(msg, MSG_SIZE - 1);
				if (crc == msg[MSG_SIZE - 1])
				{
					target.ballX = msg[1];
					target.ballY = msg[2]; 
					target.yGoalX = msg[3];
					target.yGoalY = msg[4];
                    target.bGoalX = msg[5];
                    target.bGoalY = msg[6];
				}
				else
				{
					GLOBAL_ERROR |= VIM_DATA_ERROR;
				}
				
				ptr = 0;
				inSeq = false;
			}
		}
	}
	
	if (millis() - lastUseTime > 100) 
	{
		GLOBAL_ERROR |= VIM_CONNECTION_ERROR;
	}
	
	if (ptr >= MSG_SIZE) 
	{
		inSeq = false;
		ptr = 0;
	}
	*/
}

#endif
