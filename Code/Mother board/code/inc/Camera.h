#ifndef CAMERA
#define CAMERA

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"

#include "Errors.h"


class Camera
{
	public:
		void init(unsigned int spi, uint16_t ss);
		unsigned int update();
	
	
	private:
		unsigned int spi;
		uint16_t ss;	
};


void Camera::init(unsigned int spi, uint16_t ss)
{
	this->spi = spi;
	this->ss = ss;
		
	initPin(ss, OUTPUTPP);
	setPin(ss, 1);
}


unsigned int Camera::update()
{
	
	
	
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
	return 0;
}

#endif
