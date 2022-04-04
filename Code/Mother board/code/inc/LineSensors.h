#ifndef LINE_SENSORS
#define LINE_SENSORS

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"
#include "tools.h"

#include "Errors.h"


#define CALIBRATION_START 0x01
#define CALIBRATION_END   0x02
#define SENSORS_ON 0x03
#define SENSORS_OFF   0x04

class LineSensors
{
	public:
		void init(unsigned int spi, uint16_t ss);
		unsigned int update();
		void beginLineCalibration();
		void endLineCalibration();
		void turnOnSensors();
		void turnOffSensors();
		uint8_t getLine();
	
	
	private:
		unsigned int spi;
		uint16_t ss;
		uint8_t lineBoardTx;	
		uint8_t line;
};


void LineSensors::init(unsigned int spi, uint16_t ss)
{
	this->spi = spi;
	this->ss = ss;
		
	initPin(ss, OUTPUTPP);
	setPin(ss, 1);
	
	lineBoardTx = CALIBRATION_END;
}


unsigned int LineSensors::update()
{
	setPin(ss, 0);
	uint8_t res = writeSPI(spi, lineBoardTx);
	setPin(ss, 1);
	
	if (res != 255 && res != 0)
	{
		line = res - 1;
		return 0;
	}
	else
	{
		line = 0;
		return LINE_BOARD_CONNECTION_ERROR;
	}
}


void LineSensors::beginLineCalibration()
{
	lineBoardTx = CALIBRATION_START;
}


void LineSensors::endLineCalibration()
{
	lineBoardTx = CALIBRATION_END;
}


void LineSensors::turnOnSensors()
{
	lineBoardTx = SENSORS_ON;
}


void LineSensors::turnOffSensors()
{
	lineBoardTx = SENSORS_OFF;
}


uint8_t LineSensors::getLine()
{
	return line;
}

#endif
