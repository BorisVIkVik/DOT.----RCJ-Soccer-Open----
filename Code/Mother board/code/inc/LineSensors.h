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

class LineSensors
{
	public:
		void init(unsigned int spi, uint16_t ss);
		unsigned int update();
		void beginLineCalibration();
		void endLineCalibration();
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
	
	if (res != 0)
		{
			line = max2(uint8_t(res - 1), line);
			return 0;
		}
	else
		return LINE_BOARD_CONNECTION_ERROR;
}


void LineSensors::beginLineCalibration()
{
	lineBoardTx = CALIBRATION_START;
}


void LineSensors::endLineCalibration()
{
	lineBoardTx = CALIBRATION_END;
}


uint8_t LineSensors::getLine()
{
	return line;
}

#endif
