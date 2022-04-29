#ifndef BALL_SENSOR_LIB
#define BALL_SENSOR_LIB

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_adc.h"

#include "Errors.h"

#define TRESHOLD_VALUE		2000


class BallSensor
{
	public:	
		void init(PL_ADC *adc, uint16_t s, uint16_t en);
		unsigned int update();
		bool getValue();
		uint16_t getSensorValue();
		void turnOn();
		void turnOff();
	
	private:
		uint16_t s, en;
		PL_ADC *adc;
		bool value;
		volatile uint16_t valueOfSensor;
		bool working;
};


void BallSensor::init(PL_ADC *adc, uint16_t s, uint16_t en)
{
	this->adc = adc;
	this->s = s;
	adc->add(s);
	adc->start();
	
	initPin(en, OUTPUTPP);
	setPin(en, 1);
	turnOn();
	
	GLOBAL_ERROR |= update();
}


unsigned int BallSensor::update()
{
	if(!working) return BALL_SENSOR_POWER_OFF;
	valueOfSensor = adc->read(s);
	value = adc->read(s) > TRESHOLD_VALUE;
	return 0;
}


bool BallSensor::getValue()
{
	return value;
}


void BallSensor::turnOn()
{
	setPin(en, 1);
	working = 1;
}

void BallSensor::turnOff()
{
	setPin(en, 0);
	working = 0;
}

uint16_t BallSensor::getSensorValue()
{
	return valueOfSensor;
}

#endif
