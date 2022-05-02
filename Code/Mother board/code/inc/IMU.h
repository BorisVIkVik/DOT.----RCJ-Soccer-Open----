#ifndef IMU_LIB
#define IMU_LIB

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"
#include "mpu9250.h"
#include "tools.h"

#include "Errors.h"


class IMU
{
	public:
		void init(unsigned int spi, uint16_t ss, uint16_t en);
		void turnOn();
		void turnOff();
		void setZeroAngle();
		void calibrate(uint32_t t);
		unsigned int update();
		void updateAnglesFromFIFO();
		double getAngle();
		double imuFloatValue, angleChange;
		long long int imuFloatTime;
	
	private:
		mpu9250 mpuSensor;
		unsigned int spi;
		uint16_t en, ss;
		volatile double angle, zeroAngle;
		bool working;
};


void IMU::init(unsigned int spi, uint16_t ss, uint16_t en)
{
	this->spi = spi;
	this->ss = ss;
	this->en = en;
	
	turnOn();
	delay(100);
	mpuSensor.initIMU(spi, ss);
	delay(100);
	
	setZeroAngle(); // IMU calibrated angle estimating
	
	imuFloatTime = millis();
	imuFloatValue = 0;
}


void IMU::turnOn()
{
	initPin(en, OUTPUTPP);
	setPin(en, 1);
	working = 1;
}


void IMU::turnOff()
{
	initPin(en, OUTPUTPP);
	setPin(en, 0);
	working = 0;
}


void IMU::setZeroAngle()
{
	update();
	zeroAngle = angle;
}


unsigned int IMU::update()
{
	if(!working) return IMU_POWER_OFF;
	
	updateAnglesFromFIFO();
	double neangle = mpuSensor.yaw;
	angle = neangle;
	//adduction(angle);
	return 0;
}


void IMU::updateAnglesFromFIFO()
{
	mpuSensor.updateAnglesFromFIFO();
}


void IMU::calibrate(uint32_t t)
{
	mpuSensor.calibrate(t);
}


double IMU::getAngle()
{
	double a = angle-zeroAngle;
	adduction(a)
	return a;
}

#endif
