#ifndef BTTERY
#define BTTERY

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_adc.h"

#include "Errors.h"

#define MAX_BATTERY_VOLTAGE		16.8
#define LOW_CELL_VOLTAGE		3.3
#define LOW_BATTERY_VOLTAGE		LOW_CELL_VOLTAGE*4
#define MAX_LOW_VOLTAGE_TIME	1000



class Battery
{
	public:	
		void init(PL_ADC *adc, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4);
		unsigned int update();
		double getPercentage();
		double getVoltage();
		double getCellVoltage(int n);	
	
	private:
		uint16_t s[4];
		PL_ADC *adc;
		double cellCoef[4];
		double cellReading[4];
		double cellVoltage[4];
		double voltage;
		long long int lowBatteryVoltageTimer;
		bool lowVoltageDetected;
		double percentage;
		
		void readCell(int n);
};


void Battery::init(PL_ADC *adc, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4)
{
	cellCoef[0] = 0.00117;		//* 3.3 * double(1200 + 3300) / double(4096 * 3300);
	cellCoef[1] = 0.002177;		//* 3.3 * double(5600 + 3300) / double(4096 * 3300);
	cellCoef[2] = 0.00331;		//* 3.3 * double(9100 + 2700) / double(4096 * 2700);
	cellCoef[3] = 0.0047;		//* 3.3 * double(13000 + 2700) / double(4096 * 2700);
	
	s[0] = s1;
	s[1] = s2;
	s[2] = s3;
	s[3] = s4;
	
	this->adc = adc;
	adc->add(s[0]);
	adc->add(s[1]);
	adc->add(s[2]);
	adc->add(s[3]);
	adc->start();
	
	lowVoltageDetected = 0;
	
	GLOBAL_ERROR |= update();
}


void Battery::readCell(int n)
{
	cellReading[n] = double(adc->read(s[n])) * cellCoef[n];
}


unsigned int Battery::update()
{
	voltage = 0;
	bool lowCellVoltage = 0;
	
	for(int i = 0; i < 4; i++){
		delayMicros(500);
		readCell(i);
	}
	
	for(int i = 0; i < 4; i++){
		cellVoltage[i] = cellReading[i];
		if(i > 0) cellVoltage[i] -= cellReading[i-1];
		
		if(cellVoltage[i] < LOW_CELL_VOLTAGE) lowCellVoltage = 1;
	}

	
	voltage = cellReading[3];
	
	percentage = (voltage - LOW_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - LOW_BATTERY_VOLTAGE) * 100;
	if(percentage > 100.0) percentage = 100.0;
	if(percentage < 0.0) percentage = 0.0;
	
	if(voltage < LOW_BATTERY_VOLTAGE || lowCellVoltage)
	{
		if(!lowVoltageDetected)
		{
			lowVoltageDetected = 1;
			lowBatteryVoltageTimer = millis();
		}
		
		if(millis() - lowBatteryVoltageTimer > MAX_LOW_VOLTAGE_TIME) return LOW_BATTERY_POWER;
		else return 0;
	}
	else
	{
		lowVoltageDetected = 0;
		return 0;
	}
}


double Battery::getPercentage()
{
	return percentage;
}


double Battery::getVoltage()
{
	return voltage;
}


double Battery::getCellVoltage(int n)
{
	return cellVoltage[n];
}

#endif
