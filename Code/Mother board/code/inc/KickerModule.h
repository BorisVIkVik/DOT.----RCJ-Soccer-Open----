#ifndef KICKER_MODULE
#define KICKER_MODULE

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"

#define KICK_TIME 2
#define COOL_DOWN_TIME 1000

class KickerModule
{
	public:
		void init(uint16_t boosterEn, uint16_t boosterDone, uint16_t kicker1, uint16_t kicker2);
		unsigned int update();
		void initCharge();
		bool isCharged();
		void kick(bool kick1, bool kick2);

	private:
		bool charged;
		uint16_t boosterEn, boosterDone, kicker1, kicker2;
		long long int kicker1Timer, kicker2Timer;
};


void KickerModule::init(uint16_t boosterEn, uint16_t boosterDone, uint16_t kicker1, uint16_t kicker2)
{
	this->boosterEn = boosterEn;
	this->boosterDone = boosterDone;
	this->kicker1 = kicker1;
	this->kicker2 = kicker2;
	
	initPin(boosterEn, OUTPUTPP);
	setPin(boosterEn, 0);
	initPin(boosterDone, INPUT, FL);
	
	charged = 0;
	
	initPin(kicker1, OUTPUTPP);
	initPin(kicker2, OUTPUTPP);
	setPin(kicker1, 0);
	setPin(kicker2, 0);
	
	kicker1Timer = millis();
	kicker2Timer = millis();
}


unsigned int KickerModule::update()
{
	if(readPin(boosterDone) == 0)
	{
		//setPin(boosterEn, 0);
		charged = 1;
	}
}


void KickerModule::initCharge()
{
	setPin(boosterEn, 1);
}


bool KickerModule::isCharged()
{
	if(charged == 1)
		return 1;
	else
		return 0;
}


void KickerModule::kick(bool kick1, bool kick2)
{
	setPin(boosterEn, 0);
	
	if (kick1 && millis() - kicker1Timer > COOL_DOWN_TIME)
		kicker1Timer = millis();
	else
		kick1 = 0;
	
	if (kick2 && millis() - kicker2Timer > COOL_DOWN_TIME)
		kicker2Timer = millis();
	else
		kick2 = 0;
	
	if(kick1 || kick2)
	{
		setPin(kicker1, kick1);
		setPin(kicker2, kick2);
		delay(KICK_TIME);
	}
	
	setPin(kicker1, 0);
	setPin(kicker2, 0);
	
	charged = 0;
}

#endif
