#ifndef KICKER_MODULE
#define KICKER_MODULE

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"

#define KICK_TIME 10
#define COOL_DOWN_TIME 1000

class KickerModule
{
	public:
		void init(uint16_t boosterEn, uint16_t boosterDone, uint16_t kicker1, uint16_t kicker2);
		void switchOnBooster();
		void switchOffBooster();
		void kick(bool kick1, bool kick2);

	private:
		uint16_t boosterEn, boosterDone, kicker1, kicker2;
		long long int kicker1Timer, kicker2Timer;
};


void KickerModule::init(uint16_t boosterEn, uint16_t boosterDone, uint16_t kicker1, uint16_t kicker2)
{
	initPin(boosterEn, OUTPUTPP);
	setPin(boosterEn, 0);
	initPin(boosterDone, INPUT, FL);
	
	initPin(kicker1, OUTPUTPP);
	initPin(kicker2, OUTPUTPP);
	setPin(kicker1, 0);
	setPin(kicker2, 0);
	
	kicker1Timer = millis();
	kicker2Timer = millis();
}


void KickerModule::switchOnBooster()
{
	setPin(boosterEn, 1);
}


void KickerModule::switchOffBooster()
{
	setPin(boosterEn, 0);
}


void KickerModule::kick(bool kick1, bool kick2)
{
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
}

#endif
