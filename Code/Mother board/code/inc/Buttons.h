#ifndef BUTTONS_LIB
#define BUTTONS_LIB

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"


class Buttons
{
	public:
		void init(uint16_t b1, uint16_t b2, uint16_t b3, uint16_t b4);
		unsigned int update();
		void disable();
		void enable();
		bool isChanged(int n, bool preferred = false);
		bool isPressed(int n);
		void setChanged(int n, bool changed);
	
	private:
		struct ButtonStruct
		{
			bool disabled;
			bool pressed;
			long counter;
			bool changed;
			
			ButtonStruct()
			{
				pressed = 0;
				counter = 0;
				changed = false;
			}
			
			bool isChanged(bool preferred = false)
			{
				if (!preferred && disabled)
					return false;
				
				bool chn = changed;
				changed = false;
				
				return chn;
			}
			
			void setState(bool state)
			{
				if (pressed == 1 && state == 0) 
				{
					changed = true;
				}
				
				pressed = state;
			}
		};
		
	private:
		uint16_t b[4];
		ButtonStruct button[4];
	
};


void Buttons::init(uint16_t b1, uint16_t b2, uint16_t b3, uint16_t b4)
{
	b[0] = b1;
	b[1] = b2;
	b[2] = b3;
	b[3] = b4;
	
	for(int i = 0; i < 4; i++)
		initPin(b[i], INPUT, FL);
	
	enable();
	update();
}


unsigned int Buttons::update()
{
	for(int i = 0; i < 4; i++)
		button[i].setState(readPin(b[i]));
	return 0;
}


void Buttons::enable()
{
	for(int i = 0; i < 4; i++)
		button[i].disabled = false;
}


void Buttons::disable()
{
	for(int i = 0; i < 4; i++)
		button[i].disabled = false;
}


bool Buttons::isChanged(int n, bool preferred)
{
	return button[n].isChanged(preferred);
}


bool Buttons::isPressed(int n)
{
	return button[n].pressed;
}


void Buttons::setChanged(int n, bool changed)
{
	button[n].changed = changed;
}

#endif
