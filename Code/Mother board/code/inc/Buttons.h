#ifndef BUTTONS_LIB
#define BUTTONS_LIB

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"

#define ENTER_BUTTON 0
#define DOWN_BUTTON 1
#define UP_BUTTON 2
#define ESC_BUTTON 3


class Buttons
{
	public:
		void init(uint16_t enterButton, uint16_t downButton, uint16_t upButton, uint16_t escButton);
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


void Buttons::init(uint16_t enterButton, uint16_t downButton, uint16_t upButton, uint16_t escButton)
{
	b[ENTER_BUTTON] = enterButton;
	b[DOWN_BUTTON] = downButton;
	b[UP_BUTTON] = upButton;
	b[ESC_BUTTON] = escButton;
	
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
