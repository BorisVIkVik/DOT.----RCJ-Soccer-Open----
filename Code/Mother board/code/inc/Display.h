#ifndef DISPLAY_LIB
#define DISPLAY_LIB

#include "stm32f407_sysFunc.h"
#include "stm32f407_SPI.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "SSD1306.h"
#include "tools.h"
#include "font.h"

#define LEN 4


void defaultCallback()
{
}

class Command
{
	public:
		char* name;
		void (*callback)();
		
		Command()
		{
			char* name = "";
			callback = 0;
		}
		
		Command(char* name, void (*callback)()=defaultCallback)
		{
			this->name = name;
			this->callback = callback;
		}
};



class Screen
{
	public:
		Command lines[10];
		Command esc;
		bool haveEsc;
		int size;
		int scroll;
		int choosenOne;
	
		Screen()
		{
			haveEsc = 0;
			choosenOne = 0;
			size = 0;
			scroll = 0;
		}
		
		void add(Command cmd)
		{
			if (size == 20)
				return;
			
			lines[size] = cmd;
			size++;
		}
		
		void addEsc(Command cmd)
		{
			esc = cmd;
			haveEsc |= 1;
		}
		
		void scrollDown()
		{
			choosenOne--;
			
			if (choosenOne < 0)
			{
				choosenOne++;
				scroll--;
			}
			
			if (scroll < 0)
				scroll = 0;
		}
		
		void scrollUp()
		{
			int savedChoosen = choosenOne;
			int savedScroll = scroll;
			
			choosenOne++;
			
			if (choosenOne >= LEN)
			{
				choosenOne = savedChoosen;
				scroll++;
			}
			
			if (choosenOne >= size)
			{
				choosenOne = savedChoosen;
				scroll = savedScroll;
			}
			
			if (scroll > size - LEN)
				scroll = savedScroll;
		}
		
		void escape()
		{
			if(haveEsc)
				esc.callback();
		}
		
		void execute()
		{
			lines[scroll + choosenOne].callback();
		}
};

class Display
{
	public:
		SSD1306 ssd1306;
		Screen screens[10];
		int screenIndex;
		
		Display()
		{
			screenIndex = 0;
		}
	
		void init(SSD1306 *disp)
		{
			this->ssd1306 = *disp;
		}
		
		void print(char* x, int row = 0, int column = 0);
		void print(int x, int row = 0, int column = 0, int next = 3);
		void print(unsigned x, int row = 0, int column = 0, int next = 3);
		void print(long long x, int row = 0, int column = 0, int next = 3);
		void print(unsigned long long x, int row = 0, int column = 0, int next = 3);
		void print(double x, int row = 0, int column = 0, int next = 3);
		void print(float x, int row = 0, int column = 0, int next = 3);
		
		void setScreen(int index)
		{
			screenIndex = index;
		}
		
		void addEsc(Command cmd)
		{
			screens[screenIndex].addEsc(cmd);
		}
		
		void addToScreen(Command cmd)
		{
			screens[screenIndex].add(cmd);
		}
		
		void scrollDown()
		{
			screens[screenIndex].scrollDown();
		}
		
		void scrollUp()
		{
			screens[screenIndex].scrollUp();
		}
		
		void escape()
		{
			screens[screenIndex].escape();
		}
		
		void update()
		{
			int size = min2(LEN, screens[screenIndex].size);
			for (int i = 0; i < size; i++)
			{
				char* text = screens[screenIndex].lines[i + screens[screenIndex].scroll].name;
				drawString(ssd1306, text, i, 1);
			}
			
			drawString(ssd1306, "+", screens[screenIndex].choosenOne, 0);
		}
		
		void execute()
		{
			screens[screenIndex].execute();
		}
		
		void clear()
		{
			ssd1306.clear();
		}
		
		void show()
		{
			ssd1306.display();
		}
};

void Display::print(char* x, int row, int column)
{		
	drawString(ssd1306, x, row, column);
}

void Display::print(int x, int row, int column, int next)
{
	printTml(ssd1306, x, next, row, column);
}

void Display::print(unsigned x, int row, int column, int next)
{
	printTml(ssd1306, x, next, row, column);
}

void Display::print(long long  x, int row, int column, int next)
{
	printTml(ssd1306, x, next, row, column);
}

void Display::print(unsigned long long  x, int row, int column, int next)
{
	printTml(ssd1306, x, next, row, column);
}

void Display::print(double x, int row, int column, int next)
{
	printTml(ssd1306, x, next, row, column);
}

void Display::print(float x, int row, int column, int next)
{
	printTml(ssd1306, x, next, row, column);
}

#endif
