#include "stm32f10x.h"
#include "stdbool.h"
#include "math.h"
#include "connectionList.h"


/********************************* periph lib *********************************/

#include "stm32f103_sysFunc.h"
#include "stm32f103_pin.h"
#include "stm32f103_UART.h"
#include "stm32f103_SPI.h"
#include "stm32f103_adc_mod.h"

/********************************* flash ********************************/

#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)

void flash_unlock() 
{
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

void flash_lock()
{
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

uint8_t flash_ready() {
  return !(FLASH->SR & FLASH_SR_BSY);
}

void flash_erase_page(uint32_t address) {
  FLASH->CR|= FLASH_CR_PER;
  FLASH->AR = address;
  FLASH->CR|= FLASH_CR_STRT;
  while(!flash_ready());
  FLASH->CR &= ~FLASH_CR_PER;
}

void flash_write(uint32_t address, uint16_t data) {
  FLASH->CR |= FLASH_CR_PG;
  while(!flash_ready());
	
  *(__IO uint16_t*)address = data;
  while(!flash_ready());
	
  FLASH->CR &= ~(FLASH_CR_PG);
  while(!flash_ready());
}

uint32_t flash_read(uint32_t address) {
  return (*(__IO uint16_t*) address);
}

/********************************* code *********************************/
#define SENSORS_COUNT 32
#define AVERAGE_COUNT 5

#define HIGH_RANGE 4000
#define LOW_RANGE 20

#define forn(x) for(unsigned i = 0; i < x; i++)
#define microdelay() asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop") // 10 ticks delay (~1us)

#define max(a, b) (a > b ? a : b)

const uint8_t HALF_OF_SENSORS_COUNT = (SENSORS_COUNT >> 1);
unsigned data[32];

uint32_t calibMin[32];
uint32_t calibMax[32];

float value[32];

uint8_t sensor[2][16] = {
	{13, 0, 7, 6, 1, 2, 10, 11, 12, 15, 3, 4, 5, 8, 9, 14},
	{18, 31, 29, 30, 26, 25, 24, 23, 22, 16, 17, 27, 28, 21, 20, 19}
};


PL_ADC reader;

void initSensors()
{
	initPin(MULTIFLEXER_INPUT_1, OUTPUTPP);
	initPin(MULTIFLEXER_INPUT_2, OUTPUTPP);
	initPin(MULTIFLEXER_INPUT_3, OUTPUTPP);
	initPin(MULTIFLEXER_INPUT_4, OUTPUTPP);
	
	reader = *(new PL_ADC(ADC1));
	reader.init();
	reader.add(MULTIFLEXER_2_OUTPUT);
	reader.start();
}


void turnSensorsOn()
{
	setPin(SENSORS_POWER, 1);
}


void turnSensorsOff()
{
	setPin(SENSORS_POWER, 0);
}


void initPeriph()
{
	initPin(LED, OUTPUTPP); // small led on board
	setPin(LED, 1);
	
	initPin(SENSORS_POWER, OUTPUTPP); // small led on board
	turnSensorsOff();
	
	initSensors();
	
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		calibMax[i] = 0;
		calibMin[i] = 0;
		value[i] = 0.0;
		data[i]  = 0;
	}
	
	flash_unlock();
}


#define flash_addr 0x08018000

void saveCalibration()
{
	flash_erase_page(flash_addr);
	
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		flash_write(flash_addr + i * 2, uint16_t(calibMin[i]));
	}
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		flash_write(flash_addr + 64 + i * 2, uint16_t(calibMax[i]));
	}
}

bool calibrationIsCorrect()
{
	return calibMin[0] != 16383;
}

void loadCalibration()
{
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		calibMin[i] = flash_read(flash_addr + i * 2);
	}
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		calibMax[i] = flash_read(flash_addr + 64 + i * 2);
	}
}

void clearValues()
{
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		value[i] = 0.0;
	}
}

#define min(a, b) (a < b ? a : b)

void readSensors()
{
	for (uint8_t sensorIndex = 0; sensorIndex < HALF_OF_SENSORS_COUNT; sensorIndex++) 
	{		
		uint64_t s1 = 0, s2 = 0; // set zero to sensor values
		
		delayMicros(40);
		
		setPin(MULTIFLEXER_INPUT_1, sensorIndex & 1);
		setPin(MULTIFLEXER_INPUT_2, sensorIndex & 2);
		setPin(MULTIFLEXER_INPUT_3, sensorIndex & 4);
		setPin(MULTIFLEXER_INPUT_4, sensorIndex & 8);
		
		delayMicros(5);
		
		forn(AVERAGE_COUNT) {
			fastStart(ADC1);
			waitEndOfInjectedConversion(ADC1);
			microdelay();
			s1 += fastRead(ADC1, JDR1);
			s2 += fastRead(ADC1, DR);
		}
		
		data[sensor[0][sensorIndex]] = s1 / AVERAGE_COUNT;
		data[sensor[1][sensorIndex]] = s2 / AVERAGE_COUNT;
	}
	
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		value[i] = max(value[i], float(long(data[i]) - long(calibMin[i])) / float(calibMax[i] - calibMin[i]));
	}
}

void clearCalibration()
{
	for (int i = 0; i < 20; i++) 
	{
		delay(5);
		readSensors();
		delay(10);
	}
	
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		calibMax[i] = 0;
		calibMin[i] = 0;
	}
}

void oneShotCalibration()
{
	readSensors();
	for (int i = 0; i < SENSORS_COUNT; i++)
	{
		if (calibMin[i] == 0) calibMin[i] = data[i];
		if (calibMax[i] == 0) calibMax[i] = data[i];
			
		/*
		if ((data[i] < calibMin[i] * 1.2) ^ (data[i] > calibMax[i] * 0.8))
		{
			if (data[i] < (calibMin[i] * 1.2)) calibMin[i] = data[i] * 0.3 + calibMin[i] * 0.7;
			if (data[i] > (calibMax[i] * 0.8)) calibMax[i] = data[i] * 0.3 + calibMax[i] * 0.7;
		}
		*/
		
		if (data[i] > calibMax[i]) calibMax[i] = data[i];
		if (data[i] < calibMin[i]) calibMin[i] = data[i];
	}
	
	delay(2);
}


void calibrateSensors(uint32_t T)
{
	uint32_t t = millis();
	
	clearCalibration();
	
	do {
		oneShotCalibration();
	} while(millis() - t < T);
	
}
