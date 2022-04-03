#include "main.h"

//#define ERROR(msg) writeStrUART(_UART1, "\r\n\r\nERROR:"); writeStrUART(msg); writeStrUART("\r\n\r\n")

#define LOG_ENABLE 1

#define SENSORS_COUNT 32
#define AVERAGE_COUNT 1

#define HIGH_RANGE 4000
#define LOW_RANGE 20

#if LOG_ENABLE
long long OUT_OF_RANGE[SENSORS_COUNT];
#endif

#define forn(x) for(unsigned i = 0; i < x; i++)
#define flexUpdateDelay() asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop"); // 10 ticks delay (~1us)

const uint8_t HALF_OF_SENSORS_COUNT = (SENSORS_COUNT >> 1);
unsigned data[32];
uint8_t sensor[2][16] = {
{15,13,14,10,9,8,7,6,0,1,11,12,5,4,3,2},
{31,29,30,26,25,24,23,22,16,17,27,28,21,20,19,18}};

unsigned long long s1, s2, cycle = 0;

int main() {
	sysStart();
	
	initPin(B, 12, OUTPUTPP);			//slave select
	initSPI(_SPI2, MASTER, 16, 128);
	
	uint16_t resData;	
	resData = 4352 + 90; // 0001 0001 + resistor value MUST BE MORE THAN 50!!! INACHE PZDA
		setPin(B, 12, 0);
		delay(1);
		writeSPI(_SPI2, resData);
		delay(1);
		setPin(B, 12, 1);
	
	/*for(int i = 255; i > 90; i--){
		resData = 4352 + i; // 0001 0001 + resistor value
		setPin(B, 12, 0);
		delay(1);
		writeSPI(_SPI2, resData);
		delay(1);
		setPin(B, 12, 1);
		delay(10);
	}
	for(int i = 90; i < 255; i++){
		resData = 4352 + i; // 0001 0001 + resistor value
		setPin(B, 12, 0);
		delay(1);
		writeSPI(_SPI2, resData);
		delay(1);
		setPin(B, 12, 1);
		delay(10);
	}*/
	
	initPin(B, 1, 	OUTPUTPP);
	initPin(B, 2, 	OUTPUTPP);
	initPin(B, 10, 	OUTPUTPP);
	initPin(B, 11, 	OUTPUTPP);
	initPin(B, 0, 	OUTPUTPP);
	
	initUART(_UART1, 115200);
	writeStrUART(_UART1, "\r\nStart\r\n");
	
	PL_ADC ADC_1 = *(new PL_ADC(ADC1));
	ADC_1.init();
	ADC_1.add(PA0);
	ADC_1.add(PA1);
	ADC_1.start();
	
	forn(SENSORS_COUNT) {
		data[i] = 0;
		#if LOG_ENABLE
		OUT_OF_RANGE[i] = 0;
		#endif
	}
	
	while (true) {
		for (uint8_t sensorIndex = 0; sensorIndex < HALF_OF_SENSORS_COUNT; sensorIndex++) {
			s1 = 0, s2 = 0; // set zero to sensor values
			
			setPin(B, 10, (sensorIndex >> 0) % 2);
			setPin(B, 2, 	(sensorIndex >> 1) % 2);
			setPin(B, 1, 	(sensorIndex >> 2) % 2);
			setPin(B, 11, (sensorIndex >> 3) % 2);
			
			//flexUpdateDelay();
			delayMicros(35);
			
			forn(AVERAGE_COUNT) {
				fastStart(ADC1);
				waitEndOfInjectedConversion(ADC1);
				asm("nop");asm("nop");
				//s1 += fastCheatRead(ADC1);
				s1 += fastRead(ADC1, JDR1);
				s2 += fastRead(ADC1, JDR2);
			}
			
			data[sensor[0][sensorIndex]] = s1 / AVERAGE_COUNT;
			data[sensor[1][sensorIndex]] = s2 / AVERAGE_COUNT;
		}
		
		#if LOG_ENABLE
		forn(HALF_OF_SENSORS_COUNT) {
			if (data[i] < LOW_RANGE || data[i] > HIGH_RANGE) {
				OUT_OF_RANGE[i] += 1;
			}
			else if (OUT_OF_RANGE[i] > 0) OUT_OF_RANGE[i] -= 2;
		}
		#endif
		
		cycle++;
		
		if (cycle % 1 == 0) {
			forn(SENSORS_COUNT) {
				printUART(_UART1, data[i]);
				writeStrUART(_UART1, " ");
			}
			
			writeStrUART(_UART1, "\r\n");
			cycle = 0;
			invertPin(B, 0);
		}
	}
}
