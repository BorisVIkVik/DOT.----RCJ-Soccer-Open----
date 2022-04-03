#include "main.h"


#define PIN	PE13
#define TIMER	_TIM1
#define CHANNEL	_CH3

int main(){
	
	sysStart();
	
	delay(500);
	
	//initPWM(TIMER, CHANNEL, PIN, 1000);					//1kHz
	//setPWM(TIMER, CHANNEL, 90); 		 							//50 is middle position (0;100)
	
	initPWM(TIMER, CHANNEL, PIN, SERVO_FREQ);
	setPWM(TIMER, CHANNEL, 180);
	delay(250);
	setPWM(TIMER, CHANNEL, 0);
	delay(250);
	setPWM(TIMER, CHANNEL, 90);  							//90 is middle position (0;180)
	delay(1000);

	setPWM(TIMER, CHANNEL, 95);
	while(1) 
	{
		
	}
}
