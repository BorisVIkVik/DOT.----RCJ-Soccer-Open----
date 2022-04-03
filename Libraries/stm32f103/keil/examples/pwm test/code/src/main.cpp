#include "main.h"


int main(){
	
	sysStart();
	initUART(_UART1, 115200);
	
	//servo
	//initServoPWM(_TIM2, 4, A, 3, 1, 72);			//for using servo freq must be 200 Hz
	//standart PWM
	initPWM(_TIM2, 2, A, 1, 100, 72);						
	
	delay(100);
	writeStrUART(_UART1, "\r\nStart\r\n");

	
	while(1){
		
		
		//servo
		/*for(int i = 0; i < 180; i++){
			writeStrUART(_UART1, "pwm: ");
			printUART(_UART1, i);
			writeStrUART(_UART1, "\r\n");
		  setServoPWM(_TIM2, 4, i);
			delay(20);
		}
		
		for(int i = 180; i > 0; i--){
			writeStrUART(_UART1, "pwm: ");
			printUART(_UART1, i);
			writeStrUART(_UART1, "\r\n");
		  setServoPWM(_TIM2, 4, i);
			delay(20);
		}
		
		while(1){
			setServoPWM(_TIM2, 4, 0);
			delay(2000);
			setServoPWM(_TIM2, 4, 90);
			delay(2000);
			setServoPWM(_TIM2, 4, 180);
			delay(2000);
		}
		*/
	}
	
}
