#ifndef CONNECTION_LIST
#define CONNECTION_LIST


///////////////////////////////		DEBUG UART (BLUETOOTH)		//////////////
#define DEBUG_UART	_UART3


///////////////////////////////		BUTTONS		//////////////////////////////
#define BUTTON_1	PE5
#define BUTTON_2	PE4
#define BUTTON_3	PE3


///////////////////////////////		LEDS		////////////////////////////////
#define LED_1	PE15
#define LED_2	PD0
#define LED_3	PC11

#define LED_ERROR	LED_1


///////////////////////////////		LINE SENSORS		////////////////////////
#define LINE_SENSORS_SPI	_SPI1
#define LINE_SENSORS_SS	PE10


///////////////////////////////		KICKER MODULE		////////////////////////
#define KICKER_MODULE_SPI	_SPI1
#define KICKER_MODULE_SS	PE11


///////////////////////////////		DISPLAY		//////////////////////////////
#define DISPLAY_SPI	_SPI3
#define DISPLAY_CS	PD4
#define DISPLAY_SCK	PC10
#define DISPLAY_MOSI	PB5
#define DISPLAY_DC	PD5
#define DISPLAY_RESET	PD6
#define DISPLAY_PWR_EN	PB6


///////////////////////////////		MOTOR DRIVERS		////////////////////////
#define MOTOR_1_UART	_UART4
#define MOTOR_2_UART	_UART5
#define MOTOR_3_UART	_UART1
#define MOTOR_4_UART	_UART6

#define MOTOR_1	0x01	//DRIVERS ADDRESSES
#define MOTOR_2	0x03 // 1-3 swap
#define MOTOR_3	0x02
#define MOTOR_4	0x04


///////////////////////////////		IMU		//////////////////////////////////
#define IMU_SPI	_SPI2
#define IMU_SS	PD8
#define IMU_PWR_EN	PB12
#define IMU_INT	PD12


///////////////////////////////		VIM		//////////////////////////////////
#define VIM_UART	_UART2
#define CALIBRATION_START 0x01
#define CALIBRATION_END   0x02


#define IMU_SPI	_SPI2				// LINE SENSORS
#define IMU_SPI_SS	PD8


////////////////////////////		DRIBLERS		///////////////////////////////
#define DRIBLER_1_TIM	_TIM1
#define DRIBLER_2_TIM	_TIM1
#define DRIBLER_1_CH	_CH3
#define DRIBLER_2_CH	_CH4
#define DRIBLER_1_PIN	PE13
#define DRIBLER_2_PIN	PE14


/////////////////////////////		BATTERY		/////////////////////////////////
#define BATTERY_S1_EN	PA11
#define BATTERY_S2_EN	PA8
#define BATTERY_S3_EN	PC9
#define BATTERY_S4_EN	PA12

#define BATTERY_S1	PC2
#define BATTERY_S2	PC1
#define BATTERY_S3	PC0
#define BATTERY_S4	PC3


//////////////////////////		BALL SENSORS		/////////////////////////////
#define BALL_SENSOR_1_EN	PA15
#define BALL_SENSOR_2_EN	PE8
#define BALL_SENSOR_1			PB1
#define BALL_SENSOR_2			PB0

#endif





#ifndef CONNECTION_LIST
#define CONNECTION_LIST


///////////////////////////////		DEBUG UART (BLUETOOTH)		//////////////
#define DEBUG_UART	_UART3
//Maybe need change. To software UART. Change to BlueTooth UART

//////////////////////////		Bluetooth		/////////////////////////////
#define	BLUETOOTH_EN		PA4
#define BLUETOOTH_KEY		PC5



///////////////////////////////		BUTTONS		//////////////////////////////
#define BUTTON_1	PD9
#define BUTTON_2	PD10
#define BUTTON_3	PD11
#define BUTTON_4 	PD13


///////////////////////////////		LEDS		////////////////////////////////
#define LED_1	PE3
#define LED_2	PE9
#define LED_3	PC11

#define LED_ERROR	LED_1


///////////////////////////////		LINE SENSORS		////////////////////////
#define LINE_SENSORS_SPI	_SPI1
#define LINE_SENSORS_SS	PE11


///////////////////////////////		KICKER MODULE		////////////////////////
#define KICKER_MODULE_SPI	_SPI1
#define KICKER_MODULE_SS	PE11

//
#define KICKER_BOOSTER_EN			PE12
#define KICKER_BOOSTER_DONE		PE13
#define KICKER_1	PA8
#define KICKER_2	PC9
//

///////////////////////////////		DISPLAY		//////////////////////////////
#define DISPLAY_SPI	_SPI3
#define DISPLAY_CS	PD4
#define DISPLAY_SCK	PC10
#define DISPLAY_MOSI	PB5
#define DISPLAY_DC	PD5
#define DISPLAY_RESET	PD6
#define DISPLAY_PWR_EN	PB6


///////////////////////////////		MOTOR DRIVERS		////////////////////////
#define MOTOR_1_UART	_UART5
#define MOTOR_2_UART	_UART6
#define MOTOR_3_UART	_UART3
#define MOTOR_4_UART	_UART4
#define MOTOR_DRIBBLER_UART	_UART1

#define MOTOR_1	0x01	//DRIVERS ADDRESSES
#define MOTOR_2	0x03 // 1-3 swap
#define MOTOR_3	0x02
#define MOTOR_4	0x04
#define MOTOR_DRIBBLER	0x07


///////////////////////////////		IMU		//////////////////////////////////
#define IMU_SPI	_SPI2
#define IMU_SS	PD8
#define IMU_PWR_EN	PB12
#define IMU_INT	PD12


///////////////////////////////		VIM		//////////////////////////////////DELETE THIS SHIT! 
#define VIM_UART	_UART2
#define CALIBRATION_START 0x01
#define CALIBRATION_END   0x02



//Cringe
//#define IMU_SPI	_SPI2				// LINE SENSORS
#define IMU_SPI_SS	PD8


////////////////////////////		DRIBLERS		///////////////////////////////DELETE THIS
#define DRIBLER_1_TIM	_TIM1
#define DRIBLER_2_TIM	_TIM1
#define DRIBLER_1_CH	_CH3
#define DRIBLER_2_CH	_CH4
#define DRIBLER_1_PIN	PE13
#define DRIBLER_2_PIN	PE14


/////////////////////////////		BATTERY		/////////////////////////////////DELETE ENABLES
#define BATTERY_S1_EN	PA11
#define BATTERY_S2_EN	PA8
#define BATTERY_S3_EN	PC9
#define BATTERY_S4_EN	PA12

#define BATTERY_S1	PC1
#define BATTERY_S2	PC2
#define BATTERY_S3	PC3
#define BATTERY_S4	PC0


//////////////////////////		BALL SENSORS		/////////////////////////////DELETE THIS. CHANge to only one
#define BALL_SENSOR_1_EN	PA15
#define BALL_SENSOR_2_EN	PE8
#define BALL_SENSOR_1			PB1
#define BALL_SENSOR_2			PB0

//This one
#define BALL_SENSOR_EN		PB9
#define BALL_SENSOR				PB1




//////////////////////////		POWER CONTROL		/////////////////////////////
#define REG_5V_EN PE14



#define POWER_SWITCH PE1














#endif
