#ifndef CONNECTION_LIST
#define CONNECTION_LIST


///////////////////////////////		DEBUG UART (BLUETOOTH)		//////////////
#define DEBUG_UART	BLUETOOTH_UART


//////////////////////////		BLUETOOTH		/////////////////////////////
#define BLUETOOTH_UART	_SOFTWARE_UART
#define BLUETOOTH_UART_RX	PE7
#define BLUETOOTH_UART_TX	PE8
#define	BLUETOOTH_EN		PA4
#define BLUETOOTH_KEY		PC5


///////////////////////////////		BUTTONS		//////////////////////////////
#define BUTTON_1_ENTER	PD9
#define BUTTON_2_DOWN	PD11
#define BUTTON_3_UP	PD10
#define BUTTON_4_ESC 	PD13


///////////////////////////////		LEDS		////////////////////////////////
#define LED_1	PE3
#define LED_2	PE9
#define LED_3	PC11

#define ERROR_LED	LED_1


///////////////////////////////		LIDAR		////////////////////////////////
#define LIDAR_UART _UART2


//////////////////////////		PERIPHERIAL SPI		//////////////////////////
#define PERIPH_SPI	_SPI1

///////////////////////////////		LINE SENSORS		////////////////////////
#define LINE_SENSORS_SPI	PERIPH_SPI
#define LINE_SENSORS_SS	PE11

///////////////////////////////		CAMERA		//////////////////////////////
#define CAMERA_SPI	PERIPH_SPI
#define CAMERA_SS	PE10


///////////////////////////////		KICKER MODULE		////////////////////////
#define KICKER_BOOSTER_EN			PE12
#define KICKER_BOOSTER_DONE		PE13
#define KICKER_1	PA8
#define KICKER_2	PC9


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
#define MOTOR_2	0x03
#define MOTOR_3	0x02
#define MOTOR_4	0x04
#define MOTOR_DRIBBLER	0x08


///////////////////////////////		IMU		//////////////////////////////////
#define IMU_SPI	_SPI2
#define IMU_SPI_SS	PD8
#define IMU_PWR_EN	PB12
#define IMU_INT	PD12


/////////////////////////////		BATTERY		/////////////////////////////////
#define BATTERY_S1	PC1
#define BATTERY_S2	PC2
#define BATTERY_S3	PC3
#define BATTERY_S4	PC0


//////////////////////////		BALL SENSORS		/////////////////////////////
#define BALL_SENSOR_EN		PB9
#define BALL_SENSOR				PB1


//////////////////////////		POWER CONTROL		/////////////////////////////
#define REG_5V_EN PE14
#define POWER_SWITCH PE1


#endif
