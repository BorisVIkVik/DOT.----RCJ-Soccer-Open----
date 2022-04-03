#include "stm32f4xx.h"
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "connectionList.h"

/********************************* periph lib *********************************/

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_UART.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_adc.h"
#include "stm32f407_SPI.h"
#include "stm32f407_PWM.h"

#include "SSD1306.h"
#include "mpu9250_registers.h"
#include "mpu9250.h"

#include "font.h"

/******************************** special libs ********************************/

uint32_t GLOBAL_ERROR = 0;

#define DRIVER_CONNECTION_ERROR 1
#define DRIVER_DATA_ERROR 2
#define DRIVER_FATAL_ERROR 4
#define IMU_DATA_ERROR 8
#define IMU_CONNECTION_ERROR 16
#define LINE_BOARD_CONNECTION_ERROR 32
#define VIM_CONNECTION_ERROR 64
#define VIM_DATA_ERROR 128
#define LOW_BATTERY_POWER 256


#include "tools.h"
#include "drivers.h"
#include "functional.h"
#include <FunctionalClass.h>

#define EMPTY_SCREEN 0
#define MENU_SCREEN 1
#define CALIBRATIONS_SCREEN 2
#define LIGHT_SENSORS_CALIBRATION_SCREEN 3
#define BATTERY_SCREEN 4
#define KOKOKO_SCREEN 5
#define GAME_SCREEN 6
#define DRIBBLER_SCREEN 7
#define KICKER_SCREEN 8
#define DEBUG_DATA_SCREEN 9

int g_state = MENU_SCREEN;
