#pragma once

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

#include "font.h"

/******************************** special libs ********************************/


#include "tools.h"
#include "Robot.h"
#include <FunctionalClass.h>
#include <FieldObject.h>
#include <CameraObject.h>
#include <Strategy.h>

#define EMPTY_SCREEN 0
#define MENU_SCREEN 1
#define CALIBRATIONS_SCREEN 2
#define LIGHT_SENSORS_CALIBRATION_SCREEN 3
#define BATTERY_SCREEN 4
#define GAME_SCREEN 6
#define DRIBBLER_SCREEN 7
#define KICKER_SCREEN 8
#define DEBUG_DATA_SCREEN 9

int g_state = MENU_SCREEN;
