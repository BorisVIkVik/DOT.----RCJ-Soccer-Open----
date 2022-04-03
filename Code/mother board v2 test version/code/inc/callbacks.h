#ifndef __MENU_CALLBACKS__
#define __MENU_CALLBACKS__


#define MANUAL_DRIBBLER_CONTROL_SPEED 22

int clb_var = 0;
unsigned long long pncBeginTime = 0;

void clb_doNothing()
{
	clb_var++;
}




//To empty screen
void clb_screenToEmpty()
{
	robot.display.setScreen(EMPTY_SCREEN);
}

//To menu screen
void clb_screenToMain()
{
	g_state = MENU_SCREEN;
	robot.display.setScreen(MENU_SCREEN);
}




//Game screen
void clb_screenToGame()
{
	g_state = GAME_SCREEN;
	robot.display.setScreen(GAME_SCREEN);
	robot.button[0].changed = false;
	robot.button[1].changed = false;
	robot.button[2].changed = false;
	robot.setupIMU();
}

//Calibrations screen
void clb_screenToCalibrations()
{
	robot.display.setScreen(CALIBRATIONS_SCREEN);
	g_state = CALIBRATIONS_SCREEN;
}

		void clb_imuCalib()
		{
			robot.imu.calibrate(20000);
		}

		void clb_imuSetup()
		{
			robot.imuFloatTime = millis();
			robot.setupIMU();
		}

		void clb_imu_pnc_clear()
		{
			robot.setupIMU();
			pncBeginTime = millis();
			
		}

		void clb_imu_pnc_start()
		{
			robot.imuFloatValue += double(robot.angle - robot.calibratedAngle) / double(millis() - pncBeginTime);
			robot.imuFloatTime = millis();
			pncBeginTime = millis();
			robot.setupIMU();
		}
		
		//Light sensors calibration
		void clb_screenToLightSensorsCalibration()
		{
			robot.display.setScreen(LIGHT_SENSORS_CALIBRATION_SCREEN);
			g_state = LIGHT_SENSORS_CALIBRATION_SCREEN;
		}
		
						
				void clb_beginLightsCalib()
				{
					robot.beginLineCalibration();
				}

				void clb_endLightsCalib()
				{
					robot.endLineCalibration();
				}


//Debug data screen
void clb_screenToDebugData()
{
	clb_screenToEmpty();
	g_state = DEBUG_DATA_SCREEN;
}

//Dribbler screen
void clb_screenToDribbler()
{
	robot.display.setScreen(DRIBBLER_SCREEN);
	g_state = DRIBBLER_SCREEN;
}

		void clb_dribblerInOn()
		{
			robot.manualDribblerControl = 1;
			robot.driblerSpeed1 = -MANUAL_DRIBBLER_CONTROL_SPEED; 
			robot.driblerSpeed2 = -MANUAL_DRIBBLER_CONTROL_SPEED;
		}

		void clb_dribblerOutOn()
		{
			robot.manualDribblerControl = 1;
			robot.driblerSpeed1 = -MANUAL_DRIBBLER_CONTROL_SPEED; 
			robot.driblerSpeed2 = -MANUAL_DRIBBLER_CONTROL_SPEED;
		}

		void clb_dribblerOff()
		{
			robot.manualDribblerControl = 0;
			robot.driblerSpeed1 = 0; 
			robot.driblerSpeed2 = 0;
		}

//Kicker screen		
void clb_screenToKicker()
{
	robot.display.setScreen(KICKER_SCREEN);
	g_state = KICKER_SCREEN;
}

		void clb_kickStraight()
		{
			robot.kick1 = 1;
			robot.kick2 = 1;
		}
		
		void clb_kick1()
		{
			robot.kick1 = 1;
		}

		void clb_kick2()
		{
			robot.kick2 = 1;
		}

//Battery screen
void clb_toBatteryScreen()
{
	clb_screenToEmpty();
	g_state = BATTERY_SCREEN;
}

//KOKOKO screen
void clb_toKokokoScreen()
{
	clb_screenToEmpty();
	g_state = KOKOKO_SCREEN;
}

#endif
