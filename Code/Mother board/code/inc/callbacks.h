#ifndef __MENU_CALLBACKS__
#define __MENU_CALLBACKS__

#define MANUAL_DRIBBLER_CONTROL_SPEED 400

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
	
	robot.switch5vOff();
	robot.lineSensors.turnOffSensors();
	robot.motorDrivers.disableMotors();
}




//Game screen
void clb_screenToGame()
{
	g_state = GAME_SCREEN;
	robot.display.setScreen(GAME_SCREEN);
	robot.motorDrivers.enableMotors();
	robot.switch5vOn();
	robot.lineSensors.turnOnSensors();
	robot.buttons.setChanged(0, false);
	robot.buttons.setChanged(1, false);
	robot.buttons.setChanged(2, false);
	robot.buttons.setChanged(3, false);
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
			robot.imu.setZeroAngle();
			/*
			robot.imu.imuFloatTime = millis();
			robot.imu.setZeroAngle();
			*/
		}

		void clb_imu_pnc_clear()
		{
			/*
			robot.imu.setZeroAngle();
			pncBeginTime = millis();
			*/
		}

		void clb_imu_pnc_start()
		{
			/*
			robot.imu.imuFloatValue += robot.imu.getAngle() / double(millis() - pncBeginTime);
			robot.imu.imuFloatTime = millis();
			pncBeginTime = millis();
			robot.imu.setZeroAngle();
			*/
		}
		
		//Light sensors calibration
		void clb_screenToLightSensorsCalibration()
		{
			robot.display.setScreen(LIGHT_SENSORS_CALIBRATION_SCREEN);
			g_state = LIGHT_SENSORS_CALIBRATION_SCREEN;
		}
		
						
				void clb_beginLightsCalib()
				{
					robot.lineSensors.beginLineCalibration();
				}

				void clb_endLightsCalib()
				{
					robot.lineSensors.endLineCalibration();
				}
				
				void clb_turnOnBackLight()
				{
					robot.lineSensors.turnOnSensors();
				}
				
				void clb_turnOffBackLight()
				{
					robot.lineSensors.turnOffSensors();
				}


//Debug data screen
void clb_screenToDebugData()
{
	clb_screenToEmpty();
	g_state = DEBUG_DATA_SCREEN;
	robot.switch5vOn();
	robot.lineSensors.turnOnSensors();
}

//Dribbler screen
void clb_screenToDribbler()
{
	robot.display.setScreen(DRIBBLER_SCREEN);
	g_state = DRIBBLER_SCREEN;
}

		void clb_dribblerInOn()
		{
			robot.motorDrivers.setDribbler(-MANUAL_DRIBBLER_CONTROL_SPEED);
		}

		void clb_dribblerOutOn()
		{
			robot.motorDrivers.setDribbler(MANUAL_DRIBBLER_CONTROL_SPEED);
		}

		void clb_dribblerOff()
		{
			robot.motorDrivers.setDribbler(0);
		}

//Kicker screen		
void clb_screenToKicker()
{
	robot.display.setScreen(KICKER_SCREEN);
	g_state = KICKER_SCREEN;
}

		void clb_initCharge()
		{
			robot.kicker.initCharge();
		}

		void clb_kickStraight()
		{
			robot.kicker.kick(1, 1);
		}
		
		void clb_kick1()
		{
			robot.kicker.kick(1, 0);
		}

		void clb_kick2()
		{
			robot.kicker.kick(0, 1);
		}

//Battery screen
void clb_toBatteryScreen()
{
	clb_screenToEmpty();
	g_state = BATTERY_SCREEN;
}


#endif
