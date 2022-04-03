#include "main.h"

Robot robot;
BaseFunctional basicFunc(&robot);

#include "callbacks.h"


void setupScreens();



int main()
{
	sysStart();
	
	clb_imu_pnc_clear();
	robot.init();
	setupScreens();
	writeStrUART(DEBUG_UART, "\r\nStart\r\n");
	
	long long int driverTimer = 0;
	long long int notGameScreenTimer = millis();
	long long int t = millis(), tt;
	
	int16_t checlBallx = 0;
	int16_t checlBally = 0;
	volatile int cringe = 0;
	int checlTime = 0;
	volatile int checkX =0;
	volatile int checkY =0;
	
	volatile int goalYellowX =0;
	volatile int goalYellowY =0;
	
	
	int32_t kickTime = 0;
	volatile int8_t state = 0;
	tt = millis() - t;
	t = millis();
	while(1)
	{
		
		
		robot.syncVIM();
		
		robot.wait(1);
		
		if(millis() - driverTimer > 100)
		{
			robot.motors[0].attemptCurrent();
			robot.motors[1].attemptCurrent();
			robot.motors[2].attemptCurrent();
			robot.motors[3].attemptCurrent();
			driverTimer = millis();
		}
		
		
		
///////////////////////////		USER INTERFACE		///////////////////////////
		robot.display.update();
		
		if(g_state != GAME_SCREEN)
		{
			robot.setPlayState(0);
			notGameScreenTimer = millis();
		}
		
		switch (g_state)
		{
			case MENU_SCREEN:
				//robot.disableMotors();
				robot.setMotors(0,0,0,0);
				break;
			
			case GAME_SCREEN:
				robot.enableMotors();

				if (robot.button[0].isChanged(true))
				{
					if(millis() - notGameScreenTimer > 500) robot.changePlayState();
				}
				if (robot.button[1].isChanged(true))
				{
					clb_imu_pnc_start();
				}
				if (robot.button[2].isChanged(true))
				{
					clb_screenToMain();
				}
				
				if (robot.playState()) robot.print("playing", 0, 6);
				else robot.print("waiting", 0, 6);
				
				robot.print("Yaw angle: ", 2, 1);
				robot.print(robot.angle - robot.calibratedAngle, 2, 11);
				break;
			
			case CALIBRATIONS_SCREEN:
				//empty
				break;
			
			case LIGHT_SENSORS_CALIBRATION_SCREEN:
				robot.print("Line data: ", 3, 1);
				robot.print(robot.line, 3, 10);
				break;
			
			case DEBUG_DATA_SCREEN:
				robot.print("Line data: ", 1, 1);
				robot.print(robot.line, 1, 11);
				robot.print("Yaw angle: ", 2, 1);
				robot.print(robot.angle - robot.calibratedAngle, 2, 11);
				robot.print(robot.ADC_2.read(BALL_SENSOR_1), 3, 1);
				robot.print(robot.ADC_2.read(BALL_SENSOR_2), 3, 6);
				break;
			
			case DRIBBLER_SCREEN:
				robot.print("S1 ", 4, 5);
				robot.print(robot.ball[0], 4, 8);
				robot.print("; S2 ", 4, 9);
				robot.print(robot.ball[1], 4, 13);
				break;
			
			case KICKER_SCREEN:
				//empty
				break;
			
			case BATTERY_SCREEN:
				robot.print("Battery:", 1, 1);
				robot.print("S1: ", 1, 8);
				robot.print(robot.cells[0], 1, 11);
				robot.print("S2: ", 2, 8);
				robot.print(robot.cells[1], 2, 11);
				robot.print("S3: ", 3, 8);
				robot.print(robot.cells[2], 3, 11);
				robot.print("S4: ", 4, 8);
				robot.print(robot.cells[3], 4, 11);
				break;
			
			case KOKOKO_SCREEN:
				robot.print(GLOBAL_ERROR, 1, 1);
				robot.print(robot.imuFloatValue * 1000, 2, 2);
				robot.print(robot.angleChange, 2, 8);
				robot.print(robot.angle - robot.calibratedAngle, 3, 3);
				robot.setMotors(10, -10, -10, 10);
				break;
			
			default:
				break;
		}
		
		if (g_state == GAME_SCREEN)
			disableButtons = true;
		else
			disableButtons = false;
		
		
		robot.print(int(100 * (robot.batteryVoltage - 13.6) / (16.8 - 13.6)), 4, 1);
		robot.print("%", 4, 3);
		if (robot.batteryVoltage < 14.5) 
		{
			robot.print("CHANGE BUTT", 4, 5);
		}
		
		robot.display.show();
		robot.display.clear();
		
		robot.wait(5);
	}
}





///////////////////////////////////////////////////////////////////////////
void setupScreens()
{
	
	robot.display.setScreen(EMPTY_SCREEN);
	robot.display.addToScreen(*(new Command("Return <-", clb_screenToMain)));
	
	robot.display.setScreen(MENU_SCREEN);
	robot.display.addToScreen(*(new Command("Game", clb_screenToGame)));
	robot.display.addToScreen(*(new Command("Calibrations", clb_screenToCalibrations))); 
	robot.display.addToScreen(*(new Command("Debug data", clb_screenToDebugData)));	
	robot.display.addToScreen(*(new Command("Dribbler", clb_screenToDribbler)));
	robot.display.addToScreen(*(new Command("Kicker", clb_screenToKicker)));
	robot.display.addToScreen(*(new Command("Battery", clb_toBatteryScreen)));
	robot.display.addToScreen(*(new Command("Some shit by Roman", clb_toKokokoScreen)));
	
	robot.display.setScreen(GAME_SCREEN);
	robot.display.addToScreen(*(new Command("Play: ", clb_doNothing)));
	
	robot.display.setScreen(CALIBRATIONS_SCREEN);
	robot.display.addToScreen(*(new Command("Return <-", clb_screenToMain))); 
	robot.display.addToScreen(*(new Command("ZerofY", clb_imuSetup)));
	robot.display.addToScreen(*(new Command("Calibrate IMU", clb_imuCalib)));
	robot.display.addToScreen(*(new Command("Clear PNC", clb_imu_pnc_clear)));
	robot.display.addToScreen(*(new Command("Start PNC", clb_imu_pnc_start)));
	robot.display.addToScreen(*(new Command("Calibrate light sens...", clb_screenToLightSensorsCalibration)));
	
	robot.display.setScreen(LIGHT_SENSORS_CALIBRATION_SCREEN);
	robot.display.addToScreen(*(new Command("Return <-", clb_screenToCalibrations))); 
	robot.display.addToScreen(*(new Command("Begin calibration", clb_beginLightsCalib)));
	robot.display.addToScreen(*(new Command("End calibration", clb_endLightsCalib)));
	
	robot.display.setScreen(DEBUG_DATA_SCREEN);
	//empty
	
	robot.display.setScreen(DRIBBLER_SCREEN);
	robot.display.addToScreen(*(new Command("Return <-", clb_screenToMain)));
	robot.display.addToScreen(*(new Command("Dribbler in ON", clb_dribblerInOn)));
	//robot.display.addToScreen(*(new Command("Dribbler out ON", clb_dribblerOutOn)));
	robot.display.addToScreen(*(new Command("Dribbler OFF", clb_dribblerOff)));
	
	robot.display.setScreen(KICKER_SCREEN);
	robot.display.addToScreen(*(new Command("Return <-", clb_screenToMain)));
	robot.display.addToScreen(*(new Command("Kick straight", clb_kickStraight)));
	robot.display.addToScreen(*(new Command("Kick diagonal 1", clb_kick2)));
	robot.display.addToScreen(*(new Command("Kick diagonal 2", clb_kick1)));
	
	robot.display.setScreen(BATTERY_SCREEN);
	//empty
	
	robot.display.setScreen(KOKOKO_SCREEN);
	//empty
	
	robot.display.setScreen(MENU_SCREEN);
	
}
