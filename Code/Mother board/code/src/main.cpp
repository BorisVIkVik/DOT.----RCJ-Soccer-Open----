#include "main.h"

Robot robot;
BaseFunctional basicFunc(&robot);

#include "callbacks.h"

void setupScreens();


int main()

{
	sysStart();
	
	robot.init();
	
	setupScreens();
	
	writeStrUART(DEBUG_UART, "\r\nStart\r\n");
	
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
	
	volatile int check = 0;
	robot.motorDrivers.disableMotors();
	while(1)
	{
		check = robot.camera.ballX;
		if(robot.playState())
		{
			int xTmp = robot.camera.ballX;
			int yTmp = -robot.camera.yGoalY - 62;
		basicFunc.move2(basicFunc.genATMPoint(xTmp,yTmp,min2(2.0, 0.08 * sqrt(double(xTmp * xTmp + yTmp * yTmp)))), 0);
		robot.wait(1);
		}
		else
		{
			robot.motorDrivers.setMotors(0,0,0,0);
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
				
				break;
			
			case GAME_SCREEN:
				if (robot.buttons.isChanged(ENTER_BUTTON, true))
				{
					if(millis() - notGameScreenTimer > 500) robot.changePlayState();
				}
				if (robot.buttons.isChanged(UP_BUTTON, true))
				{
					robot.imu.setZeroAngle();
				}
				if (robot.buttons.isChanged(ESC_BUTTON, true))
				{
					if(robot.playState()) robot.changePlayState();
					else clb_screenToMain();
				}
				
				if (robot.playState()) robot.display.print("playing", 0, 6);
				else robot.display.print("waiting", 0, 6);
				
				robot.display.print("Yaw angle: ", 2, 1);
				robot.display.print(robot.imu.getAngle(), 2, 11);
				break;
			
			case CALIBRATIONS_SCREEN:
				//empty
				break;
			
			case LIGHT_SENSORS_CALIBRATION_SCREEN:
				robot.display.print("Line: ", 4, 6);
				robot.display.print(robot.lineSensors.getLine(), 4, 10);
				break;
			
			case DEBUG_DATA_SCREEN:
				robot.display.print("Line data: ", 0, 1);
				robot.display.print(robot.lineSensors.getLine(), 0, 11);
				robot.display.print("Yaw angle: ", 1, 1);
				robot.display.print(robot.imu.getAngle(), 1, 11);
				robot.display.print(robot.ADC_2.read(BALL_SENSOR), 2, 1);
				robot.display.print("ballX: ", 2, 1);
				robot.display.print(robot.camera.ballX, 2, 9);
				robot.display.print("ballY: ", 3, 1);
				robot.display.print(robot.camera.ballY, 3, 9);
				robot.display.print(int(millis()), 3, 15);
			
				break;
			
			case DRIBBLER_SCREEN:
				robot.display.print("S ", 4, 5);
				robot.display.print(robot.ballSensor.getValue(), 4, 8);
				break;
			
			case KICKER_SCREEN:
				//empty
				break;
			
			case BATTERY_SCREEN:
				robot.display.print("Battery:", 0, 1);
				robot.display.print("S1: ", 0, 8);
				robot.display.print(robot.battery.getCellVoltage(0), 0, 11);
				robot.display.print("S2: ", 1, 8);
				robot.display.print(robot.battery.getCellVoltage(1), 1, 11);
				robot.display.print("S3: ", 2, 8);
				robot.display.print(robot.battery.getCellVoltage(2), 2, 11);
				robot.display.print("S4: ", 3, 8);
				robot.display.print(robot.battery.getCellVoltage(3), 3, 11);
				break;
			
			default:
				break;
		}
		
		if (g_state == GAME_SCREEN)
			robot.buttons.disable();
		else
			robot.buttons.enable();
		
		
		robot.display.print(int(robot.battery.getPercentage()), 4, 0);
		robot.display.print("%", 4, 3);
		if (robot.battery.getVoltage() < 14.5) 
		{
			robot.display.print("CHANGE BUTT", 4, 5);
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
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	
	robot.display.setScreen(MENU_SCREEN);
	robot.display.addToScreen(*(new Command("Game", clb_screenToGame)));
	robot.display.addToScreen(*(new Command("Calibrations", clb_screenToCalibrations))); 
	robot.display.addToScreen(*(new Command("Debug data", clb_screenToDebugData)));	
	robot.display.addToScreen(*(new Command("Dribbler", clb_screenToDribbler)));
	robot.display.addToScreen(*(new Command("Kicker", clb_screenToKicker)));
	robot.display.addToScreen(*(new Command("Battery", clb_toBatteryScreen)));
	
	robot.display.setScreen(GAME_SCREEN);
	robot.display.addToScreen(*(new Command("Play: ", clb_doNothing)));
	
	robot.display.setScreen(CALIBRATIONS_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain))); 
	robot.display.addToScreen(*(new Command("Zero IMU", clb_imuSetup)));
	robot.display.addToScreen(*(new Command("Calibrate IMU", clb_imuCalib)));
	//robot.display.addToScreen(*(new Command("Clear PNC", clb_imu_pnc_clear)));
	//robot.display.addToScreen(*(new Command("Start PNC", clb_imu_pnc_start)));
	robot.display.addToScreen(*(new Command("Calibrate light sens...", clb_screenToLightSensorsCalibration)));
	
	robot.display.setScreen(LIGHT_SENSORS_CALIBRATION_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToCalibrations))); 
	robot.display.addToScreen(*(new Command("Turn on backlight", clb_turnOnBackLight)));
	robot.display.addToScreen(*(new Command("Turn off backlight", clb_turnOffBackLight)));
	robot.display.addToScreen(*(new Command("Begin calibration", clb_beginLightsCalib)));
	robot.display.addToScreen(*(new Command("End calibration", clb_endLightsCalib)));
	
	robot.display.setScreen(DEBUG_DATA_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	//empty
	
	robot.display.setScreen(DRIBBLER_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	robot.display.addToScreen(*(new Command("Dribbler in ON", clb_dribblerInOn)));
	robot.display.addToScreen(*(new Command("Dribbler out ON", clb_dribblerOutOn)));
	robot.display.addToScreen(*(new Command("Dribbler OFF", clb_dribblerOff)));
	
	robot.display.setScreen(KICKER_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	robot.display.addToScreen(*(new Command("Kick straight", clb_kickStraight)));
	robot.display.addToScreen(*(new Command("Kick diagonal 1", clb_kick2)));
	robot.display.addToScreen(*(new Command("Kick diagonal 2", clb_kick1)));
	
	robot.display.setScreen(BATTERY_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	//empty
	
	robot.display.setScreen(MENU_SCREEN);
	
}
