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
	while(1)
	{
		
		robot.wait(1);
		
		
		
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
				//robot.motorDrivers.disableMotors();
				robot.motorDrivers.setMotors(0,0,0,0);
				break;
			
			case GAME_SCREEN:
				robot.motorDrivers.enableMotors();

				if (robot.buttons.isChanged(0, true))
				{
					if(millis() - notGameScreenTimer > 500) robot.changePlayState();
				}
				if (robot.buttons.isChanged(1, true))
				{
					clb_imu_pnc_start();
				}
				if (robot.buttons.isChanged(3, true))
				{
					clb_screenToMain();
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
				robot.display.print("Line data: ", 3, 1);
				robot.display.print(robot.lineSensors.getLine(), 3, 10);
				break;
			
			case DEBUG_DATA_SCREEN:
				robot.display.print("Line data: ", 1, 1);
				robot.display.print(robot.lineSensors.getLine(), 1, 11);
				robot.display.print("Yaw angle: ", 2, 1);
				robot.display.print(robot.imu.getAngle(), 2, 11);
				robot.display.print(robot.ADC_2.read(BALL_SENSOR), 3, 1);
				break;
			
			case DRIBBLER_SCREEN:
				robot.display.print("S ", 4, 5);
				robot.display.print(robot.ballSensor.getValue(), 4, 8);
				break;
			
			case KICKER_SCREEN:
				//empty
				break;
			
			case BATTERY_SCREEN:
				robot.display.print("Battery:", 1, 1);
				robot.display.print("S1: ", 1, 8);
				robot.display.print(robot.battery.getCellVoltage(0), 1, 11);
				robot.display.print("S2: ", 2, 8);
				robot.display.print(robot.battery.getCellVoltage(1), 2, 11);
				robot.display.print("S3: ", 3, 8);
				robot.display.print(robot.battery.getCellVoltage(2), 3, 11);
				robot.display.print("S4: ", 4, 8);
				robot.display.print(robot.battery.getCellVoltage(3), 4, 11);
				break;
			
			case KOKOKO_SCREEN:
				robot.display.print(GLOBAL_ERROR, 1, 1);
				robot.display.print(robot.imu.imuFloatValue * 1000, 2, 2);
				robot.display.print(robot.imu.angleChange, 2, 8);
				robot.display.print(robot.imu.getAngle(), 3, 3);
				robot.motorDrivers.setMotors(10, -10, -10, 10);
				break;
			
			default:
				break;
		}
		
		if (g_state == GAME_SCREEN)
			robot.buttons.disable();
		else
			robot.buttons.enable();
		
		
		robot.display.print(int(robot.battery.getPercentage()), 4, 1);
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
	robot.display.addToScreen(*(new Command("Some shit by Roman", clb_toKokokoScreen)));
	
	robot.display.setScreen(GAME_SCREEN);
	robot.display.addToScreen(*(new Command("Play: ", clb_doNothing)));
	
	robot.display.setScreen(CALIBRATIONS_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain))); 
	robot.display.addToScreen(*(new Command("ZerofY", clb_imuSetup)));
	robot.display.addToScreen(*(new Command("Calibrate IMU", clb_imuCalib)));
	robot.display.addToScreen(*(new Command("Clear PNC", clb_imu_pnc_clear)));
	robot.display.addToScreen(*(new Command("Start PNC", clb_imu_pnc_start)));
	robot.display.addToScreen(*(new Command("Calibrate light sens...", clb_screenToLightSensorsCalibration)));
	
	robot.display.setScreen(LIGHT_SENSORS_CALIBRATION_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToCalibrations))); 
	robot.display.addToScreen(*(new Command("Begin calibration", clb_beginLightsCalib)));
	robot.display.addToScreen(*(new Command("End calibration", clb_endLightsCalib)));
	
	robot.display.setScreen(DEBUG_DATA_SCREEN);
	//empty
	
	robot.display.setScreen(DRIBBLER_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	robot.display.addToScreen(*(new Command("Dribbler in ON", clb_dribblerInOn)));
	//robot.display.addToScreen(*(new Command("Dribbler out ON", clb_dribblerOutOn)));
	robot.display.addToScreen(*(new Command("Dribbler OFF", clb_dribblerOff)));
	
	robot.display.setScreen(KICKER_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	robot.display.addToScreen(*(new Command("Kick straight", clb_kickStraight)));
	robot.display.addToScreen(*(new Command("Kick diagonal 1", clb_kick2)));
	robot.display.addToScreen(*(new Command("Kick diagonal 2", clb_kick1)));
	
	robot.display.setScreen(BATTERY_SCREEN);
	//empty
	
	robot.display.setScreen(KOKOKO_SCREEN);
	//empty
	
	robot.display.setScreen(MENU_SCREEN);
	
}
