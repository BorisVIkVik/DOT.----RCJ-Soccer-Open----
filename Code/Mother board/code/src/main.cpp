#include "main.h"

Robot robot;
BaseFunctional basicFunc(&robot);
Functional func(&robot);
#include "callbacks.h"

//#define KALMAN
#define KALMAN_K		0.5

//#define PREDICTION
#define CAMERA_LATENCY	48

#define SPEED_CALC_TIME	200

void updateKalman(CameraObject &obj, double K, pair<double, double> objV, pair<double, double> rpV, double dt)
{
	dt /= 1000;
	obj.pos.X = obj.pos.X*K + (obj.oldPos.X + (objV.X - rpV.X)*dt)*(1 - K);
	obj.pos.Y = obj.pos.Y*K + (obj.oldPos.Y + (objV.Y - rpV.Y)*dt)*(1 - K);
	obj.oldPos = obj.pos;
}

void updatePrediction(CameraObject &obj, pair<double, double> objV, pair<double, double> rpV, double dt)
{
	dt /= 1000;
	obj.pos.X = obj.pos.X + (objV.X - rpV.X)*dt;
	obj.pos.Y = obj.pos.Y + (objV.Y - rpV.Y)*dt;
}


void setupScreens();




	 
	pt goalPoints[6] = {{70, -97}, {70, -89}, {55, -74}, {-55, -74}, {-70, -89}, {-70, -97}};
	segment goalLines[5] = {{0, 1, 60, -25, 25, 0, 0}, {2, 5, 250, 25, 50, 0, 0}, {-2, 5, 250, -50, -25, 0, 0}, {1, 0, 50, 0, 0, -90, -70}, {1, 0, -50, 0, 0, -90, -70}};















int main()

{
	sysStart();
	robot.init();
	setupScreens();
	
	writeStrUART(DEBUG_UART, "\r\nStart\r\n");
	
	uint32_t notGameScreenTimer = millis();

	uint32_t time = millis();
	uint32_t oldTime = time;
	uint16_t dt = time - oldTime;
	
	
	pair<int, int> zero;
	zero.X = 0;
	zero.Y = 0;
	
	//PairSaver robotAngle, robotVelocity;
	//pair<double, double> robotA, robotV, ballV;
	//pair<double, double> robotGlobalPos;

	//CameraObject camYellow, camBlue, camBall;
	//FieldObject ball;
	
	volatile int check = 0;
	robot.motorDrivers.disableMotors();
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	pt toGo = {0, 0};
	
	PairSaver ballPosSave;
	pair<double, double> old = make_pair(0,0);
	uint32_t predictTime = 0;
	uint32_t strikeTime = 0;
	uint32_t stateTime = 0;
	uint32_t angleCheckTest = 0;
	bool strike = false;
	uint32_t mainTime = 0;
	volatile uint32_t cycleTime = 0;
	while(1)
	{
		cycleTime = millis() - mainTime;
		mainTime = millis();
		robot.wait(5);
		func.posCalc();
		func.strategy2();

		
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
				
				robot.display.print(cycleTime, 0, 15);
				robot.display.print("Yaw angle: ", 2, 1);
				robot.display.print(robot.imu.getAngle(), 2, 11);
				robot.display.print("Ball x: ", 3, 1);
				robot.display.print(func.ball.globalPos.X, 3, 11);
				robot.display.print("Ball y: ", 4, 1);
				robot.display.print(func.ball.globalPos.Y, 4, 11);
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
				//robot.display.print(robot.ADC_2.read(BALL_SENSOR), 2, 1);
				robot.display.print("x: ", 2, 1);
				robot.display.print(robot.getPos().X, 2, 9);
				//robot.display.print(robot.camera.yellow.X, 2, 9);
				robot.display.print("y: ", 3, 1);
				robot.display.print(robot.getPos().Y, 3, 9);
				robot.display.print("Ball sens: ", 3, 1);
				robot.display.print(robot.ballSensor.getSensorValue(), 3, 14);
				//robot.display.print(robot.camera.yellow.Y, 3, 9);
				//robot.display.print(int(millis()), 3, 15);
			
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
