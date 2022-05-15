#include "main.h"

Robot robot;
//BaseFunctional basicFunc(&robot);
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


#define COORDS_ARRAY_SIZE	41
int coordsArr[COORDS_ARRAY_SIZE][2] = {
{131, 135},
{192, 235 },
{201,	240 },
{222,	280 },
{235,	300 },
{258,	340 },
{289,	400 },
{314,	450 },
{337,	500 },
{358,	550 },
{378,	600 },
{397,	650 },
{416,	700 },
{434,	750 },
{450,	800 },
{465,	850 },
{480,	900 },
{496,	950 },
{507,	1000},
{520,	1050},
{536,	1100},
{544,	1150},
{557,	1200},
{568,	1250},
{577,	1300},
{586,	1350},
{595,	1400},
{604,	1450},
{612,	1500},
{621,	1550},
{629,	1600},
{636,	1650},
{644,	1700},
{650,	1750},
{658,	1800},
{664,	1850},
{670,	1900},
{676,	1950},
{682,	2000},
{689,	2050},
{694,	2100}	};
	
uint16_t realDistance(uint16_t dist)
{
		int xAbs = 0;
    int xOne = 0;
    int xTwo = 0;
    int yOne = 0;
    int yTwo = 0;
    int fSign = 1;
    if (dist < 0)
		{
			xAbs = dist * -1;
      fSign = -1;
		}
    else
    {
			xAbs = dist;
		}
    for(int i = 0; i < COORDS_ARRAY_SIZE - 1; i++)
		{
        if (coordsArr[i][0] <= xAbs && xAbs < coordsArr[i + 1][0])
				{
					xOne = coordsArr[i][0];
					xTwo = coordsArr[i + 1][0];
					yOne = coordsArr[i][1];
					yTwo = coordsArr[i + 1][1];
					return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign;
				}
		}
    if (coordsArr[COORDS_ARRAY_SIZE][0] <= xAbs)
		{
        xOne = coordsArr[COORDS_ARRAY_SIZE - 2][0];
        xTwo = coordsArr[COORDS_ARRAY_SIZE - 1][0];
        yOne = coordsArr[COORDS_ARRAY_SIZE - 2][1];
        yTwo = coordsArr[COORDS_ARRAY_SIZE - 1][1];
				return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign;
		}
}






	
	
void enemyFind(T_LIDARSCANINFO& LSI, int quantity, int minDist)
{
//	T_POINT* lidarDots = new T_POINT[robot.lidar.LSI.OneCriclePointNum];
//	for( int i = 0 ; i < LSI.OneCriclePointNum; i++)
//	{
//		lidarDots[i] = LSI.OneCriclePoint[i];
//	}
//		for( int i = 0 ; i < LSI.OneCriclePointNum; i++)
//		{
//			
//		}
	uint32_t maxPoints = 0;
	uint32_t maxPoints2 = 0;
	int saveC1 = -210;
	int saveC2 = -210;
	int saveC3 = -210;
	int saveC4 = -210;
	for(int c = -105; c <= 105; c++)
	{
		int pCoordsY = 0;
		int pCoordsX = 0;
		for(int i = 0; i < LSI.OneCriclePointNum; i++ )
		{
			int y = sin(LSI.OneCriclePoint[i].Angle/57.3) * realDistance(LSI.OneCriclePoint[i].Distance)/10 - robot.getPos().Y;
			if(abs(c - y) < minDist)
			{
				pCoordsY++;
			}
			
			int x = -cos(LSI.OneCriclePoint[i].Angle/57.3) * realDistance(LSI.OneCriclePoint[i].Distance)/10 - robot.getPos().X;
			if(abs(c - x) < minDist)
			{
				pCoordsX++;
			}
			
		}
		if(pCoordsY > quantity && pCoordsY > maxPoints)
		{
			saveC1 = c;
			maxPoints = pCoordsY;
		}
		else if(pCoordsY > quantity && pCoordsY > maxPoints2 && abs(saveC1 - saveC2) >= 80) 
		{
			saveC2 = c;
			maxPoints2 = pCoordsY;
		}
		
		if(pCoordsX > quantity && pCoordsX > maxPoints)
		{
			saveC3 = c;
			maxPoints = pCoordsX;
			//if
		}
		else if(pCoordsX > quantity && pCoordsX > maxPoints2 && abs(saveC3 - saveC4) >= 80) 
		{
			saveC4 = c;
			maxPoints2 = pCoordsX;
		}
		
	}
	
//	maxPoints = 0;
//	
//	for(int c = -210; c <= 210; c++)
//	{
//		int pCoords = 0;
//		for(int i = 0; i < LSI.OneCriclePointNum; i++ )
//		{
//			int y = sin(LSI.OneCriclePoint[i].Angle/57.3) * LSI.OneCriclePoint[i].Distance - robot.getPos().Y;
//			if(abs(c - y) < minDist)
//			{
//				pCoords++;
//			}
//		}
//		if(pCoords > quantity && pCoords > maxPoints && abs(saveC1 - saveC2) >= 80) 
//		{
//			saveC2 = c;
//			maxPoints = pCoords;
//		}
//	}
	
	
	
//	maxPoints = 0;
//	maxPoints2 = 0;
//	int saveC3 = -210;
//	int saveC4 = -210;
//	for(int c = -210; c <= 210; c++)
//	{
//		int pCoords = 0;
//		for(int i = 0; i < LSI.OneCriclePointNum; i++ )
//		{
//			int x = -cos(LSI.OneCriclePoint[i].Angle/57.3) * LSI.OneCriclePoint[i].Distance - robot.getPos().X;
//			if(abs(c - x) < minDist)
//			{
//				pCoords++;
//			}
//		}
//		if(pCoords > quantity && pCoords > maxPoints)
//		{
//			saveC3 = c;
//			maxPoints = pCoords;
//			//if
//		}
//		else if(pCoords > quantity && pCoords > maxPoints2 && abs(saveC3 - saveC4) >= 80) 
//		{
//			saveC4 = c;
//			maxPoints2 = pCoords;
//		}
//	}
	
//	maxPoints = 0;
//	
//	for(int c = -210; c <= 210; c++)
//	{
//		int pCoords = 0;
//		for(int i = 0; i < LSI.OneCriclePointNum; i++ )
//		{
//			int x = -cos(LSI.OneCriclePoint[i].Angle/57.3) * LSI.OneCriclePoint[i].Distance - robot.getPos().X;
//			if(abs(c - x) < minDist)
//			{
//				pCoords++;
//			}
//		}
//		if(pCoords > quantity && pCoords > maxPoints && abs(saveC3 - saveC4) >= 80) 
//		{
//			saveC4 = c;
//			maxPoints = pCoords;
//		}
//	}
	



	for(int i = 0; i < LSI.OneCriclePointNum; i++ )
	{
		int x = -cos(LSI.OneCriclePoint[i].Angle/57.3) * LSI.OneCriclePoint[i].Distance - robot.getPos().X;
		int y = sin(LSI.OneCriclePoint[i].Angle/57.3) * LSI.OneCriclePoint[i].Distance - robot.getPos().Y;
		if(abs(y-saveC1) < minDist || abs(y-saveC2) < minDist || abs(x-saveC3) < minDist || abs(x-saveC4) < minDist)
		{
			LSI.OneCriclePoint[i].Angle = -228;
			LSI.OneCriclePoint[i].Distance = -228;
		}
	}
	//delete[] lidarDots;
}







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
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//	pt toGo = {0, 0};
//	
//	PairSaver ballPosSave;
//	pair<double, double> old = make_pair(0,0);
//	uint32_t predictTime = 0;
//	uint32_t strikeTime = 0;
//	uint32_t stateTime = 0;
//	uint32_t angleCheckTest = 0;
//	bool strike = false;
	uint32_t mainTime = 0;
	volatile uint32_t cycleTime = 0;
//	volatile double v1 = 0, v2 = 0, v3 = 0;
//	robot.lidar.LidarInit(_UART2);
//	volatile uint32_t saveLidar = 0;
	while(1)
	{
		cycleTime = millis() - mainTime;
		mainTime = millis();
		robot.wait(5);
		
		//robot.lidar.ProcessUartRxData();
		//lidarCircles = robot.lidar.LSI.OneCriclePoint[99].Angle;
//		volatile uint32_t lidarAngle = 0;
//		bool draw = false;
		
//	uint32_t minDist = 1000000000;
//		for(int i = 0; i < 1000; i++)
//		{
//			if(robot.lidar.LSI.OneCriclePoint[i].Distance != 0 && robot.lidar.LSI.OneCriclePoint[i].Distance < minDist)
//			{
//				minDist = robot.lidar.LSI.OneCriclePoint[i].Distance;
//				lidarAngle = robot.lidar.LSI.OneCriclePoint[i].Angle;
//			}
//		}
//		if(robot.lidar.LSI.Result == LIDAR_GRAB_SUCESS)//扫描到完整一圈
//		{
//		//draw = true;
//			
////			robot.lidar.LSI.Result=LIDAR_GRAB_ING;//恢复正在扫描状态
////			
////			
////			
////			/**************打印扫描一圈的总点数：lidarscaninfo.OneCriclePointNum*********************************************/
////			//printf("one circle point num:%d\n",lidarscaninfo.OneCriclePointNum);
////		
////			/*****lidarscaninfo.OneCriclePoint[lidarscaninfo.OneCriclePointNum]：存放一圈总点数的角度和距离*********/
////			
////			//打印某个点信息：一圈从零点开始，打印第100个点的角度和距离
////			/*printf("point %d: angle=%5.2f,distance=%5.2fmm\n",100,
////					lidarscaninfo.OneCriclePoint[100].Angle,
////					lidarscaninfo.OneCriclePoint[100].Distance);*/
////			
//		}
		
		func.posCalc();
		//func.testBorder();
		func.SIMP();
		//func.goalkeeper();
		//v1 = func.getRobotClass()->imu.getXa();
		//v2 = func.getRobotClass()->imu.getYa();
		//v3 = func.getRobotClass()->imu.getZa();
		
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
				if (robot.buttons.isChanged(DOWN_BUTTON, true))
				{
					robot.imu.calibrate(228);
					robot.switch5vOff();
					delay(10);
					robot.switch5vOn();
					delay(1);
				}
				if (robot.buttons.isChanged(ESC_BUTTON, true))
				{
					if(robot.playState()) robot.changePlayState();
					else clb_screenToMain();
				}
				
				if (robot.playState()) robot.display.print("playing", 0, 6);
				else robot.display.print("waiting", 0, 6);
				
//				robot.display.print(cycleTime, 0, 15);
				robot.display.print("L:", 2, 1);
				for(int i = 0; i < robot.lidar.LSI.OneCriclePointNum; i++)
				{
					if(robot.lidar.LSI.OneCriclePoint[i].Angle == 90)
					{
					//robot.display.print(robot.lidar.LSI.OneCriclePoint[100].Angle, 2, 3);
					robot.display.print(realDistance(robot.lidar.LSI.OneCriclePoint[i].Distance), 2, 11);
					}
				}
				robot.display.print("Yaw angle: ", 1, 1);
				robot.display.print(robot.imu.getAngle(), 1, 11);
//				robot.display.print("Line:", 3, 1);
//				robot.display.print(robot.lineSensors.getLine(), 3, 7);
//				robot.display.print("Ball x: ", 3, 1);
//				robot.display.print(func.ball.globalPos.X, 3, 11);
//				robot.display.print("Ball y: ", 4, 1);
//				robot.display.print(func.ball.globalPos.Y, 4, 11);
				break;
			
			case CALIBRATIONS_SCREEN:
				//empty
				break;
			
			case LIGHT_SENSORS_CALIBRATION_SCREEN:
				robot.display.print("Line: ", 4, 6);
				robot.display.print(robot.lineSensors.getLine(), 4, 10);
				break;
			
			case DEBUG_DATA_SCREEN:
//			if(draw)enemyFind(robot.lidar.LSI, 5, 5);
//					for(int i = 0; i < robot.lidar.LSI.OneCriclePointNum; i++)
//				{
//					if(robot.lidar.LSI.OneCriclePoint[i].Distance != 228)
//					{
//						int L = realDistance(robot.lidar.LSI.OneCriclePoint[i].Distance) * 32/2100;
//						int xTmp = (L)*sin((robot.lidar.LSI.OneCriclePoint[i].Angle-90)/57.3) + 64;
//						int yTmp = -(L)*cos((robot.lidar.LSI.OneCriclePoint[i].Angle-90)/57.3) + 32;
//						if(xTmp >= 0 && xTmp < 128 && yTmp >= 0 && yTmp < 64)
//						{
//							robot.display.ssd1306.drawPixel(xTmp, yTmp, 1);
//						}
//					}
//					
//				}
//				//robot.lidar.LSI.Result = LIDAR_GRAB_ING;
//				//}
//				//robot.display.clear();
//				//robot.display.print(cycleTime, 0, 0);
//				
//				saveLidar++;
//				if(saveLidar == 2)
//				{
//					robot.display.clear();
//					saveLidar = 0;
//				}
//					robot.display.print("Ball x: ", 2, 1);
//					robot.display.print(func.camBall.pos.X, 2, 11);
//					robot.display.print("Ball y: ", 3, 1);
//					robot.display.print(func.camBall.pos.Y, 3, 11);
//				double yelX = func.camYellow.pos.X;
//				double yelY = func.camYellow.pos.Y;
//			
//				double blueX = func.camBlue.pos.X;
//				double blueY = func.camBlue.pos.Y;
//				robot.display.print("Yx", 0, 0);
//				robot.display.print(yelX, 0, 3);
//			
//				robot.display.print("Yy", 0, 8);
//				robot.display.print(yelY, 0, 11);
//			
//				robot.display.print("Bx", 1, 0);
//				robot.display.print(blueX, 1, 3);
//			
//				robot.display.print("By", 1, 8);
//				robot.display.print(blueY, 1, 11);
//			
//				robot.display.print("Bd", 3, 0);
//				robot.display.print(sqrt(double(blueX*blueX + blueY*blueY)), 3, 3);
//			
//				robot.display.print("Yd", 3, 8);
//				robot.display.print(sqrt(double(yelX*yelX + yelY*yelY)), 3, 11);
				//robot.display.print("", 0, 0);
				//robot.display.print(func.camYellow.pos.X, 0, 3);
			
//				robot.display.print("S", 2, 8);
//				robot.display.print(func.state, 2, 10);
				//robot.display.print("Line data: ", 0, 1);
				//robot.display.print(robot.lineSensors.getLine(), 0, 11);
//				robot.display.print("Yaw angle: ", 0, 1);
//				robot.display.print(robot.imu.getAngle(), 0, 11);
//				robot.display.print("AX: ", 1, 1);
//				robot.display.print(double(func.getRobotClass()->imu.getXa()), 1, 5);
//				robot.display.print("AY: ", 2, 1);
//				robot.display.print(double(func.getRobotClass()->imu.getYa()), 2, 5);
//				robot.display.print("AZ: ", 3, 1);
//				robot.display.print(double(func.getRobotClass()->imu.getZa()), 3, 5);
//			
//						robot.display.print("GX: ", 1, 1);
//				robot.display.print(robot.imu.mpuSensor.xGyroOffset, 1, 5);
//				robot.display.print("GY: ", 2, 1);
//				robot.display.print(robot.imu.mpuSensor.yGyroOffset, 2, 5);
//				robot.display.print("GZ: ", 3, 1);
//				robot.display.print(robot.imu.mpuSensor.zGyroOffset, 3, 5);
//			
//			robot.display.print("GX: ", 1, 10);
//				robot.display.print(robot.imu.getXg(), 1, 14);
//				robot.display.print("GY: ", 2, 10);
//				robot.display.print(robot.imu.getYg(), 2, 14);
//				robot.display.print("GZ: ", 3, 10);
//				robot.display.print(robot.imu.getZg(), 3, 14);
//				//robot.display.print(robot.ADC_2.read(BALL_SENSOR), 2, 1);
				
				robot.display.print("x: ", 2, 1);
				robot.display.print(robot.getPos().X, 2, 9);
//				//robot.display.print(robot.camera.yellow.X, 2, 9);
				robot.display.print("y: ", 3, 1);
				robot.display.print(robot.getPos().Y, 3, 9);
////				robot.display.print("Ball sens: ", 3, 1);
////				robot.display.print(robot.ballSensor.getSensorValue(), 3, 14);
				//robot.display.print(robot.camera.yellow.Y, 3, 9);
				//robot.display.print(int(millis()), 3, 15);
			
				break;
			
			case DRIBBLER_SCREEN:
				robot.display.print("S ", 4, 5);
				robot.display.print(robot.ballSensor.getValue(), 4, 8);
				break;
			
			case KICKER_SCREEN:
				if(robot.kicker.isCharged()) 
					robot.display.print("done", 0, 11);
				else
					robot.display.print("charging", 0, 11);
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
//		if(g_state != DEBUG_DATA_SCREEN)
//		{
			robot.display.clear();
//		}
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
	robot.display.addToScreen(*(new Command("Charge", clb_initCharge)));
	robot.display.addToScreen(*(new Command("Kick straight", clb_kickStraight)));
	robot.display.addToScreen(*(new Command("Kick diagonal 1", clb_kick2)));
	robot.display.addToScreen(*(new Command("Kick diagonal 2", clb_kick1)));
	
	robot.display.setScreen(BATTERY_SCREEN);
	robot.display.addEsc(*(new Command("", clb_screenToMain)));
	//empty
	
	robot.display.setScreen(MENU_SCREEN);
	
}
