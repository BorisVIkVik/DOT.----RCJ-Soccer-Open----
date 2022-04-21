#include "main.h"

Robot robot;
BaseFunctional basicFunc(&robot);

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




//TEST AREA-------------------------------------------------------------
	struct pt {
		double x, y;
	};
	 
	struct line {
		double a, b, c;
	};
	 
	struct segment {
		double a, b, c, leftX, rightX, downY, upY;
	}; 

	double distanceVec(pair<double, double> f, pair<double, double> s)
	{
		return sqrt((f.X - s.X) * (f.X - s.X) + (f.Y - s.Y) * (f.Y - s.Y));
	}
	const double EPS = 1e-9;
	 
	double det (double a, double b, double c, double d) {
		return a * d - b * c;
	}
	 
	bool intersect (segment m, line n, pt & res) {
		double zn = det (m.a, m.b, n.a, n.b);
		if (abs (zn) < EPS)
			return false;
		pt tmp;
		tmp.x = - det (m.c, m.b, n.c, n.b) / zn;
		tmp.y = - det (m.a, m.c, n.a, n.c) / zn;
			if (m.leftX != m.rightX && !(m.leftX <= tmp.x && tmp.x <= m.rightX)) 
					return false;
			if (m.downY != m.upY && !(m.downY <= tmp.y && tmp.y <= m.upY))  
					return false;
		res = tmp;
		return true;
	}
	 
	pt goalPoints[6] = {{70, -97}, {70, -89}, {55, -74}, {-55, -74}, {-70, -89}, {-70, -97}};
	segment goalLines[5] = {{0, 1, 60, -25, 25, 0, 0}, {2, 5, 250, 25, 50, 0, 0}, {-2, 5, 250, -50, -25, 0, 0}, {1, 0, 50, 0, 0, -90, -70}, {1, 0, -50, 0, 0, -90, -70}};

	//(y2 - y1)*x + y * (x1 - x2) +  y1 * (x2 - x1) - (y2 - y1)*x1 = 0 
	//A = (y2 - y1); B = (x1 - x2); C = y1 * (x2 - x1) - (y2 - y1) * x1

	// int A = (robot.camera.ballY + 97);
	// int B = (0 - robot.camera.ballX);
	// int C = (-97 * (robot.camera.ballX - 0) - (robot.camera.ballY + 97) * 0);



	
	
	//TEST AREA-------------------------------------------------------------













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
	
	PairSaver robotAngle, robotVelocity;
	pair<double, double> robotA, robotV, ballV;
	pair<double, double> robotGlobalPos;

	CameraObject camYellow, camBlue, camBall;
	FieldObject ball;
	
	volatile int check = 0;
	robot.motorDrivers.disableMotors();
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	pt toGo = {0, 0};
	
	PairSaver ballPosSave;
	pair<double, double> old = make_pair(0,0);
	uint32_t predictTime = 0;
	while(1)
	{

		robot.wait(5);
		time = millis();
		dt = time - oldTime;
		oldTime = time;

		robotAngle.add(make_pair(robot.imu.getAngle(), 0), time);
		robotVelocity.add(robot.getV(), time);
		robotA = robotAngle.pop(time - CAMERA_LATENCY);
		robotV = robotVelocity.pop(time - CAMERA_LATENCY);
		robotV.X *= 100;
		robotV.Y *= 100;
		
		ballV = ball.speedSaver.pop(time - CAMERA_LATENCY);

		camYellow.pos = rotate(robot.camera.yellow, -robotA.first);
		camBlue.pos = rotate(robot.camera.blue, -robotA.first);
		camBall.pos = rotate(robot.camera.ball, -robotA.first);

		#ifdef KALMAN
		updateKalman(camYellow, KALMAN_K, zero, robotV, dt);
		updateKalman(camBlue, KALMAN_K, zero, robotV, dt);
		updateKalman(camBall, KALMAN_K, ballV, robotV, dt);
		#endif
		#ifdef PREDICTION
		updatePrediction(camYellow, zero, robotV, CAMERA_LATENCY);
		updatePrediction(camBlue, zero, robotV, CAMERA_LATENCY);
		updatePrediction(camBall, ballV, robotV, CAMERA_LATENCY);
		#endif

		robot.updateSelfPos(camYellow.pos, camBlue.pos);
		robotGlobalPos = robot.getPos();
		ball.update(camBall.pos, robotGlobalPos, time, SPEED_CALC_TIME);
		
		ballPosSave.add(ball.globalPos, time);
	/*
		check = ball.globalPos.X;
		if(robot.playState())
		{
			int xTmp = ball.globalPos.X - robotGlobalPos.X;
			int yTmp = robotGlobalPos.Y - 70;
		basicFunc.move2(basicFunc.genATMPoint(xTmp,yTmp,min2(2.0, 0.08 * sqrt(double(xTmp * xTmp + yTmp * yTmp)))), 0);
		robot.wait(1);
		}
		else
		{
			robot.motorDrivers.setMotors(0,0,0,0);
		}
	*/
	line goalToRobot;
	int lineA = 0;
	int lineB = 0;
	int lineC = 0;
	//TEST AREA-------------------------------------------------------------
	int timeBack = 200;
	if(millis() - predictTime > 500)
	{
		old = ball.globalPos;
		predictTime = millis();
	}
	if(distanceVec(old, ball.globalPos) > 5)
	{
//		lineA = 0 + 20;
//		lineB = 22 - 21;
//		lineC = 22 * 20;
		//A = (y2 - y1); B = (x1 - x2); C = y1 * (x2 - x1) - (y2 - y1) * x1
//		lineA = (ball.globalPos.Y - ballPosSave.pop(time - timeBack).Y);
	lineA = (ball.globalPos.Y - old.Y);
	lineB = (old.X - ball.globalPos.X);
	lineC = old.Y * (ball.globalPos.X - old.X) - (ball.globalPos.Y - old.Y) * old.X;
	}
//		lineB = (ballPosSave.pop(time - timeBack).X - ball.globalPos.X);
//		lineC = ballPosSave.pop(time - timeBack).Y * (ball.globalPos.X - ballPosSave.pop(time - timeBack).X) - (ball.globalPos.Y - ballPosSave.pop(time - timeBack).Y) * ballPosSave.pop(time - timeBack).X;
//	}
	else
	{
		lineA = (ball.globalPos.Y + 97);
		lineB = (0 - ball.globalPos.X);
		lineC = (-97 * ball.globalPos.X);
	}

	
	goalToRobot.a = lineA;
	goalToRobot.b = lineB;
	goalToRobot.c = lineC;
	
	bool foundToGoPoint = false;
  
		if(abs(ball.globalPos.Y) < 75)
		{

			
			if(intersect(goalLines[0], goalToRobot, toGo))
			{
					foundToGoPoint = true;
			}
			else if(intersect(goalLines[1], goalToRobot, toGo))
			{
					pt checkPoint;
					if(intersect(goalLines[2], goalToRobot, checkPoint))
					{
							double distance1 = sqrt(double((checkPoint.x - ball.globalPos.X) * (checkPoint.x - ball.globalPos.X) + (checkPoint.y - ball.globalPos.Y) * (checkPoint.y - ball.globalPos.Y)));
							double distance2 = sqrt(double((toGo.x - ball.globalPos.X) * (toGo.x - ball.globalPos.X) + (toGo.y - ball.globalPos.Y) * (toGo.y - ball.globalPos.Y)));        
							if(distance2 > distance1)
									toGo = checkPoint; 
					}
					foundToGoPoint = true;
			}
			else if(intersect(goalLines[2], goalToRobot, toGo))
			{
					foundToGoPoint = true;
			}
			else if(intersect(goalLines[3], goalToRobot, toGo))
			{
					pt checkPoint;
					if(intersect(goalLines[4], goalToRobot, checkPoint))
					{
							double distance1 = sqrt(double((checkPoint.x - ball.globalPos.X) * (checkPoint.x - ball.globalPos.X) + (checkPoint.y - ball.globalPos.Y) * (checkPoint.y - ball.globalPos.Y)));
							double distance2 = sqrt(double((toGo.x - ball.globalPos.X) * (toGo.x - ball.globalPos.X) + (toGo.y - ball.globalPos.Y) * (toGo.y - ball.globalPos.Y)));        
					    if(distance2 > distance1)
					        toGo = checkPoint; 
					}
					foundToGoPoint = true;
			}
			else if(intersect(goalLines[4], goalToRobot, toGo))
			{
					foundToGoPoint = true;
			}
		}
		else
		{
//			 toGo.x = 0;
//			 toGo.y = -60;
			 foundToGoPoint = true;
		}

//    if(!foundToGoPoint)
//    {
//        double distance = sqrt(double((goalPoints[0].x - robot.camera.ball.X) * (goalPoints[0].x - robot.camera.ball.X) + (goalPoints[0].y - robot.camera.ball.Y) * (goalPoints[0].y - robot.camera.ball.Y)));

//        for(int i = 1; i < 6; i++)
//        {
//            //distance1 = sqrt(double((goalLines[i]. - robot.camera.ball.X) * (checkPoint.x - robot.camera.ball.X) + (checkPoint.y - robot.camera.ball.Y) * (checkPoint.y - robot.camera.ball.Y)));
//        }
//    }

		
		if(robot.playState())
		{
			check = basicFunc.genVTMGlobalPoint(make_pair(toGo.x, toGo.y), robot.getPos(), 1)._angle;
			basicFunc.move2(basicFunc.genVTMGlobalPoint(make_pair(toGo.x, toGo.y), robot.getPos(), 0.8), 0);//-atan2(camBall.pos.X, camBall.pos.Y)*57.3);
		}
		
		//TEST AREA-------------------------------------------------------------
		
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
				robot.display.print("Ball x: ", 3, 1);
				robot.display.print(ball.globalPos.X, 3, 11);
				robot.display.print("Ball y: ", 4, 1);
				robot.display.print(ball.globalPos.Y, 4, 11);
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
