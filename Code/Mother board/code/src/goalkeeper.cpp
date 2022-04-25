//sysStart();
//	robot.init();
//	setupScreens();
//	
//	writeStrUART(DEBUG_UART, "\r\nStart\r\n");
//	
//	uint32_t notGameScreenTimer = millis();

//	uint32_t time = millis();
//	uint32_t oldTime = time;
//	uint16_t dt = time - oldTime;
//	
//	
//	pair<int, int> zero;
//	zero.X = 0;
//	zero.Y = 0;
//	
//	PairSaver robotAngle, robotVelocity;
//	pair<double, double> robotA, robotV, ballV;
//	pair<double, double> robotGlobalPos;

//	CameraObject camYellow, camBlue, camBall;
//	FieldObject ball;
//	
//	volatile int check = 0;
//	robot.motorDrivers.disableMotors();
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	pt toGo = {0, 0};
//	
//	PairSaver ballPosSave;
//	pair<double, double> old = make_pair(0,0);
//	uint32_t predictTime = 0;
//	uint32_t strikeTime = 0;
//	uint32_t stateTime = 0;
//	uint32_t angleCheckTest = 0;
//	bool strike = false;
//	while(1)
//	{

//		robot.wait(5);
//		time = millis();
//		dt = time - oldTime;
//		oldTime = time;

//		robotAngle.add(make_pair(robot.imu.getAngle(), 0), time);
//		robotVelocity.add(robot.getV(), time);
//		robotA = robotAngle.pop(time - CAMERA_LATENCY);
//		robotV = robotVelocity.pop(time - CAMERA_LATENCY);
//		robotV.X *= 100;
//		robotV.Y *= 100;
//		
//		ballV = ball.speedSaver.pop(time - CAMERA_LATENCY);

//		camYellow.pos = rotate(robot.camera.yellow, -robotA.first);
//		camBlue.pos = rotate(robot.camera.blue, -robotA.first);
//		camBall.pos = rotate(robot.camera.ball, -robotA.first);

//		#ifdef KALMAN
//		updateKalman(camYellow, KALMAN_K, zero, robotV, dt);
//		updateKalman(camBlue, KALMAN_K, zero, robotV, dt);
//		updateKalman(camBall, KALMAN_K, ballV, robotV, dt);
//		#endif
//		#ifdef PREDICTION
//		updatePrediction(camYellow, zero, robotV, CAMERA_LATENCY);
//		updatePrediction(camBlue, zero, robotV, CAMERA_LATENCY);
//		updatePrediction(camBall, ballV, robotV, CAMERA_LATENCY);
//		#endif

//		robot.updateSelfPos(camYellow.pos, camBlue.pos);
//		robotGlobalPos = robot.getPos();
//		ball.update(camBall.pos, robotGlobalPos, time, SPEED_CALC_TIME);
//		
//		ballPosSave.add(ball.globalPos, time);
//	/*
//		check = ball.globalPos.X;
//		if(robot.playState())
//		{
//			int xTmp = ball.globalPos.X - robotGlobalPos.X;
//			int yTmp = robotGlobalPos.Y - 70;
//		basicFunc.move2(basicFunc.genATMPoint(xTmp,yTmp,min2(2.0, 0.08 * sqrt(double(xTmp * xTmp + yTmp * yTmp)))), 0);
//		robot.wait(1);
//		}
//		else
//		{
//			robot.motorDrivers.setMotors(0,0,0,0);
//		}
//	*/
//	line goalToRobot;
//	int lineA = 0;
//	int lineB = 0;
//	int lineC = 0;
//	//TEST AREA-------------------------------------------------------------
//	int timeBack = 200;
//	if(millis() - predictTime > 200)
//	{
//		old = ball.globalPos;
//		predictTime = millis();
//	}
//	if(distanceVec(old, ball.globalPos) > 5 && !strike)
//	{
//		strikeTime = millis();
//		strike = false;
////		lineA = 0 + 20;
////		lineB = 22 - 21;
////		lineC = 22 * 20;
//		//A = (y2 - y1); B = (x1 - x2); C = y1 * (x2 - x1) - (y2 - y1) * x1
////		lineA = (ball.globalPos.Y - ballPosSave.pop(time - timeBack).Y);
//	lineA = (ball.globalPos.Y - old.Y);
//	lineB = (old.X - ball.globalPos.X);
//	lineC = old.Y * (ball.globalPos.X - old.X) - (ball.globalPos.Y - old.Y) * old.X;
//	}
////		lineB = (ballPosSave.pop(time - timeBack).X - ball.globalPos.X);
////		lineC = ballPosSave.pop(time - timeBack).Y * (ball.globalPos.X - ballPosSave.pop(time - timeBack).X) - (ball.globalPos.Y - ballPosSave.pop(time - timeBack).Y) * ballPosSave.pop(time - timeBack).X;
////	}
//	else
//	{
//		if(millis() - strikeTime > 5000 && !strike)
//		{
//			strikeTime = millis();
//			strike = true;
//		}
//		lineA = (ball.globalPos.Y + 97);
//		lineB = (0 - ball.globalPos.X);
//		lineC = (-97 * ball.globalPos.X);
//	}

//	
//	goalToRobot.a = lineA;
//	goalToRobot.b = lineB;
//	goalToRobot.c = lineC;
//	
//	bool foundToGoPoint = false;
//  
//		if(abs(ball.globalPos.Y) < 75)
//		{

//			
//			if(intersect(goalLines[0], goalToRobot, toGo))
//			{
//					foundToGoPoint = true;
//			}
//			else if(intersect(goalLines[1], goalToRobot, toGo))
//			{
//					pt checkPoint;
//					if(intersect(goalLines[2], goalToRobot, checkPoint))
//					{
//							double distance1 = sqrt(double((checkPoint.x - ball.globalPos.X) * (checkPoint.x - ball.globalPos.X) + (checkPoint.y - ball.globalPos.Y) * (checkPoint.y - ball.globalPos.Y)));
//							double distance2 = sqrt(double((toGo.x - ball.globalPos.X) * (toGo.x - ball.globalPos.X) + (toGo.y - ball.globalPos.Y) * (toGo.y - ball.globalPos.Y)));        
//							if(distance2 > distance1)
//									toGo = checkPoint; 
//					}
//					foundToGoPoint = true;
//			}
//			else if(intersect(goalLines[2], goalToRobot, toGo))
//			{
//					foundToGoPoint = true;
//			}
//			else if(intersect(goalLines[3], goalToRobot, toGo))
//			{
//					pt checkPoint;
//					if(intersect(goalLines[4], goalToRobot, checkPoint))
//					{
//							double distance1 = sqrt(double((checkPoint.x - ball.globalPos.X) * (checkPoint.x - ball.globalPos.X) + (checkPoint.y - ball.globalPos.Y) * (checkPoint.y - ball.globalPos.Y)));
//							double distance2 = sqrt(double((toGo.x - ball.globalPos.X) * (toGo.x - ball.globalPos.X) + (toGo.y - ball.globalPos.Y) * (toGo.y - ball.globalPos.Y)));        
//					    if(distance2 > distance1)
//					        toGo = checkPoint; 
//					}
//					foundToGoPoint = true;
//			}
//			else if(intersect(goalLines[4], goalToRobot, toGo))
//			{
//					foundToGoPoint = true;
//			}
//		}
//		else
//		{
////			 toGo.x = 0;
////			 toGo.y = -60;
//			 foundToGoPoint = true;
//		}

////    if(!foundToGoPoint)
////    {
////        double distance = sqrt(double((goalPoints[0].x - robot.camera.ball.X) * (goalPoints[0].x - robot.camera.ball.X) + (goalPoints[0].y - robot.camera.ball.Y) * (goalPoints[0].y - robot.camera.ball.Y)));

////        for(int i = 1; i < 6; i++)
////        {
////            //distance1 = sqrt(double((goalLines[i]. - robot.camera.ball.X) * (checkPoint.x - robot.camera.ball.X) + (checkPoint.y - robot.camera.ball.Y) * (checkPoint.y - robot.camera.ball.Y)));
////        }
////    }

////		if(millis() - stateTime > 3000)
////		{ 
////			stateTime = millis();
////			angleCheckTest += 90;
////			angleCheckTest %= 360;
////			if(angleCheckTest == 90)
////			{
////				toGo.x = 20;
////				toGo.y = -40;
////			}
////			else if (angleCheckTest == 180)
////			{
////				toGo.x = 20;
////				toGo.y = 40;
////			}
////			else if(angleCheckTest == 270)
////			{
////				toGo.x = -20;
////				toGo.y = 40;
////			}
////			else if (angleCheckTest == 0)
////			{
////				toGo.x = -20;
////				toGo.y = -40;
////			}
////		}
//		if(strike)
//		{
//			toGo.x = ball.globalPos.X;
//			toGo.y = ball.globalPos.Y;
//			if(millis() - strikeTime > 3000 || !basicFunc.checkBounds(make_pair(-30, -80), make_pair(30, 0), robot.getPos()))
//			{
//				strikeTime = millis();
//				strike = false;
//			}
//		}
//		if(robot.playState())
//		{
//			check = basicFunc.genVTMGlobalPoint(make_pair(toGo.x, toGo.y), robot.getPos(), 1)._angle;
//			//robot.move(0.8, angleCheckTest);
//			basicFunc.move2(basicFunc.genVTMGlobalPoint(make_pair(toGo.x, toGo.y), robot.getPos(), 2.5), 0);//-atan2(camBall.pos.X, camBall.pos.Y)*57.3);
//		}
//		
//		//TEST AREA-------------------------------------------------------------