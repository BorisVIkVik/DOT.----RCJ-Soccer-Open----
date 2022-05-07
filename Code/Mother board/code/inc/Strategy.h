#pragma once

#include <CameraObject.h>
#include <FieldObject.h>
#include <FunctionalClass.h>
#include <Border.h>
//#include <VTM.h>

#define SPEED_TRAJ_FOLLOW_M_S									1.0
#define SPEED_TRAJ_FOLLOW_CM_MILLIS						SPEED_TRAJ_FOLLOW_M_S * 0.1 

Border b1(1.0, 'x', '-', -23, -28, -90, 90);
Border b2(1.0, 'x', '+', 23, 28, -90, 90);
Border b3(1.0, 'y', '-', -50, -55, -30, 30);
Border b4(1.0, 'y', '+', 50, 55, -30, 30);

//Border b5('x', '+', -20, -15, -90, -60);
//Border b6('x', '-', 20, 15, -90, -60);
//Border b7('x', '+', -20, -15, 60, 90);
//Border b8('x', '-', 20, 15, 60, 90);

//Border b9('y', '-', -80, -90, -35, -20);
//Border b10('y', '-', -80, -90, 20, 35);
//Border b11('y', '+', 80, 90, -35, -20);
//Border b12('y', '+', 80, 90, 20, 35);

//pt goalPoints[6] = {{70, -97}, {70, -89}, {55, -74}, {-55, -74}, {-70, -89}, {-70, -97}};
segment goalLines[5] = {{0, 1, 40, -18, 18, 0, 0}, {2, 5, 164, 18, 30, 0, 0}, {-2, 5, 164, -30, -18, 0, 0}, {1, 0, 30, 0, 0, -75, -44.8}, {1, 0, -30, 0, 0, -75, -44.8}};

class Functional:  public BaseFunctional 
{
	public: 
		int32_t kickTime;
		FieldObject ball;
		int32_t trajectoryTime;
		int8_t state;
		Functional(Robot* RC):BaseFunctional(RC)
		{
			kickTime = 0;
			state = 0;
			trajectoryTime = 0;
			attackerStopTime = 0;
			oldPosIndex = 0;
			attackerStop = false;
			side = 'r';
			lost = 0;
			got = 0;
			///-------------------------
			old = make_pair(0,0);
			predictTime = 0;
			strikeTime = 0;
			stateTime = 0;
			angleCheckTest = 0;
			strike = false;
		}
		
		
		void posCalc()
		{
			time = millis();
			dt = time - oldTime;
			oldTime = time;

			robotAngle.add(make_pair(getRobotClass()->imu.getAngle(), 0), time);
			robotVelocity.add(getRobotClass()->getV(), time);
			robotA = robotAngle.pop(time - CAMERA_LATENCY);
			robotV = robotVelocity.pop(time - CAMERA_LATENCY);
			robotV.X *= 100;
			robotV.Y *= 100;
			
			ballV = ball.speedSaver.pop(time - CAMERA_LATENCY);
			/*
			camYellow.pos = rotate(getRobotClass()->camera.yellow, 0);
			camBlue.pos = rotate(getRobotClass()->camera.blue, 0);
			camBall.pos = rotate(getRobotClass()->camera.ball, 0);
			*/
			/*
			camYellow.pos = rotate(getRobotClass()->camera.yellow, -getRobotClass()->imu.getAngle());
			camBlue.pos = rotate(getRobotClass()->camera.blue, -getRobotClass()->imu.getAngle());
			camBall.pos = rotate(getRobotClass()->camera.ball, -getRobotClass()->imu.getAngle());
			*/
			
			camYellow.pos = rotate(getRobotClass()->camera.yellow, -robotA.first);
			camBlue.pos = rotate(getRobotClass()->camera.blue, -robotA.first);
			camBall.pos = rotate(getRobotClass()->camera.ball, -robotA.first);
			
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

			getRobotClass()->updateSelfPos(camYellow.pos, camBlue.pos);
			robotGlobalPos = getRobotClass()->getPos();
			ball.update(camBall.pos, robotGlobalPos, time, SPEED_CALC_TIME);
			
			ballPosSave.add(ball.globalPos, time);
			
			//getRobotClass()->display.print("RAZ:", 1, 1);
			//getRobotClass()->display.print(getRobotClass()->imu.getAngle() - robotA.X, 1, 7);

		}
		void strategy1()
		{
		//int32_t goalYellowX = ((getRobotClass()->target.yGoalY * 2) - 130);
		//int32_t goalYellowY = ((getRobotClass()->target.yGoalX * 2) - 130);
			if(getRobotClass()->playState())		//playing
			{
				//if((abs(double((robot.target.ballY * 2) - 130 - checlBallx)) < 60 && abs(double((robot.target.ballX * 2) - 130 - checlBally)) < 60) || t - checlTime > 1000)
				//{
				//VectorToMove vectres(0, 1);
				//	int32_t checlBallx = ((getRobotClass()->target.ballY * 2) - 130) / 2;
				//	int32_t checlBally = ((getRobotClass()->target.ballX * 2) - 130) / 2;
				//	checlTime = t;
			//	}
				//cringe = basicFunc.genATMPoint(checlBallx, checlBally, 1)._angle;
				//vector<Obstacle> ryadCringa(0, {});
				//cringe = basicFunc.genATMVecField(checlBallx, checlBally)._angle;
				double avatarAngToBall = 90 - atan2(double(camBall.pos.Y), double(camBall.pos.X)) * 57.3;
			//	basicFunc.turnCoord(-90, 0, 0, checlBallx, checlBally);
				//checkX = checlBallx;
				//checkY = checlBally;
				if(state == 0)
				{
					if(abs(double(camBall.pos.X)) < 8.0 && abs(double(camBall.pos.Y)) < 2.0)
					{
						getRobotClass()->move(0.5, (camBall.pos.X > 0 ? 1 : -1) * 90, 90);
					}
					else
					{
						move2(genATMVecField(camBall.pos.X, camBall.pos.Y), 90);
					}
					
					if(abs(double(camBall.pos.X)) < 20 && abs(double(camBall.pos.Y)) < 20)
					{
						getRobotClass()->motorDrivers.setMotor(4, -22);
					}
					else
					{
						getRobotClass()->motorDrivers.setMotor(4, 0);
					}
					if(getRobotClass()->ballSensor.getValue())// || robot.ball[1])
					{
						state = 1;
					}
				}
				else if (state == 1)
				{
					getRobotClass()->motorDrivers.setMotor(4, -22);
					if(robotGlobalPos.X > 0)
					{
						getRobotClass()->move(0.5, 90, 90);
					}
					else
					{
						getRobotClass()->move(0.5, -90, 90);
					}
					if(getRobotClass()->lineSensors.getLine() > 2)
					{
						state = 2;
					}
				}
				else if(state == 2)
				{
					getRobotClass()->motorDrivers.setMotor(4, -22);
					getRobotClass()->move(0.5, 0, 90);
					if(robotGlobalPos.Y > 45)
					{
						state = 3;
						kickTime = millis();
					}
				}
				else if(state == 3)
				{
					getRobotClass()->motorDrivers.setMotor(4, -22);
					getRobotClass()->motorDrivers.setMotors(100, 100, 100, 100);
					//int32_t C = robot.target.yGoalY * robot.target.yGoalX - robot.target.yGoalX * robot.target.yGoalY;
					if(abs(atan2(double(camYellow.pos.X), double(camYellow.pos.Y)) * 57.3 + getRobotClass()->imu.getAngle() * 1.0 + 180) < 5.0)// || millis() - kickTime > 800)
					//if(abs(double((-robot.target.yGoalY * checlBallx) + (robot.target.yGoalX * checlBally) /*+ C*/)) < 5)
					{
						state = 4;
						kickTime = millis();
					}
					
				}
				else if(state == 4)
				{
					getRobotClass()->motorDrivers.setMotor(4, 0);
					getRobotClass()->motorDrivers.setMotors(0, 0, 0, 0);
					if(millis() - kickTime > 7)
					{
						//robot.kick1 = true;
						//robot.kick2 = true;
						kickTime = millis();
						//for(int i = 0; i < 1000; i++);
						state = 5;
					}
				}
				else if(state == 5)
				{
					if(millis() - kickTime > 1)
					{
						getRobotClass()->kickerModule.kick(true, true);
						kickTime = millis();
						//for(int i = 0; i < 1000; i++);
						state = 0;
					}
				}
				
				//robot.move(1, -90);
				
//				setPin(PE14, 1);
				//for(int i = 0; i < 200000000; i++){}
				//setPin(PE1, 0);
				//for(int i = 0; i < 200000000; i++){}
//				setPin(LED_3, 0);
//				for(int i = 0; i < 10000000; i++){}
//				setPin(LED_1, 0);
//				setPin(LED_2, 1);
//				for(int i = 0; i < 10000000; i++){}
//				setPin(LED_2, 0);
//				setPin(LED_3, 1);
//				for(int i = 0; i < 10000000; i++){}
//				robot.motors[4].setVelocity(100);//Ponyat cho ne tak
//				//robot.move(robot.target.vel, robot.target.dir, robot.target.heading, robot.target.acc);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			}
			else
			{
				getRobotClass()->motorDrivers.setMotor(4, 0);
			}
		}
		
		
		void testBorder()
		{
			if(getRobotClass()->playState())	
			{
			VectorToMove res(0,0,0);
			res = genVTMGlobalPoint(ball.globalPos, getRobotClass()->getPos(), 2.0);
			//getRobotClass()->move(1, 90);
			b1.dempher(getRobotClass()->getPos(), res);
			b2.dempher(getRobotClass()->getPos(), res);
			b3.dempher(getRobotClass()->getPos(), res);
			b4.dempher(getRobotClass()->getPos(), res);
//			b5.dempher(getRobotClass()->getPos(), res);
//			b6.dempher(getRobotClass()->getPos(), res);
//			b7.dempher(getRobotClass()->getPos(), res);
//			b8.dempher(getRobotClass()->getPos(), res);
//			b9.dempher(getRobotClass()->getPos(), res);
//			b10.dempher(getRobotClass()->getPos(), res);
//			b11.dempher(getRobotClass()->getPos(), res);
//			b12.dempher(getRobotClass()->getPos(), res);
			move2(res, 0);
			}
			else
			{
				getRobotClass()->motorDrivers.setMotors(0,0,0,0,0);
			}
			
		}
		
		
		
		///
		void strategy2()
		{
			if(millis() - attackerStopTime > 500)
			{
				attackerStop = true;
				state = 0;
			}
			if(getRobotClass()->ballSensor.getValue() || (getRobotClass()->camera.objects & 1))
			{
				
				attackerStop = false;
				attackerStopTime = millis();
			}
			if(getRobotClass()->playState() && !attackerStop)		//playing
			{
				
				double avatarAngToBall = 0;
//				if(getRobotClass()->camera.objects & 1)
				avatarAngToBall = -atan2(double(camBall.pos.X), double(camBall.pos.Y)) * 57.3;
				if(state == 0)
				{
					getRobotClass()->motorDrivers.setMotor(4, -100);
					VectorToMove res(0,0,0);
					if((abs(double(camBall.pos.X)) < 30.0 && abs(double(camBall.pos.Y)) < 30.0))
					{
						res = genVTMGlobalPoint(ball.globalPos, getRobotClass()->getPos(), 0.3);
					}
					else
					{
						res = Parabola(ball.globalPos, getRobotClass()->getPos(), 0.9);
					}
					b1.dempher(getRobotClass()->getPos(), res);
					b2.dempher(getRobotClass()->getPos(), res);
					b3.dempher(getRobotClass()->getPos(), res);
					b4.dempher(getRobotClass()->getPos(), res);
					move2(res, avatarAngToBall);
					
//						VectorToMove res(0,0,0);
//						res = genVTMGlobalPoint(ball.globalPos, getRobotClass()->getPos(), 2.0);
//						b1.dempher(getRobotClass()->getPos(), res);
//						b2.dempher(getRobotClass()->getPos(), res);
//						b3.dempher(getRobotClass()->getPos(), res);
//						b4.dempher(getRobotClass()->getPos(), res);
//						move2(res, avatarAngToBall);//move(0.5, (camBall.pos.X < 0 ? 1 : -1) * 90, -90);
//					}
//					else 
//					{
//						VectorToMove res(0,0,0);
//						res = genATMVecField(camBall.pos.X/2, -camBall.pos.Y/2);
//						b1.dempher(getRobotClass()->getPos(), res);
//						b2.dempher(getRobotClass()->getPos(), res);
//						b3.dempher(getRobotClass()->getPos(), res);
//						b4.dempher(getRobotClass()->getPos(), res);
//						move2(res, avatarAngToBall);//-90);
//					}
//					
					
//					if(abs(double(camBall.pos.X)) < 20 && abs(double(camBall.pos.Y)) < 20)
//					{
//						getRobotClass()->motorDrivers.setMotor(4, -22);
//					}
//					else
//					{
//						getRobotClass()->motorDrivers.setMotor(4, 0);
//					}
					if(getRobotClass()->ballSensor.getValue())// || robot.ball[1])
					{
						if(millis() - got > 1000)
						{
							state = 1;
						}
						getRobotClass()->motorDrivers.setMotors(0,0,0,0);
						
						trajectoryTime = millis();
						if(getRobotClass()->getPos().X > 0)
						{
							side = 'r';
						}
						else
						{
							side = 'l';
						}
						oldPosIndex = findStartOfTrajectory(getRobotClass()->getPos());
					}
					else
					{
						got = millis();
					}
				}
				else if (state == 1)
				{
					getRobotClass()->motorDrivers.setMotor(4, -500);
					double avatarAngToGoalBlue = 180-atan2(double(camBlue.pos.X), double(camBlue.pos.Y)) * 57.3;
					double flex = (millis() - trajectoryTime) * SPEED_TRAJ_FOLLOW_CM_MILLIS * 0.01;
					if(oldPosIndex > 190 && checkBounds(make_pair(-25, 55), make_pair(25, 70), getRobotClass()->getPos()))
					{
						kickTime = millis();
						state = 2;
					}
					
					if(getRobotClass()->ballSensor.getValue())
					{
						lost = millis();
					}
					
					if(millis() - lost > 500)
					{
						state = 0;
						lost = millis();
					}
					
					if(side == 'r')
						move2(trajectoryFollowingDots(oldPosIndex, flex, side), 180 -atan2(double(camBlue.pos.X), double(camBlue.pos.Y)) * 57.3);
					else
						move2(trajectoryFollowingDots(oldPosIndex, flex, side), 180 -atan2(double(camBlue.pos.X), double(camBlue.pos.Y)) * 57.3);
						
					printUART(DEBUG_UART, flex);
				}
				else if(state == 2)
				{
					double avatarAngToGoalBlue = -atan2(double(camBlue.pos.X), double(camBlue.pos.Y)) * 57.3;
					getRobotClass()->move(0, 0, avatarAngToGoalBlue);
					//getRobotClass()->motorDrivers.setMotors(-100, -100, -100, -100);
//					getRobotClass()->motorDrivers.setMotor(4, -22);
//					getRobotClass()->move(0.5, 0, 90);
//					if(robotGlobalPos.Y > 45)
//					{
//						state = 3;
//						kickTime = millis();
//					}
				}
				else if(state == 3)
				{
					getRobotClass()->motorDrivers.setMotor(4, -22);
					getRobotClass()->motorDrivers.setMotors(-100, -100, -100, -100);
					//int32_t C = robot.target.yGoalY * robot.target.yGoalX - robot.target.yGoalX * robot.target.yGoalY;
					if(/*abs(atan2(double(goalYellowX), double(goalYellowY)) * 57.3 + _RC->angle * -1.0) < 5.0)// */millis() - kickTime > 800)
					//if(abs(double((-robot.target.yGoalY * checlBallx) + (robot.target.yGoalX * checlBally) /*+ C*/)) < 5)
					{
						state = 4;
						kickTime = millis();
					}
					
				}
				else if(state == 4)
				{
					getRobotClass()->motorDrivers.setMotor(4, 0);
					getRobotClass()->motorDrivers.setMotors(0, 0, 0, 0);
					if(millis() - kickTime > 7)
					{
						kickTime = millis();
						state = 5;
					}
				}
				else if(state == 5)
				{
					if(millis() - kickTime > 1)
					{
						getRobotClass()->kickerModule.kick(true, true);
						kickTime = millis();
						state = 0;
					}
				}
				
			}
			else
			{
				getRobotClass()->motorDrivers.setMotors(0,0,0,0,0);
				//getRobotClass()->motorDrivers.setMotor(4, 0);
			}
		}
		///
		CameraObject camYellow, camBlue, camBall;
		
		
	
	void goalkeeper()
	{
		
		if(millis() - goalkeeperStopTime > 1000)
				goalkeeperStop = true;
			if(getRobotClass()->ballSensor.getValue() || (getRobotClass()->camera.objects & 1))
			{
				goalkeeperStop = false;
				goalkeeperStopTime = millis();
			}
		
		line goalToRobot;
		int lineA = 0;
		int lineB = 0;
		int lineC = 0;
		//TEST AREA-------------------------------------------------------------
		int timeBack = 200;
		if(millis() - predictTime > 200)
		{
			old = ball.globalPos;
			predictTime = millis();
		}
		if(distanceVec(old, ball.globalPos) > 1000 && !strike)
		{
			strikeTime = millis();
			strike = false;
		lineA = (ball.globalPos.Y - old.Y);
		lineB = (old.X - ball.globalPos.X);
		lineC = old.Y * (ball.globalPos.X - old.X) - (ball.globalPos.Y - old.Y) * old.X;
		}
	//		lineB = (ballPosSave.pop(time - timeBack).X - ball.globalPos.X);
	//		lineC = ballPosSave.pop(time - timeBack).Y * (ball.globalPos.X - ballPosSave.pop(time - timeBack).X) - (ball.globalPos.Y - ballPosSave.pop(time - timeBack).Y) * ballPosSave.pop(time - timeBack).X;
	//	}
		else
		{
			if(millis() - strikeTime > 5000 && !strike)
			{
				strikeTime = millis();
				strike = true;
			}
			lineA = (ball.globalPos.Y + 97);
			lineB = (0 - ball.globalPos.X);
			lineC = (-97 * ball.globalPos.X);
		}

		
		goalToRobot.a = lineA;
		goalToRobot.b = lineB;
		goalToRobot.c = lineC;
		
		bool foundToGoPoint = false;
		
			if(abs(ball.globalPos.Y) < 70)
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

	//		if(millis() - stateTime > 3000)
	//		{ 
	//			stateTime = millis();
	//			angleCheckTest += 90;
	//			angleCheckTest %= 360;
	//			if(angleCheckTest == 90)
	//			{
	//				toGo.x = 20;
	//				toGo.y = -40;
	//			}
	//			else if (angleCheckTest == 180)
	//			{
	//				toGo.x = 20;
	//				toGo.y = 40;
	//			}
	//			else if(angleCheckTest == 270)
	//			{
	//				toGo.x = -20;
	//				toGo.y = 40;
	//			}
	//			else if (angleCheckTest == 0)
	//			{
	//				toGo.x = -20;
	//				toGo.y = -40;
	//			}
	//		}
			if(strike)
			{
				toGo.x = ball.globalPos.X;
				toGo.y = ball.globalPos.Y;
				if(millis() - strikeTime > 3000 || !checkBounds(make_pair(-30, -80), make_pair(30, 0), getRobotClass()->getPos()))
				{
					strikeTime = millis();
					strike = false;
				}
			}
			if(getRobotClass()->playState())
			{
				if(!goalkeeperStop)
					move2(genVTMGlobalPoint(make_pair(toGo.x, toGo.y), getRobotClass()->getPos(), 2.0), 0);//-atan2(camBall.pos.X, camBall.pos.Y)*57.3);
				else
					move2(genVTMGlobalPoint(make_pair(0, -40), getRobotClass()->getPos(),2.0), 0);
			}
			else
			{
				getRobotClass()->motorDrivers.setMotors(0,0,0,0,0);
			}
			
			//TEST AREA-------------------------------------------------------------
	}
	private:
		//Attacker
		int16_t oldPosIndex;
		uint32_t time;
		uint32_t dt;
		uint32_t oldTime;
		PairSaver robotAngle, robotVelocity;
		pair<double, double> robotA, robotV, ballV;
		pair<double, double> robotGlobalPos;
		uint32_t attackerStopTime;
		bool attackerStop;
		PairSaver ballPosSave;
		char side;
		uint32_t got;
		uint32_t lost;
		//Attacker
	
		//Goalkeeper
		pt toGo;
		pair<double, double> old;
		uint32_t predictTime;
		uint32_t strikeTime; 
		uint32_t stateTime;
		uint32_t angleCheckTest;
		bool strike;
		uint32_t goalkeeperStopTime;
		bool goalkeeperStop;
		//Goalkeeper
};
	