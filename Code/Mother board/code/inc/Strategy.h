#include <CameraObject.h>
#include <FieldObject.h>
#include <FunctionalClass.h>

#define SPEED_TRAJ_FOLLOW_M_S									1.0
#define SPEED_TRAJ_FOLLOW_CM_MILLIS						SPEED_TRA 

class Functional:  public BaseFunctional 
{
	public: 
		int32_t kickTime;
		int32_t trajectoryTime;
		int8_t state;
		Functional(Robot* RC):BaseFunctional(RC)
		{
			kickTime = 0;
			state = 0;
			trajectoryTime = 0;
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
		
		
		
		
		
		
		///
		void strategy2()
		{
		//	setPin(LED_3, _RC.playState());
		
//		_RC.imu.updateAnglesFromFIFO();
		//int32_t goalYellowX = ((getRobotClass()->target.yGoalY * 2) - 130);
		//int32_t goalYellowY = ((getRobotClass()->target.yGoalX * 2) - 130);
		//robot.driblerSpeed1 = -10;
		//robot.setMotors(100, 100, 100, 100);
	//	robot.move(1, 0);
//		if( !((GLOBAL_ERROR & VIM_CONNECTION_ERROR) || (GLOBAL_ERROR & VIM_DATA_ERROR) || (GLOBAL_ERROR & LOW_BATTERY_POWER)) )		//if everything ok
//		{
			if(getRobotClass()->playState())		//playing
			{
				//if((abs(double((robot.target.ballY * 2) - 130 - checlBallx)) < 60 && abs(double((robot.target.ballX * 2) - 130 - checlBally)) < 60) || t - checlTime > 1000)
				//{
				//VectorToMove vectres(0, 1);
					//int32_t checlBallx = ((getRobotClass()->target.ballY * 2) - 130) / 2;
					//int32_t checlBally = ((getRobotClass()->target.ballX * 2) - 130) / 2;
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
						getRobotClass()->move(0.5, (camBall.pos.X > 0 ? 1 : -1) * 90, -90);
					}
					else
					{
						move2(genATMVecField(camBall.pos.X, camBall.pos.Y), -90);
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
						trajectoryTime = millis();
					}
				}
				else if (state == 1)
				{
					double flex = 1.0*(double(millis()) - double(trajectoryTime))/30.0;
					if (flex < 98.0)
					{
						flex = -1/(flex - 100);
						flex *= 2;
						if(getRobotClass()->lineSensors.getLine() > 4)
							setMoveCoords(goalYellowX, DEFAULT_MOVE_Y);
					}
					else if(flex < 120.0)
					{
						//flex = flex;
						flex = (flex - 97);
						//flex *= 2;
					}
					else
					{
						kickTime = millis();
						state = 3;
					}
					//flex = 1.0*(double(millis()) - double(trajectoryTime))/1000.0 + 1;
//					if((double(millis()) - double(trajectoryTime))/1000.0 < 4)
//					{
//						flex = 1.0 * (double(millis()) - double(trajectoryTime)) / 1000.0 + 1;
//					}
//					else
//					{
//						flex = 1.0 * (double(millis()) - double(trajectoryTime)) / 2.0 + 1;
//					}
//					if(flex > 2550)
//					{
//						state = 3;
//						kickTime = millis();
//					}
					
					//if(flex > 2500) flex = 2500;
					move2(trajectoryFollowing(flex, goalYellowX, goalYellowY), -90);
					printUART(DEBUG_UART, flex);
					
						
						
//					_RC->driblerSpeed1 = -22;
//					if(_RC->target.yGoalY > 0)
//					{
//						_RC->move(0.5, 90, 90);
//					}
//					else
//					{
//						_RC->move(0.5, -90, 90);
//					}
//					if(_RC->line > 2)
//					{
//						state = 2;
//					}
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
		///
	private:
		CameraObject camYellow, camBlue, camBall;
		uint32_t time;
		uint32_t dt;
		uint32_t oldTime;
		PairSaver robotAngle, robotVelocity;
		pair<double, double> robotA, robotV, ballV;
		pair<double, double> robotGlobalPos;
		FieldObject ball;
		PairSaver ballPosSave;
};
	