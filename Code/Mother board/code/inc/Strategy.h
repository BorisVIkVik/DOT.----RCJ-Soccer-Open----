//#include <FunctionalClass.h>

//class Functional:  public BaseFunctional 
//{
//	public: 
//		int32_t kickTime;
//		int32_t trajectoryTime;
//		int8_t state;
//		Functional(Robot* RC):BaseFunctional(RC)
//		{
//			kickTime = 0;
//			state = 0;
//			trajectoryTime = 0;
//		}
//		void strategy1()
//		{
//		int32_t goalYellowX = ((_RC->target.yGoalY * 2) - 130);
//		int32_t goalYellowY = ((_RC->target.yGoalX * 2) - 130);
//			if(_RC->playState())		//playing
//			{
//				//if((abs(double((robot.target.ballY * 2) - 130 - checlBallx)) < 60 && abs(double((robot.target.ballX * 2) - 130 - checlBally)) < 60) || t - checlTime > 1000)
//				//{
//				//VectorToMove vectres(0, 1);
//					int32_t checlBallx = ((_RC->target.ballY * 2) - 130) / 2;
//					int32_t checlBally = ((_RC->target.ballX * 2) - 130) / 2;
//				//	checlTime = t;
//			//	}
//				//cringe = basicFunc.genATMPoint(checlBallx, checlBally, 1)._angle;
//				//vector<Obstacle> ryadCringa(0, {});
//				//cringe = basicFunc.genATMVecField(checlBallx, checlBally)._angle;
//				double avatarAngToBall = 90 - atan2(double(checlBally), double(checlBallx)) * 57.3;
//			//	basicFunc.turnCoord(-90, 0, 0, checlBallx, checlBally);
//				//checkX = checlBallx;
//				//checkY = checlBally;
//				if(state == 0)
//				{
//					if(abs(double(checlBallx)) < 8.0 && abs(double(checlBally)) < 2.0)
//					{
//						_RC->move(0.5, (checlBallx > 0 ? 1 : -1) * 90, 90);
//					}
//					else
//					{
//						move2(genATMVecField(checlBallx, checlBally), 90);
//					}
//					
//					if(abs(double(checlBallx)) < 20 && abs(double(checlBally)) < 20)
//					{
//						_RC->driblerSpeed1 = -22;
//					}
//					else
//					{
//						_RC->driblerSpeed1 = 0;
//					}
//					if(_RC->ball[0])// || robot.ball[1])
//					{
//						state = 1;
//					}
//				}
//				else if (state == 1)
//				{
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
//				}
//				else if(state == 2)
//				{
//					_RC->driblerSpeed1 = -22;
//					_RC->move(0.5, 0, 90);
//					if(abs(double(goalYellowY)) < 40)
//					{
//						state = 3;
//						kickTime = millis();
//					}
//				}
//				else if(state == 3)
//				{
//					_RC->driblerSpeed1 = -22;
//					_RC->setMotors(100, 100, 100, 100);
//					//int32_t C = robot.target.yGoalY * robot.target.yGoalX - robot.target.yGoalX * robot.target.yGoalY;
//					if(abs(atan2(double(goalYellowX), double(goalYellowY)) * 57.3 + _RC->angle * 1.0 + 180) < 5.0)// || millis() - kickTime > 800)
//					//if(abs(double((-robot.target.yGoalY * checlBallx) + (robot.target.yGoalX * checlBally) /*+ C*/)) < 5)
//					{
//						state = 4;
//						kickTime = millis();
//					}
//					
//				}
//				else if(state == 4)
//				{
//					_RC->driblerSpeed1 = 0;
//					_RC->setMotors(0, 0, 0, 0);
//					if(millis() - kickTime > 7)
//					{
//						//robot.kick1 = true;
//						//robot.kick2 = true;
//						kickTime = millis();
//						//for(int i = 0; i < 1000; i++);
//						state = 5;
//					}
//				}
//				else if(state == 5)
//				{
//					if(millis() - kickTime > 1)
//					{
//						_RC->kick1 = true;
//						_RC->kick2 = true;
//						kickTime = millis();
//						//for(int i = 0; i < 1000; i++);
//						state = 0;
//					}
//				}
//				
//				//robot.move(1, -90);
//				
////				setPin(PE14, 1);
//				//for(int i = 0; i < 200000000; i++){}
//				//setPin(PE1, 0);
//				//for(int i = 0; i < 200000000; i++){}
////				setPin(LED_3, 0);
////				for(int i = 0; i < 10000000; i++){}
////				setPin(LED_1, 0);
////				setPin(LED_2, 1);
////				for(int i = 0; i < 10000000; i++){}
////				setPin(LED_2, 0);
////				setPin(LED_3, 1);
////				for(int i = 0; i < 10000000; i++){}
////				robot.motors[4].setVelocity(100);//Ponyat cho ne tak
////				//robot.move(robot.target.vel, robot.target.dir, robot.target.heading, robot.target.acc);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			}
//			else
//			{
//				_RC->driblerSpeed1 = 0;
//			}
//		}
//		
//		
//		
//		
//		
//		
//		///
//		void strategy2()
//		{
//		//	setPin(LED_3, _RC.playState());
//		
////		_RC.imu.updateAnglesFromFIFO();
//		int32_t goalYellowX = ((_RC->target.yGoalY * 2) - 130);
//		int32_t goalYellowY = ((_RC->target.yGoalX * 2) - 130);
//		//robot.driblerSpeed1 = -10;
//		//robot.setMotors(100, 100, 100, 100);
//	//	robot.move(1, 0);
////		if( !((GLOBAL_ERROR & VIM_CONNECTION_ERROR) || (GLOBAL_ERROR & VIM_DATA_ERROR) || (GLOBAL_ERROR & LOW_BATTERY_POWER)) )		//if everything ok
////		{
//			if(_RC->playState())		//playing
//			{
//				//if((abs(double((robot.target.ballY * 2) - 130 - checlBallx)) < 60 && abs(double((robot.target.ballX * 2) - 130 - checlBally)) < 60) || t - checlTime > 1000)
//				//{
//				//VectorToMove vectres(0, 1);
//					int32_t checlBallx = ((_RC->target.ballY * 2) - 130) / 2;
//					int32_t checlBally = ((_RC->target.ballX * 2) - 130) / 2;
//				//	checlTime = t;
//			//	}
//				//cringe = basicFunc.genATMPoint(checlBallx, checlBally, 1)._angle;
//				//vector<Obstacle> ryadCringa(0, {});
//				//cringe = basicFunc.genATMVecField(checlBallx, checlBally)._angle;
//				double avatarAngToBall = 90 - atan2(double(checlBally), double(checlBallx)) * 57.3;
//			//	basicFunc.turnCoord(-90, 0, 0, checlBallx, checlBally);
//				//checkX = checlBallx;
//				//checkY = checlBally;
//				if(state == 0)
//				{
//					if(abs(double(checlBallx)) < 8.0 && abs(double(checlBally)) < 2.0)
//					{
//						_RC->move(0.5, (checlBallx > 0 ? 1 : -1) * 90, -90);
//					}
//					else
//					{
//						move2(genATMVecField(checlBallx, checlBally), -90);
//					}
//					
//					if(abs(double(checlBallx)) < 20 && abs(double(checlBally)) < 20)
//					{
//						_RC->driblerSpeed1 = -22;
//					}
//					else
//					{
//						_RC->driblerSpeed1 = 0;
//					}
//					if(_RC->ball[0])// || robot.ball[1])
//					{
//						state = 1;
//						trajectoryTime = millis();
//					}
//				}
//				else if (state == 1)
//				{
//					double flex = 1.0*(double(millis()) - double(trajectoryTime))/30.0;
//					if (flex < 98.0)
//					{
//						flex = -1/(flex - 100);
//						flex *= 2;
//						if(_RC->line > 4)
//							setMoveCoords(goalYellowX, DEFAULT_MOVE_Y);
//					}
//					else if(flex < 120.0)
//					{
//						//flex = flex;
//						flex = (flex - 97);
//						//flex *= 2;
//					}
//					else
//					{
//						kickTime = millis();
//						state = 3;
//					}
//					//flex = 1.0*(double(millis()) - double(trajectoryTime))/1000.0 + 1;
////					if((double(millis()) - double(trajectoryTime))/1000.0 < 4)
////					{
////						flex = 1.0 * (double(millis()) - double(trajectoryTime)) / 1000.0 + 1;
////					}
////					else
////					{
////						flex = 1.0 * (double(millis()) - double(trajectoryTime)) / 2.0 + 1;
////					}
////					if(flex > 2550)
////					{
////						state = 3;
////						kickTime = millis();
////					}
//					
//					//if(flex > 2500) flex = 2500;
//					move2(trajectoryFollowing(flex, goalYellowX, goalYellowY), -90);
//					printUART(DEBUG_UART, flex);
//					
//						
//						
////					_RC->driblerSpeed1 = -22;
////					if(_RC->target.yGoalY > 0)
////					{
////						_RC->move(0.5, 90, 90);
////					}
////					else
////					{
////						_RC->move(0.5, -90, 90);
////					}
////					if(_RC->line > 2)
////					{
////						state = 2;
////					}
//				}
//				else if(state == 2)
//				{
//					_RC->driblerSpeed1 = -22;
//					_RC->move(0.5, 0, 90);
//					if(abs(double(goalYellowY)) < 40)
//					{
//						state = 3;
//						kickTime = millis();
//					}
//				}
//				else if(state == 3)
//				{
//					_RC->driblerSpeed1 = -22;
//					_RC->setMotors(-100, -100, -100, -100);
//					//int32_t C = robot.target.yGoalY * robot.target.yGoalX - robot.target.yGoalX * robot.target.yGoalY;
//					if(/*abs(atan2(double(goalYellowX), double(goalYellowY)) * 57.3 + _RC->angle * -1.0) < 5.0)// */millis() - kickTime > 800)
//					//if(abs(double((-robot.target.yGoalY * checlBallx) + (robot.target.yGoalX * checlBally) /*+ C*/)) < 5)
//					{
//						state = 4;
//						kickTime = millis();
//					}
//					
//				}
//				else if(state == 4)
//				{
//					_RC->driblerSpeed1 = 0;
//					_RC->setMotors(0, 0, 0, 0);
//					if(millis() - kickTime > 7)
//					{
//						//robot.kick1 = true;
//						//robot.kick2 = true;
//						kickTime = millis();
//						//for(int i = 0; i < 1000; i++);
//						state = 5;
//					}
//				}
//				else if(state == 5)
//				{
//					if(millis() - kickTime > 1)
//					{
//						_RC->kick1 = true;
//						_RC->kick2 = true;
//						kickTime = millis();
//						//for(int i = 0; i < 1000; i++);
//						state = 0;
//					}
//				}
//				
//				//robot.move(1, -90);
//				
////				setPin(PE14, 1);
//				//for(int i = 0; i < 200000000; i++){}
//				//setPin(PE1, 0);
//				//for(int i = 0; i < 200000000; i++){}
////				setPin(LED_3, 0);
////				for(int i = 0; i < 10000000; i++){}
////				setPin(LED_1, 0);
////				setPin(LED_2, 1);
////				for(int i = 0; i < 10000000; i++){}
////				setPin(LED_2, 0);
////				setPin(LED_3, 1);
////				for(int i = 0; i < 10000000; i++){}
////				robot.motors[4].setVelocity(100);//Ponyat cho ne tak
////				//robot.move(robot.target.vel, robot.target.dir, robot.target.heading, robot.target.acc);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			}
//			else
//			{
//				_RC->driblerSpeed1 = 0;
//			}
//		}
//		///
//};
