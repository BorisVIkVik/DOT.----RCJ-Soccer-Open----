#pragma once

#include "Robot.h"
#include <RobotParametrs.h>
//#include <stdint.h>
#include <Obstacle.h>
#include <VTM.h>
#include <VecField.h>
#include <vector>
#include <math.h>
#include <PairSaver.h>

using namespace std;

int16_t vecNum = 10800;
int8_t infNum = 2;
//int8_t** map = new int8_t*[vecNum];




class BaseFunctional
{
    public:
                            BaseFunctional(Robot* RC):_RC(RC){}
        void                dribblerSpeed(int8_t speed);
        //void                initVecField();
        int8_t              lineCheck();
        void                chargeShooter();
        void                shoot(uint8_t side, uint8_t power);
        //RobotParametrs      turnRC(RobotParametrs RP, int8_t angle, int8_t x, int8_t y);
				void					      turnCoord(double angle, int16_t x, int16_t y, int16_t& xtoChange, int16_t& ytoChange);
        //RobotParametrs      setAngle(RobotParametrs RP, int8_t angle);
        bool                checkBall();
        //void                addObstacle(VecField* vF, VecField oF, int8_t x, int8_t y);
        void                move2(VectorToMove vtm, double heading);
        VectorToMove        genATMPoint(int16_t x, int16_t y, int8_t vecMod); 
        VectorToMove        genATMVecField(int16_t x, int16_t y);//, vector<Obstacle> obs);
				
    private:
        Robot* _RC;
};

//class Functional: public BaseFunctional 
//{
//	public: 
//		Functional();
//		void strategy1()
//		{
//			setPin(LED_3, _RC.playState());
//		
//		robot.imu.updateAnglesFromFIFO();
//		goalYellowX = ((robot.target.yGoalY * 2) - 130);
//		goalYellowY = ((robot.target.yGoalX * 2) - 130);
//		//robot.driblerSpeed1 = -10;
//		//robot.setMotors(100, 100, 100, 100);
//	//	robot.move(1, 0);
////		if( !((GLOBAL_ERROR & VIM_CONNECTION_ERROR) || (GLOBAL_ERROR & VIM_DATA_ERROR) || (GLOBAL_ERROR & LOW_BATTERY_POWER)) )		//if everything ok
////		{
//			if(robot.playState())		//playing
//			{
//				//if((abs(double((robot.target.ballY * 2) - 130 - checlBallx)) < 60 && abs(double((robot.target.ballX * 2) - 130 - checlBally)) < 60) || t - checlTime > 1000)
//				//{
//				//VectorToMove vectres(0, 1);
//					checlBallx = ((robot.target.ballY * 2) - 130) / 2;
//					checlBally = ((robot.target.ballX * 2) - 130) / 2;
//				//	checlTime = t;
//			//	}
//				//cringe = basicFunc.genATMPoint(checlBallx, checlBally, 1)._angle;
//				//vector<Obstacle> ryadCringa(0, {});
//				//cringe = basicFunc.genATMVecField(checlBallx, checlBally)._angle;
//				double avatarAngToBall = 90 - atan2(double(checlBally), double(checlBallx)) * 57.3;
//			//	basicFunc.turnCoord(-90, 0, 0, checlBallx, checlBally);
//				checkX = checlBallx;
//				checkY = checlBally;
//				if(state == 0)
//				{
//					if(abs(double(checlBallx)) < 8.0 && abs(double(checlBally)) < 2.0)
//					{
//						robot.move(0.5, (checlBallx > 0 ? 1 : -1) * 90, 90);
//					}
//					else
//					{
//						basicFunc.move2(basicFunc.genATMVecField(checlBallx, checlBally), 90);
//					}
//					
//					if(abs(double(checlBallx)) < 20 && abs(double(checlBally)) < 20)
//					{
//						robot.driblerSpeed1 = -22;
//					}
//					else
//					{
//						robot.driblerSpeed1 = 0;
//					}
//					if(robot.ball[0])// || robot.ball[1])
//					{
//						state = 1;
//					}
//				}
//				else if (state == 1)
//				{
//					robot.driblerSpeed1 = -22;
//					if(robot.target.yGoalY > 0)
//					{
//						robot.move(0.5, 90, 90);
//					}
//					else
//					{
//						robot.move(0.5, -90, 90);
//					}
//					if(robot.line > 2)
//					{
//						state = 2;
//					}
//				}
//				else if(state == 2)
//				{
//					robot.driblerSpeed1 = -22;
//					robot.move(0.5, 0, 90);
//					if(abs(double(goalYellowY)) < 40)
//					{
//						state = 3;
//						kickTime = millis();
//					}
//				}
//				else if(state == 3)
//				{
//					robot.driblerSpeed1 = -22;
//					robot.setMotors(100, 100, 100, 100);
//					//int32_t C = robot.target.yGoalY * robot.target.yGoalX - robot.target.yGoalX * robot.target.yGoalY;
//					if(abs(atan2(double(goalYellowX), double(goalYellowY)) * 57.3 + robot.angle * 1.0 + 180) < 5.0)// || millis() - kickTime > 800)
//					//if(abs(double((-robot.target.yGoalY * checlBallx) + (robot.target.yGoalX * checlBally) /*+ C*/)) < 5)
//					{
//						state = 4;
//						kickTime = millis();
//					}
//					
//				}
//				else if(state == 4)
//				{
//					robot.driblerSpeed1 = 0;
//					robot.setMotors(0, 0, 0, 0);
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
//						robot.kick1 = true;
//						robot.kick2 = true;
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
//				robot.driblerSpeed1 = 0;
//			}
////			else		//not plaiyng
////			{
////				setPin(LED_2, 0);
//				
////				if(!robot.manualDribblerControl)
////				{
////					robot.driblerSpeed1 = 0; 
////					robot.driblerSpeed2 = 0;
////				}
////			}
////		}
////		else	//if errors occurs
////		{
////			setPin(LED_2, 0);
////			
////			if((GLOBAL_ERROR & LOW_BATTERY_POWER))
////			{
////				robot.setPlayState(0);
////				
////				robot.driblerSpeed1 = 0; 
////				robot.driblerSpeed2 = 0;
////				
////				static int lowBatteryCounter = 0;
////				lowBatteryCounter++;
////				if (lowBatteryCounter < 50)
////					robot.setMotors(100, 100, 100, 100);
////				else if (lowBatteryCounter < 100)
////					robot.setMotors(-100, -100, -100, -100);
////				else
////					lowBatteryCounter = 0;
////			}
////		}
//		
//		}
//};

//BaseFunctional::BaseFunctional(Robot RC):_RC(RC)
//{

//}

bool BaseFunctional::checkBall()
{
    return _RC->ballSensor.getValue();
}

void BaseFunctional::dribblerSpeed(int8_t speed)
{
//    _RC->updateDribblers(speed);
}

//RobotParametrs BaseFunctional::setAngle(RobotParametrs RP, int8_t angle)
//{
//    RP.angle = angle;
//    return RP;
//}

//RobotParametrs BaseFunctional::turnRC(RobotParametrs RP, int8_t angle, int8_t x, int8_t y)
//{
//    int8_t tmpX = RP.robot->_x - x;
//    int8_t tmpY = RP.robot->_y - y;
//    int8_t turnX = tmpX * cos(angle/57.3) - tmpY * sin(angle/57.3);
//    int8_t turnY = tmpX * sin(angle/57.3) + tmpY * cos(angle/57.3);
//    turnX = turnX + x;
//    turnY = turnY + y;
//    RP.robot->_x = turnX;
//    RP.robot->_y = turnY;
//    return RP;
//}

VectorToMove BaseFunctional::genATMVecField(int16_t x, int16_t y)//, vector<Obstacle> obs)
{
    //int16_t cX = RP.robot->_x - (ob._x - 59);
    ////int16_t cY = RP.robot->_y - (ob._y + 44);

		int16_t cX = 44 + x;
    int16_t cY = 59 - y;

	
    int16_t vecNum = 120 * (cX) + cY;//90 * (cY) + cX - 1;
    //int8_t atm = vF[vecNum][0] * 2;
    //int8_t atmMod = vF[vecNum][1];
		int16_t atm = 0;
    int8_t atmMod = 0;
		if(vecNum >= 0 && vecNum < 10800)
		{
			atm = firstField[vecNum][0] * 2 - 180 - 90;
			atmMod = firstField[vecNum][1];
		}
		
	
//    for(int i = 0; i < obs.size(); i++)
//    {
//        int16_t cObX = obs[i]._x - (ob._x - 59);
//        int16_t cObY = obs[i]._y - (ob._y + 44);

//        if(abs(double(cX - cObX)) <= (obs[i]._n / 2) && abs(double(cY - cObY)) <= (obs[i]._n / 2))
//        {
//            int16_t tmpATMX = atmMod * cos(double(atm)) + cObX;
//            int16_t tmpATMY = atmMod * sin(double(atm)) + cObY;
//            atmMod = sqrt(double(tmpATMX * tmpATMX + tmpATMY * tmpATMY));
//            atm = atan2(double(tmpATMY), double(tmpATMX));
//        }
//    }
		VectorToMove res(atm, atmMod);
    return res;
}

VectorToMove BaseFunctional::genATMPoint(int16_t x, int16_t y, int8_t vecMod)
{
    int16_t tmpX = y;
    int16_t tmpY = x;
    int16_t atm = atan2(double(tmpY), double(tmpX)) * 57.3;
		VectorToMove res(atm, vecMod);
		return res;
}

void BaseFunctional::move2(VectorToMove vtm, double heading)
{
    _RC->move(0.5 * vtm._mod, vtm._angle, heading);
}


void BaseFunctional::turnCoord(double angle, int16_t x, int16_t y, int16_t& xtoChange, int16_t& ytoChange)
{
    int8_t tmpX = x - xtoChange;//
    int8_t tmpY = y - ytoChange;
    int8_t turnX = tmpX * cos(angle/57.3) - tmpY * sin(angle/57.3);
    int8_t turnY = tmpX * sin(angle/57.3) + tmpY * cos(angle/57.3);
    turnX = turnX + x;
    turnY = turnY + y;
    xtoChange = turnX;
    ytoChange = turnY;
}


//void BaseFunctional::initVecField()
//{
//    for(int iter = 0; iter < vecNum; iter++)
//    {
//        map[iter] = new int8_t[infNum];
//    }
//    for(int iter = 0; iter < vecNum; iter++)
//    {
//        for(int jter = 0; jter < infNum; jter++)
//        {
//            map[iter][jter] = firstField[iter][jter];
//        }
//    }
//}
