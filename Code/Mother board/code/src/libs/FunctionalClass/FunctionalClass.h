#pragma once

#include "Robot.h"
#include <RobotParametrs.h>
//#include <stdint.h>
#include <Obstacle.h>
#include <VTM.h>
#include <VecField.h>
//#include <vector>
#include <math.h>
#include <PairSaver.h>
#include <trajectory.h>

using namespace std;

int16_t vecNum = 10800;
int8_t infNum = 2;
//int8_t** map = new int8_t*[vecNum];

#define KOEF_A_P	0.05
#define KOEF_A_D	0.09

#define KOEF_G_P	0.08
#define KOEF_G_I	0.000000001
#define KOEF_G_D	0.5

class BaseFunctional
{
    public:
                            BaseFunctional(Robot* RC):_RC(RC),errorOld(0), i(0){}
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
        void                move2(VectorToMove vtm, double heading,  double acceleration, uint16_t maxRotSpeed = 400);
				void								vectorMove(VectorToMove vtm);
        VectorToMove        genATMPoint(int16_t x, int16_t y, double vecMod); 
        VectorToMove        genATMVecField(int16_t x, int16_t y);//, vector<Obstacle> obs);
				VectorToMove 				genVTMGlobalPoint(pair<int16_t, int16_t> toGoCoords, pair<int16_t, int16_t> robotCoords, double vecMod, char robotMode);
				bool 								checkBounds(pair<double, double> nizLF, pair<double, double> verxPR, pair<double, double> pointToCheck);
				Robot*							getRobotClass();
				VectorToMove 				trajectoryFollowingDots(int16_t& oldPosIndex, double distance, char side, double maxVecSpeed);
				int16_t 						findStartOfTrajectory(pair<int16_t, int16_t> pos);
				VectorToMove 				Parabola(pair<int16_t, int16_t> toGoCoords, pair<int16_t, int16_t> robotCoords, double maxVecMod);
    private:
				double errorOld;
        Robot* _RC;
				double i;
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
    int16_t cY = 58 - y;

	
    int16_t vecNum = 120 * (cX) + cY;//90 * (cY) + cX - 1;
    //int8_t atm = vF[vecNum][0] * 2;
    //int8_t atmMod = vF[vecNum][1];
		int16_t atm = 0;
    double atmMod = 0;
		if(vecNum >= 0 && vecNum < 10800)
		{
			atm = firstField[vecNum][0] * 2 - 180 - 90;
			double error = sqrt(double(cX * cX + cY * cY));
			double p = error * KOEF_A_P;
			//double d = (error - errorOld) * KOEF_D;
			double u = p;
			//errorOld = error;
			atmMod = min2(2.0, abs(u));
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
		VectorToMove res(sin(double(atm/57.3))*atmMod, cos(double(atm/57.3))*atmMod, atmMod);
    return res;
}

VectorToMove BaseFunctional::genATMPoint(int16_t x, int16_t y, double vecMod)
{
    //int16_t tmpX = x;
    //int16_t tmpY = y;
    //int16_t atm = atan2(double(), double()) * 57.3;
		VectorToMove res(x, y, vecMod);
		return res;
}

void BaseFunctional::move2(VectorToMove vtm, double heading, double acceleration, uint16_t maxRotSpeed)
{
		int16_t atm = atan2(double(vtm._x), double(vtm._y)) * 57.3;
    _RC->move(vtm._mod, atm, heading, acceleration, 1.1, maxRotSpeed);
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

VectorToMove BaseFunctional::genVTMGlobalPoint(pair<int16_t, int16_t> toGoCoords, pair<int16_t, int16_t> robotCoords, double maxVecMod, char robotMode)
{
    int16_t tmpX = toGoCoords.X - robotCoords.X;
    int16_t tmpY = -toGoCoords.Y + robotCoords.Y;
    //int16_t atm = atan2(double(), double()) * 57.3;
		double error = sqrt(double(tmpX * tmpX + tmpY * tmpY));
		double p = error * (robotMode == 'a'? KOEF_A_P : KOEF_G_P);
		i += error * (robotMode == 'a'? 0 : KOEF_G_I);
		double d = (error - errorOld) * (robotMode == 'a'? KOEF_A_D : KOEF_G_D);
		double u = p + i + d;
		errorOld = error;
	VectorToMove res(tmpX, tmpY, min2(maxVecMod, abs(u)));
	return res;
}

bool BaseFunctional::checkBounds(pair<double, double> nizLF, pair<double, double> verxPR, pair<double, double> pointToCheck)
{
	if((nizLF.Y < pointToCheck.Y && pointToCheck.Y < verxPR.Y) && (nizLF.X < pointToCheck.X && pointToCheck.X < verxPR.X))
	{
		return true;
	}
	else
	{
		return false;
	}
}

Robot* BaseFunctional::getRobotClass()
{
	return _RC;
}

VectorToMove BaseFunctional::trajectoryFollowingDots(int16_t& oldPosIndex, double distance, char side, double maxVecSpeed)
{
		int toGoPosIndex = oldPosIndex;
		for(; toGoPosIndex < TRAJECTORY1_STOP - 1; toGoPosIndex++)
		{
				pair<int16_t, int16_t> tmpMovedVec = make_pair(trajectory1[toGoPosIndex][0] - trajectory1[oldPosIndex][0], trajectory1[toGoPosIndex][1] - trajectory1[oldPosIndex][1]);
				double tmpDistance = sqrt(double(tmpMovedVec.X * tmpMovedVec.X + tmpMovedVec.Y * tmpMovedVec.Y));
				if(distance <= tmpDistance)
						break;
		}
			
		
    //pair<int16_t, int16_t> toGo = make_pair(trajectory1[toGoPosIndex][0] - _RC->getPos().X, trajectory1[toGoPosIndex][1] - _RC->getPos().Y);
		_RC->display.print("GP:", 1, 1);
		_RC->display.print(toGoPosIndex, 1, 5);
    //toGo.X += 
    //toGo.Y +=
    //double angleToMove = atan2(double(), double()) * 57.3;
		oldPosIndex = toGoPosIndex;
    double vecMod = maxVecSpeed;//SPEED_TRAJ_FOLLOW_M_S;
    VectorToMove res(0, 0, 0);
		if (side == 'r')
		{
			res = genVTMGlobalPoint(make_pair(trajectory1[toGoPosIndex][0] , trajectory1[toGoPosIndex][1] - 8), _RC->getPos(), vecMod, 'a');
		}
		else
		{
			res = genVTMGlobalPoint(make_pair(-trajectory1[toGoPosIndex][0] - 5, trajectory1[toGoPosIndex][1] - 8), _RC->getPos(), vecMod, 'a');
		}
    return res;
}


int16_t BaseFunctional::findStartOfTrajectory(pair<int16_t, int16_t> pos)
{
	int iter = 0;
	for(; iter < TRAJECTORY1_STOP - 1; iter++)
	{
		if(pos.Y <= trajectory1[iter][1])
			break;
	}
	return iter;
}

VectorToMove BaseFunctional::Parabola(pair<int16_t, int16_t> toGoCoords, pair<int16_t, int16_t> robotCoords, double maxVecMod)
{
	double ang = 0;
	if(toGoCoords.X - robotCoords.X != 0)
	{
		ang = atan2(((toGoCoords.X - robotCoords.X) > 0 ? 1.0 : -1.0), ((toGoCoords.X - robotCoords.X) > 0 ? -1.0 : 1.0) * double(double((robotCoords.Y - toGoCoords.Y)*2) / double(robotCoords.X - toGoCoords.X))) * 57.3;
	}
	else
	{
		if(toGoCoords.Y >= robotCoords.Y)
			ang = PI/2;
		else
			ang = -PI/2;
	}
	//VectorToMove res(0,0,0);
	double vecMod = maxVecMod;
	VectorToMove res(sin(double(ang/57.3))*vecMod, cos(double(ang/57.3))*vecMod, vecMod);
  return res;
}

void BaseFunctional::vectorMove(VectorToMove vtm,  double angle)
{
	vtm._x * 0 + vtm._y * 2/R + angle * L/R;
	vtm._x * -2/R + vtm._y * 0 + angle * L/R;
	vtm._x * 0 + vtm._y * -2/R + angle * L/R;
	vtm._x * 2/R + vtm._y * 0 + angle * L/R;
}
