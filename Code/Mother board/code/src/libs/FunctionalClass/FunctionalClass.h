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


bool BaseFunctional::checkBall()
{
    return _RC->ballSensor.getValue();
}

void BaseFunctional::dribblerSpeed(int8_t speed)
{
//    _RC->updateDribblers(speed);
}


VectorToMove BaseFunctional::genATMVecField(int16_t x, int16_t y)//, vector<Obstacle> obs)
{
		int16_t cX = 44 + x;
    int16_t cY = 58 - y;

    int16_t vecNum = 120 * (cX) + cY;//90 * (cY) + cX - 1;
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
		VectorToMove res(sin(double(atm/57.3))*atmMod, cos(double(atm/57.3))*atmMod, atmMod);
    return res;
}

VectorToMove BaseFunctional::genATMPoint(int16_t x, int16_t y, double vecMod)
{
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
			
		
		_RC->display.print("GP:", 1, 1);
		_RC->display.print(toGoPosIndex, 1, 5);
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
	double vecMod = maxVecMod;
	VectorToMove res(sin(double(ang/57.3))*vecMod, cos(double(ang/57.3))*vecMod, vecMod);
  return res;
}

