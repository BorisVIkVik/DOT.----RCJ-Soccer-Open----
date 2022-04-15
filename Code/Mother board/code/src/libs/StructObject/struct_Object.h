#pragma once
#include <stdint.h>
#include <utility>
#include <math.h>

using namespace std;

//To global defines
#define X		first
#define Y		second
///

#define DEG2RAD	0.01745329252

#define GOAL_X 0
#define GOAL_Y 103


pair<int32_t, int32_t> filterKalman(pair<int32_t, int32_t> sensorValue, pair<int32_t, int32_t> predictedValue, double coeffK)
{
    pair<int32_t, int32_t> res;
    res.first = sensorValue.first * coeffK + (1 - coeffK) * (predictedValue.first);
    res.second = sensorValue.second * coeffK + (1 - coeffK) * (predictedValue.second);
    return res;
}

pair<int32_t, int32_t> predict(uint32_t delay, pair<int, int> old, pair<double, double> speedRV, double speedRW, pair<double, double> speedBV)
{
    double fi = speedRW * delay;
    int32_t tmpX = old.X + (speedBV.first - speedRV.first) * delay; 
    int32_t tmpY = old.Y + (speedBV.second - speedRV.second) * delay; 
    pair<int32_t, int32_t> res;
    res.first = tmpX * cos(fi) - tmpY * sin(fi);
    res.second = tmpX * sin(fi) + tmpY * cos(fi);
    return res;
}


class Object
{
	public:
    Object(pair<int, int>* _yellow, pair<int, int>* _blue, pair<int, int>* _obj, double* _angle)
    {
				this->_yellow = _yellow;
				this->_blue = _blue;
				this->_obj = _obj;
				this->_angle=_angle;
        cam.X = 0;
        cam.Y = 0;
				oldCam.X = 0;
        oldCam.Y = 0;
				global.X = 0;
				global.Y = 0;
        v.X = 0;
        v.Y = 0;
    }
		
    void update(uint32_t time)
    {
				double angle = *_angle * DEG2RAD;			
			
				cam.X = (*_obj).X * cos(angle) - (*_obj).Y * sin(angle);		//Local
				cam.Y = (*_obj).X * sin(angle) - (*_obj).Y * cos(angle);
			
				int xYellowCam = -((*_yellow).X * cos(angle) - (*_yellow).Y * sin(angle));
				int yYellowCam = -((*_yellow).X * sin(angle) + (*_yellow).Y * cos(angle));
				int xBlueCam = -((*_blue).X * cos(angle) - (*_blue).Y * sin(angle));
				int yBlueCam = ((*_blue).X * sin(angle) + (*_blue).Y * cos(angle));
			
				double K = 0.5 - (sqrt(double(xYellowCam*xYellowCam + yYellowCam*yYellowCam)) - sqrt(double(xBlueCam*xBlueCam + yBlueCam*yBlueCam)))/134*0.5;
				if(K > 1.0) K = 1.0;
				if(K < 0.0) K = 0.0;
					
				global.X = (GOAL_X + xYellowCam)*K + (GOAL_X + xBlueCam)*(1.0 - K) + cam.X;
				global.Y = (-GOAL_Y + yYellowCam)*K + (GOAL_Y + yBlueCam)*(1.0 - K) + cam.Y;
			
				coordSaver.add(global, time);
			
				v.X = 
				v.Y = 
				
				coordSaver.add(v, time);
    }
		
		
	private:
		pair<int, int>* _yellow;
		pair<int, int>* _blue;
    pair<int, int>* _obj;
		double* _angle;
    pair<double, double> v; //Global
		pair<int, int> cam;	//Local
		pair<int, int> oldCam;	//Local
    pair<int, int> global; //Global
};
