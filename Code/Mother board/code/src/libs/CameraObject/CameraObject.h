#pragma once
#include <stdint.h>
#include <utility>
#include <math.h>
#include <tools.h>

using namespace std;

struct CameraObject
{
	public:
		pair<double, double> pos;			//Local camera coordinates
		pair<double, double> oldPos;		//Previous local cam coordinates
	
    CameraObject()
    {
        pos.X = 0;
		pos.Y = 0;
		oldPos.X = 0;
        oldPos.Y = 0;
    }
		
};
