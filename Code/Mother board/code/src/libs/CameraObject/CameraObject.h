#pragma once
#include <stdint.h>
#include <utility>
#include <math.h>
#include <PairSaver.h>
#include <tools.h>

using namespace std;

class CameraObject
{
	public:
		pair<int, int> pos;			//Local camera coordinates
		pair<int, int> oldPos;		//Previous local cam coordinates
	
    CameraObject()
    {
        pos.X = 0;
				pos.Y = 0;
				oldPos.X = 0;
        oldPos.Y = 0;
    }
		
};
