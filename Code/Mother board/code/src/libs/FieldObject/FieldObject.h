#pragma once
#include <stdint.h>
#include <utility>
#include <math.h>
#include <PairSaver.h>
#include <tools.h>

using namespace std;

#define GOAL_X 0
#define GOAL_Y 103


class FieldObject
{
	public:
    	pair<int, int> globalPos;	//Position in Global coordinates
		pair<int, int> v;		//Speed in Global coordinates
		PairSaver posSaver;
		PairSaver speedSaver;
	
	
	public:
    FieldObject()
    {
			posSaver = *(new PairSaver());
			speedSaver = *(new PairSaver());
			globalPos.X = 0;
			globalPos.Y = 0;
        v.X = 0;
        v.Y = 0;
    }
		
		
    void update(pair<int, int> localInput, pair<int, int> globalInput, uint32_t time, uint16_t cameraDelay)
    {

		globalPos.X = globalInput.X + localInput.X;
		globalPos.Y = globalInput.Y + localInput.Y;

		posSaver.add(globalPos.X, time);
		pair<int, int> oldPos = posSaver.pop(time - cameraDelay);

		v.X = (globalPos.X - oldPos.X)/(cameraDelay/1000);
		v.Y = (globalPos.Y - oldPos.Y)/(cameraDelay/1000);

		speedSaver.add(v, time);
    }
		
};
