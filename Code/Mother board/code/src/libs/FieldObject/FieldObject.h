#pragma once
#include <stdint.h>
#include <utility>
#include <math.h>
#include <PairSaver.h>
#include <tools.h>

using namespace std;

class FieldObject
{
	public:
    pair<double, double> globalPos;	//Position in Global coordinates
		pair<double, double> v;		//Speed in Global coordinates
		PairSaver posSaver;
		PairSaver speedSaver;
	
	
	public:
    FieldObject()
    {
			posSaver.add(make_pair(0,0), 0);
			speedSaver.add(make_pair(0,0), 0);
			globalPos.X = 0;
			globalPos.Y = 0;
			v.X = 0;
			v.Y = 0;
    }
		
		
    void update(pair<double, double> localInput, pair<double, double> globalInput, uint32_t time, uint16_t calcTime)
    {
		globalPos.X = globalInput.X + localInput.X;
		globalPos.Y = globalInput.Y + localInput.Y;

		posSaver.add(globalPos, time);
		pair<int, int> oldPos = posSaver.pop(time - calcTime);

		v.X = (globalPos.X - oldPos.X)/(calcTime/1000);
		v.Y = (globalPos.Y - oldPos.Y)/(calcTime/1000);

		speedSaver.add(v, time);
    }
		
};
