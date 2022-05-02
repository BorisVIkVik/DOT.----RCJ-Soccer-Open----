#pragma once 

#include <stdint.h>
#include <utility>
#include <VTM.h>

//#include <main.h>

using namespace std;
class Border
{
    public:
        Border(char coord, char side, int16_t start, int16_t end, int16_t minLimit, int16_t maxLimit):_coord(coord), _side(side), _start(start), _end(end), _minLimit(minLimit), _maxLimit(maxLimit){}
        void dempher(pair<int16_t, int16_t> pos, VectorToMove& vtm);
    private:
        char _coord;
        char _side;
        int16_t _start;
        int16_t _end;
				int16_t _minLimit;
				int16_t _maxLimit;
		
};

void Border::dempher(pair<int16_t, int16_t> pos, VectorToMove& vtm)
{
    if(_coord == 'x')
    {
		//	if(_minLimit < pos.Y && pos.Y < _maxLimit)
		//	{
        int32_t error = pos.X - _start;
        double p = (error * 1.0)/(_end - _start);
        double u = abs(double(p));
        if(_side == '+' && _start < pos.X && vtm._x > 0)
            vtm._x -= u * vtm._x;
        else if(_side == '-' && pos.X < _start && vtm._x < 0)
            vtm._x -= u * vtm._x;
			//}
    }
    else 
    {
			//if(_minLimit < pos.X && pos.X < _maxLimit)
		//	{
        int32_t error = pos.Y - _start;
        double p = (error * 1.0)/(_end - _start);
        double u = abs(double(p));
        if(_side == '+' && _start < pos.Y && vtm._y < 0)
            vtm._y -= u * vtm._y;
        else if(_side == '-' && pos.Y < _start && vtm._y > 0)
            vtm._y -= u * vtm._y;
		//	}
    }
}

