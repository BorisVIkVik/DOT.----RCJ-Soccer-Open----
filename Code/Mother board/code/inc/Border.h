#pragma once 

#include <tools.h>
#include <VTM.h>

//#include <main.h>

using namespace std;
class Border
{
    public:
        Border(double speed, char coord, char side, int16_t start, int16_t end, int16_t minLimit, int16_t maxLimit):_speed(speed), _coord(coord), _side(side), _start(start), _end(end), _minLimit(minLimit), _maxLimit(maxLimit), errorOldX(0), errorOldY(0){}
        void dempher(pair<int16_t, int16_t> pos, VectorToMove& vtm);
    private:
				double _speed;
        char _coord;
        char _side;
        int16_t _start;
        int16_t _end;
				int16_t _minLimit;
				int16_t _maxLimit;
				double errorOldX;
				double errorOldY;
		
};

void Border::dempher(pair<int16_t, int16_t> pos, VectorToMove& vtm)
{
    if(_coord == 'x')
    {
        double error = (pos.X - _start)/(_end - _start);				
        double p = (error * 1.0);
        double d = (error - errorOldX) * 6.0;
				errorOldX = error;
				double u = abs(double(p + d));
				
        if(_side == '+' && _start < pos.X)
				{
					if(vtm._x > 0)
					{
						
						if (abs(u * vtm._x) <= abs(vtm._x))
						{
							vtm._x -= u * vtm._x;
							vtm._mod = _speed/u;
						}
						else
							vtm._x = -u * vtm._x * 0.01;
						
					}
					else if(vtm._x == 0)
					{
						vtm._x = -1.0;
						vtm._mod = _speed;
					}
					
					vtm._mod = _speed;// / u;
				}
        else if(_side == '-' && pos.X < _start)
				{				
					if(vtm._x < 0)
					{
						if (abs(u * vtm._x) <= abs(vtm._x))
						{
							vtm._x -= u * vtm._x;
							vtm._mod = _speed/u;
						}
						else
							vtm._x = -u * vtm._x * 0.01;
						
					}
					else if(vtm._x == 0)
					{
						vtm._x = 1.0;
						vtm._mod = _speed;
					}
						
					// / u;
				}
    }
    else 
    {
        double error = (pos.Y - _start)/(_end - _start);
        double p = (error * 4.0);
				double d = (error - errorOldY) * 16.0;
				errorOldY = error;
				double u = abs(double(p + d));
        if(_side == '+' && _start < pos.Y)
				{
					if(vtm._y < 0)
					{
						if (abs(u * vtm._y) <= abs(vtm._y))
						{	
							vtm._y -= u * vtm._y;
							vtm._mod = _speed/u;
						}
						else
							vtm._y = -u * vtm._y * 0.1;
					}
					else if(vtm._y == 0)
					{
						vtm._y = 1.0;
						vtm._mod = _speed;
					}
					
					
					// / u;
				}
        else if(_side == '-' && pos.Y < _start)
				{
					if(vtm._y > 0)
					{
						
            if (abs(u * vtm._y) <= abs(vtm._y))
						{
							vtm._y -= u * vtm._y;
							vtm._mod = _speed/u;
						}
						else
							vtm._y = -u * vtm._y * 0.1;
						
					}
					else if (vtm._y == 0)
					{
						vtm._y = -1.0;
						vtm._mod = _speed;
					}
					
					//vtm._mod = _speed;// / u;
				}
    }
}

