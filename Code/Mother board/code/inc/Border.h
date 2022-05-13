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
        double error = double(pos.X - _start)/double(_end - _start);				
        double p = (error * 10.0);
        double d = (error - errorOldX) * 10.0;
				errorOldX = error;
				double u = abs(double(p + d));
				
        if(_side == '+' && _start < pos.X)
				{
					if(vtm._x > 0)
					{
						
						if (abs(u * vtm._x) <= abs(vtm._x))
						{
							vtm._x -= u * vtm._x;
							vtm._y -= u * vtm._y;
							vtm._mod = _speed;
							//vtm._mod = _speed/u;
						}
						else
						{
							vtm._x = -u * vtm._x * 0.01;
							vtm._mod = _speed;
						}
							
						
					}
					else if(vtm._x == 0)
					{
						vtm._x = -1.0;
						vtm._mod = _speed * u;
					}
					
					// / u;
				}
        else if(_side == '-' && pos.X < _start)
				{				
					if(vtm._x < 0)
					{
						if (abs(u * vtm._x) <= abs(vtm._x))
						{
							vtm._x -= u * vtm._x;
							vtm._y -= u * vtm._y;
							vtm._mod = _speed;
							//vtm._mod = _speed/u;
						}
						else
						{
							vtm._x = -u * vtm._x * 0.01;
							vtm._mod = _speed;
						}
						
					}
					else if(vtm._x == 0)
					{
						vtm._x = 1.0;
						vtm._mod = _speed * u;
					}
						
					// / u;// / u;
				}
    }
    else 
    {
        double error = double(pos.Y - _start)/double(_end - _start);
        double p = (error * 5.0);
				double d = (error - errorOldY) * 10.0;
				errorOldY = error;
				double u = abs(double(p + d));
        if(_side == '+' && _start < pos.Y)
				{
					if(vtm._y < 0)
					{
						if (abs(u * vtm._y) <= abs(vtm._y))
						{	
							vtm._y -= u * vtm._y;
							vtm._mod = _speed;
							//vtm._mod = _speed;
						}
						else
						{
							vtm._y = -u * vtm._y * 0.1;
							vtm._mod = _speed;
						}
					}
					else if(vtm._y == 0)
					{
						vtm._y = 1.0;
					  vtm._mod = _speed * u;
					}
					
					
					// / u;// / u;
				}
        else if(_side == '-' && pos.Y < _start)
				{
					if(vtm._y > 0)
					{
						
            if (abs(u * vtm._y) <= abs(vtm._y))
						{
							vtm._y -= u * vtm._y;
							vtm._mod = _speed;
							//vtm._mod = _speed;
						}
						else
						{
							vtm._y = -u * vtm._y*0.01;
							vtm._mod = _speed;
						}
						
					}
					else if (vtm._y == 0)
					{
						vtm._y = -1.0;
						vtm._mod = _speed * u;
					}
					
					// / u;
				}
    }
}

