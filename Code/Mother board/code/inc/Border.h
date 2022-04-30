#pragma once 

#include <stdint.h>
#include <utility>
#include <VTM.h>
#include <main.h>

using namespace std;
class Border
{
    public:
        Border(char coord, char side, int16_t start, int16_t end):_coord(coord), _side(side), _start(start), _end(end){}
        void dempher(pair<int16_t, int16_t> pos, VectorToMove& vtm);
    private:
        char _coord;
        char _side;
        int16_t _start;
        int16_t _end;
};

void Border::dempher(pair<int16_t, int16_t> pos, VectorToMove& vtm)
{
    if(_coord == 'x')
    {
        int32_t error = pos.X - _start;
        double p = (error * 1.0)/(_end - _start);
        double u = abs(double(p));
        if(_side == '+' && _start < pos.X && vtm.x > 0)
            vtm.x -= u * vtm.x;
        else if(_side == '-' && pos.X < _start && vtm.x < 0)
            vtm.x -= u * vtm.x;
    }
    else 
    {
        int32_t error = pos.Y - _start;
        double p = (error * 1.0)/(_end - _start);
        double u = abs(double(p));
        if(_side == '+' && _start < pos.Y && vtm.y > 0)
            vtm.y -= u * vtm.y;
        else if(_side == '-' && pos.Y < _start && vtm.y < 0)
            vtm.y -= u * vtm.y;
    }
}

