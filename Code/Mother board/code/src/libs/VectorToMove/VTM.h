#pragma once

#include <stdint.h>
struct VectorToMove
{
    //double _angle;
    double _mod;
		int16_t _x;
		int16_t _y;
		VectorToMove(int16_t x, int16_t y, double mod):_x(x), _y(y), _mod(mod){} //double angle, double mod)
};
