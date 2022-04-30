#pragma once

#include <stdint.h>
struct VectorToMove
{
    //double _angle;
    double _mod;
		double _x;
		double _y;
		VectorToMove(double x, double y, double mod):_x(x), _y(y), _mod(mod){} //double angle, double mod)
};
