#pragma once

//#include <struct_Object.h>
#include <stdint.h>

struct Object
{
    int* _xAddress;
    int* _yAddress;
    double vx, vy;
    int xOld, yOld;
    int timeOld;
    int _x;
    int _y;
    Object(int* xAddress, int* yAddress)
    {
        _xAddress = xAddress;
        _yAddress = yAddress;
        _x = 0;
        _y = 0;
        vx = 0;
        vy = 0;
        xOld = 0;
        yOld = 0;
        timeOld = 0;
    }
    void update(int time)
    {
        _x = *_xAddress;
        _y = *_yAddress;
        vx = 1.0*(_x - xOld) / (time - timeOld);
        vy = 1.0*(_y - yOld) / (time - timeOld);
        timeOld = time;
        xOld = _x;
        yOld = _y;
    }
};

//struct RobotParametrs
//{
//    int8_t angle;
//    Object* robot;
//    RobotParametrs(Object* rOb, int8_t ang)
//    {
//        robot = rOb;
//        angle = ang;
//    }
//};
