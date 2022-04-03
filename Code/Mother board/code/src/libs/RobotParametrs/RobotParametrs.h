#include <struct_Object.h>
#include <stdint.h>

struct RobotParametrs
{
    int8_t angle;
    Object* robot;
    RobotParametrs(Object* rOb, int8_t ang)
    {
        robot = rOb;
        angle = ang;
    }
};
