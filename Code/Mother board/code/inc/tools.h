#ifndef TOOLS_LIB
#define TOOLS_LIB

#include <stdint.h>
#include <utility>
#include <math.h>

/* CONSTANTS */
#define Byte2Angle 	1.40625f
#define DEG2RAD 		0.01745329251f
#define RAD2DEG 		57.2957795131f
#define PI 					3.14159265359f

#define NO_RATIONAL 1
#define ST_RATIONAL 3
#define LG_RATIONAL 6

#define X		first
#define Y		second

using namespace std;

/* cutting with boundaries */
#define stp(x, val) (x > val ? val : (x < -val ? -val : x))

#define adduction(x) {while(x>180)x-=360;while(x<-180)x+=360;}

template<typename T> T abs(T x) { return (x > 0 ? x : -x); }

template<typename T> T sq(T x) { return x * x; }

template<typename T> T max(T a, T b) { return (a > b ? a : b); }
template<typename T> T max2(T a, T b) { return (a > b ? a : b); }
template<typename T> T max4(T a, T b, T c, T d) { return max2(max2(a, b), max2(c, d)); }
template<typename T> T max5(T a, T b, T c, T d, T e) { return max2(max4(a, b, c, d), e); }

template<typename T> T min(T a, T b) { return (a < b ? a : b); }
template<typename T> T min2(T a, T b) { return (a < b ? a : b); }

template<typename T> T crc8(T* data, int len)
{
    T crc = 0xFF, i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (char)((crc << 1) ^ 0x31);
            else crc <<= 1;
        }
    }
    return crc;
}

template<typename T> T sgn(T x)
{
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}

pair<int, int> rotate(pair<int, int> pos, double angle)
{
    angle *= DEG2RAD;
    pair<int, int> tmpPos;

    tmpPos.X = pos.X*cos(angle) - pos.Y*sin(angle);
    tmpPos.Y = pos.X*sin(angle) + pos.Y*cos(angle);

    return tmpPos;
}

#endif
