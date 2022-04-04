#ifndef MPU9250_LIB
#define MPU9250_LIB

//for stm32f407!!!

#include "math.h"

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"


#include "mpu9250_registers.h"


#define ACC_FS 4				//+-2g, +-4g, +-8g, +-16g				select accelerometer and gyroscope full-scale range
#define GYRO_FS 1000		//+-250°/s, +-500°/s, +-1000°/s, +-2000°/s


class mpu9250
{
public:
	double xa, ya, za;
	double xg, yg, zg;
	double temp;
	double q0, q1, q2, q3;
	double pitch, roll, yaw;
	double pitchOffset, rollOffset, yawOffset;
	
	void initIMU(unsigned int _spi, int _ssPin);
	void readAcc();
	void readGyro();
	void readTemp();
	int readFIFOsize();
	void readFIFO();
	void setGyroOffset(int xOffset, int yOffset, int zOffset);
	void setAccOffset(int xOffset, int yOffset, int zOffset);
	void calibrate(int _T);
	void correct();
	void MadgwickAHRSupdateIMU(double _time);
	float invSqrt(float x);
	void updateAngles(double _time = -1, bool flag1 = 1);
	void updateAnglesFromFIFO();
	
	
private:
	unsigned int mpuSPI;
	int mpuSSPin;
	double ACC_DIV, GYRO_DIV;
	double xGyroOffset, yGyroOffset, zGyroOffset;
	double xAccOffset, yAccOffset, zAccOffset;
	long long int time;
	long long int correctionTimer;
	
	inline int convert(uint16_t msb, uint8_t lsb);
	int readReg(int regAddr, int * data = 0, int q = 0);
	void writeReg(int regAddr, uint8_t data);
};

#endif
