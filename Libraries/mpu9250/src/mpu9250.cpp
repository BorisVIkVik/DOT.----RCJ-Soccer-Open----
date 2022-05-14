//for stm32f407!!!

#include "mpu9250.h"

#define RAD2DEG	57.2957795130823208767
#define DEG2RAD	0.01745329251994329576
#define beta		0.25f		// 2 * proportional gain



inline int mpu9250::convert(uint16_t msb, uint8_t lsb)
{
	uint16_t a = (msb<<8) | lsb;
	if(a & 32768) return (a & 32767) - 32768;
	else return a;
}


///volatile int32_t alucard = 0;
int mpu9250::readReg(int regAddr, int * data, int q)
{
	setPin(mpuSSPin, 0);
	writeSPI(mpuSPI, 0x80 | regAddr);
	for(int i = 0; i < q; i++)
		data[i] = writeSPI(mpuSPI, 0);
	int a = writeSPI(mpuSPI, 0);
	setPin(mpuSSPin, 1);
	return a;
}



void mpu9250::writeReg(int regAddr, uint8_t data)
{
	setPin(mpuSSPin, 0);
	writeSPI(mpuSPI, regAddr);
	writeSPI(mpuSPI, data);
	setPin(mpuSSPin, 1);
}



void mpu9250::initIMU(unsigned int _spi, int _ssPin)
{
	delay(10);
	ACC_DIV = 32768.0 / ACC_FS;
	GYRO_DIV = 32768.0 / GYRO_FS / DEG2RAD;
	
	mpuSPI = _spi;
	mpuSSPin = _ssPin;
	xGyroOffset = 0.0235, yGyroOffset = 0.0155, zGyroOffset = 3.1545;
	xAccOffset = 0.001, yAccOffset = 0.001, zAccOffset = 0.001;
	pitchOffset = 0, rollOffset = 0, yawOffset = 0;
	
	xa = 0, ya = 0, za = 0;
	xg = 0, yg = 0, zg = 0;
	temp = 0;
	pitch = 0, roll = 0, yaw = 0;
	q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
	time = 0;
	
	int accFSsel, gyroFSsel;
	if(ACC_FS == 2) accFSsel = 0x0;
	else if(ACC_FS == 4) accFSsel = 0x8;
	else if(ACC_FS == 8) accFSsel = 0x10;
	else if(ACC_FS == 16) accFSsel = 0x18;
	
	if(GYRO_FS == 250) gyroFSsel = 0x0;
	else if(GYRO_FS == 500) gyroFSsel = 0x8;
	else if(GYRO_FS == 1000) gyroFSsel = 0x10;
	else if(GYRO_FS == 2000) gyroFSsel = 0x18;

		
	initSPI(mpuSPI, MASTER, 8, 256);			//init spi
	initPin(mpuSSPin, OUTPUTPP);
	setPin(mpuSSPin, 1);
	
	writeReg(PWR_MGMT_1, 0x80);			//reset
	delay(1);
	writeReg(PWR_MGMT_1, 0x01);			//autoselect the best available clock source
	delay(1);
	writeReg(PWR_MGMT_2, 0x00);			//enable all axis of accelerometer and gyroscope
	delay(1);
	writeReg(CONFIG, 0x03);						//DLPF (digital low pass filter) = 3
	delay(1);
	writeReg(GYRO_CONFIG, gyroFSsel);			//set gyroscope scale, FCHOICE = 2b11 (FCHOICE_b = 2b00)
	delay(1);
	writeReg(ACCEL_CONFIG, accFSsel);			//set accelerometer scale
	delay(1);
	writeReg(ACCEL_CONFIG_2, 0x03);			//accelerometer DLPF = 3, FCHOICE = 1 (FCHOICE_b = 0)
	delay(1);
	writeReg(INT_PIN_CFG, 0x30);				//latch INT pin untill any read operation
	delay(1);
	writeReg(USER_CTRL, 0x04);			//enable fifo (first input first output buffer) and reset it
	delay(1);
	writeReg(USER_CTRL, 0x40);
	delay(1);
	writeReg(FIFO_EN, 0x78);			//write accel and gyro data to FIFO
	delay(5);
	
	
	setGyroOffset(0, 0, 0);
	setAccOffset(0, 0, 0);
	
	
	if(_spi == _SPI1)											//set frequency to 17.75 MHz
	{
		SPI1->CR1 &= ~SPI_CR1_BR;
		SPI1->CR1 |= SPI_CR1_BR_1;
	}
	else if(_spi == _SPI2)
	{
		SPI2->CR1 &= ~SPI_CR1_BR;
		SPI2->CR1 |= SPI_CR1_BR_1;
	}
	else if(_spi == _SPI3)
	{
		SPI3->CR1 &= ~SPI_CR1_BR;
		SPI3->CR1 |= SPI_CR1_BR_1;
	}
	correctionTimer = millis();
	
}



void mpu9250::readAcc( )						//reads data directly from accelerometer registers
{
	int data[6];
	readReg(ACCEL_XOUT_H, data, 6);
	xa = double(convert(data[0], data[1])) / ACC_DIV - xAccOffset;
	ya = double(convert(data[2], data[3])) / ACC_DIV - yAccOffset;
	za = double(convert(data[4], data[5])) / ACC_DIV - zAccOffset;
}



void mpu9250::readGyro()					//reads data directly from gyroscope registers. Returns values in RAD/S
{
	int data[6];
	readReg(GYRO_XOUT_H, data, 6);
	xg = double(convert(data[0], data[1])) / GYRO_DIV - xGyroOffset;
	yg = double(convert(data[2], data[3])) / GYRO_DIV - yGyroOffset;
	zg = double(convert(data[4], data[5])) / GYRO_DIV - zGyroOffset;
}



void mpu9250::readTemp()					//reads data directly from temperature registers
{
	temp = (convert(readReg(TEMP_OUT_H), readReg(TEMP_OUT_L)) - 21.0)/333.87 + 21.0;	//21.0 and 333.87 are some magic numbers
}



int mpu9250::readFIFOsize()					//reads quantity of bites in FIFO
{
	return convert((readReg(FIFO_COUNTH) & 0x1F), readReg(FIFO_COUNTL));
}



void mpu9250::readFIFO()					//reads accelerometer and gyroscope data from FIFO
{
	int d0 = readReg(FIFO_R_W);			//first writes values to variables because
	int d1 = readReg(FIFO_R_W);			//just read from fifo rw register is too fast
	int d2 = readReg(FIFO_R_W);			//(values in that buffer havent enough time to update)
	int d3 = readReg(FIFO_R_W);			//da, eto kostyl`!
	int d4 = readReg(FIFO_R_W);
	int d5 = readReg(FIFO_R_W);
	int d6 = readReg(FIFO_R_W);
	int d7 = readReg(FIFO_R_W);
	int d8 = readReg(FIFO_R_W);
	int d9 = readReg(FIFO_R_W);
	int d10 = readReg(FIFO_R_W);
	int d11 = readReg(FIFO_R_W);
	xa = double(convert(d0, d1) - xAccOffset) / ACC_DIV;
	ya = double(convert(d2, d3) - yAccOffset) / ACC_DIV;
	za = double(convert(d4, d5) - zAccOffset) / ACC_DIV;
	
	xg = double(convert(d6, d7) - xGyroOffset) / GYRO_DIV;
	yg = double(convert(d8, d9) - yGyroOffset) / GYRO_DIV;
	zg = double(convert(d10, d11) - zGyroOffset) / GYRO_DIV;
}



void mpu9250::setGyroOffset(int _xOffset, int _yOffset, int _zOffset)				//sets hardware gyroscope offset for +-1000°/s full-scale
{
	writeReg(XG_OFFSET_H, _xOffset >> 8);
	writeReg(XG_OFFSET_L, _xOffset & 0xFF);
	writeReg(YG_OFFSET_H, _yOffset >> 8);
	writeReg(YG_OFFSET_L, _yOffset & 0xFF);
	writeReg(ZG_OFFSET_H, _zOffset >> 8);
	writeReg(ZG_OFFSET_L, _zOffset & 0xFF);
}



void mpu9250::setAccOffset(int xOffset, int yOffset, int zOffset)				//sets hardware accelerometer offset for +-16g full-scale
{
	writeReg(XG_OFFSET_H, xOffset >> 7);
	writeReg(XG_OFFSET_L, (xOffset & 0x7F) << 1);
	writeReg(YG_OFFSET_H, yOffset >> 7);
	writeReg(YG_OFFSET_L, (yOffset & 0x7F) << 1);
	writeReg(ZG_OFFSET_H, zOffset >> 7);
	writeReg(ZG_OFFSET_L, (zOffset & 0x7F) << 1);
}



//For calibration imu must stand still (for finding gyro offsets)
//and pointing acc Z axis straight downwards!!!
//
//If other axis points upperwards
//it`s (n is x, y or z) nAccOffset calculates as 
//_dan/_T - (32768 / ACC_FS) (or + (32768 / ACC_FS) if it points downwards)
void mpu9250::calibrate(int _T)
{
	xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
	double _dgx = 0, _dgy = 0, _dgz = 0;

	
	delay(1000);
	
	int num = 100;
	
	for(int i = 0; i < num; i++)
	{
		readGyro();
		
		_dgx += xg;
		_dgy += yg;
		_dgz += zg;

		delay(1);
	}

	xGyroOffset = _dgx/num;
	yGyroOffset = _dgy/num;
	zGyroOffset = _dgz/num;
}


void mpu9250::correct()
{
	yawOffset -= yaw / (millis() - correctionTimer);
	correctionTimer = millis();
}


void mpu9250::MadgwickAHRSupdateIMU(double _time)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * xg - q2 * yg - q3 * zg);
	qDot2 = 0.5f * (q0 * xg + q2 * zg - q3 * yg);
	qDot3 = 0.5f * (q0 * yg - q1 * zg + q3 * xg);
	qDot4 = 0.5f * (q0 * zg + q1 * yg - q2 * xg);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((xa == 0.0f) && (ya == 0.0f) && (za == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(xa * xa + ya * ya + za * za);
		xa *= recipNorm;
		ya *= recipNorm;
		za *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * xa + _4q0 * q1q1 - _2q1 * ya;
		s1 = _4q1 * q3q3 - _2q3 * xa + 4.0f * q0q0 * q1 - _2q0 * ya - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * za;
		s2 = 4.0f * q0q0 * q2 + _2q0 * xa + _4q2 * q3q3 - _2q3 * ya - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * za;
		s3 = 4.0f * q1q1 * q3 - _2q1 * xa + 4.0f * q2q2 * q3 - _2q2 * ya;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback stepcorrect
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * _time;
	q1 += qDot2 * _time;
	q2 += qDot3 * _time;
	q3 += qDot4 * _time;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}



float mpu9250::invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}



void mpu9250::updateAngles(double _time, bool flag1)
{
	if(_time == -1)
	{
		_time = double(millis() - time) / 1000;
		time = millis();
		if(_time > 1) _time = 0;
	}
	
	if(flag1)
	{
		readAcc();
		readGyro();
	}
	
	MadgwickAHRSupdateIMU(_time);

	pitch = asin(2*(q0*q2 - q3*q1));
	roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
	yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
	
	pitch = pitch * RAD2DEG - pitchOffset;
	roll = roll * RAD2DEG - rollOffset;
	yaw = yaw * RAD2DEG - yawOffset;
	/*
	while (pitch < -180) pitch += 360;
	while (pitch > 180) pitch -= 360;
	while (roll < -180) roll += 360;
	while (roll > 180) roll -= 360;
	while (yaw < -180) yaw += 360;
	while (yaw > 180) yaw -= 360;
	*/
}



void mpu9250::updateAnglesFromFIFO()
{
	int FIFOsize = readFIFOsize() / 12;			//12 bytes is packet size (6 for gyro and 6 for acc)

	for(int i = 0; i < FIFOsize; i++)
	{
		//readFIFO();
		updateAngles(-1, 1);			//0.001s because gyro and acc sample rate is 1 kHz
	}	
}


////for stm32f407!!!

//#include "mpu9250.h"

//#define RAD2DEG	57.2957795130823208767
//#define DEG2RAD	0.01745329251994329576
//#define beta		0.01f		// 2 * proportional gain



//inline int mpu9250::convert(uint16_t msb, uint8_t lsb)
//{
//	uint16_t a = (msb<<8) | lsb;
//	if(a & 32768) return (a & 32767) - 32768;
//	else return a;
//}


/////volatile int32_t alucard = 0;
//int mpu9250::readReg(int regAddr, int * data, int q)
//{
//	setPin(mpuSSPin, 0);
//	writeSPI(mpuSPI, 0x80 | regAddr);
//	for(int i = 0; i < q; i++)
//		data[i] = writeSPI(mpuSPI, 0);
//	int a = writeSPI(mpuSPI, 0);
//	setPin(mpuSSPin, 1);
//	return a;
//}



//void mpu9250::writeReg(int regAddr, uint8_t data)
//{
//	setPin(mpuSSPin, 0);
//	writeSPI(mpuSPI, regAddr);
//	writeSPI(mpuSPI, data);
//	setPin(mpuSSPin, 1);
//}



//void mpu9250::initIMU(unsigned int _spi, int _ssPin)
//{
//	delay(10);
//	//ACC_DIV = 32768.0 / ACC_FS;
//	//GYRO_DIV = 32768.0 / GYRO_FS / DEG2RAD;
//	ACC_DIV = 1;
//	GYRO_DIV = 1;
//	
//	mpuSPI = _spi;
//	mpuSSPin = _ssPin;
//	xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
//	xAccOffset = 0, yAccOffset = 0, zAccOffset = 0;
//	pitchOffset = 0, rollOffset = 0, yawOffset = 0;
//	
//	xa = 0, ya = 0, za = 0;
//	xg = 0, yg = 0, zg = 0;
//	temp = 0;
//	pitch = 0, roll = 0, yaw = 0;
//	q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
//	time = 0;
//	
//	int accFSsel, gyroFSsel;
//	if(ACC_FS == 2) accFSsel = 0x0;
//	else if(ACC_FS == 4) accFSsel = 0x8;
//	else if(ACC_FS == 8) accFSsel = 0x10;
//	else if(ACC_FS == 16) accFSsel = 0x18;
//	
//	if(GYRO_FS == 250) gyroFSsel = 0x0;
//	else if(GYRO_FS == 500) gyroFSsel = 0x8;
//	else if(GYRO_FS == 1000) gyroFSsel = 0x10;
//	else if(GYRO_FS == 2000) gyroFSsel = 0x18;

//		
//	initSPI(mpuSPI, MASTER, 8, 256);			//init spi
//	initPin(mpuSSPin, OUTPUTPP);
//	setPin(mpuSSPin, 1);
//	
//	writeReg(PWR_MGMT_1, 0x80);			//reset
//	delay(1);
//	writeReg(PWR_MGMT_1, 0x01);			//autoselect the best available clock source
//	delay(1);
//	writeReg(PWR_MGMT_2, 0x00);			//enable all axis of accelerometer and gyroscope
//	delay(1);
//	writeReg(CONFIG, 0x03);						//DLPF (digital low pass filter) = 3
//	delay(1);
//	writeReg(GYRO_CONFIG, gyroFSsel);			//set gyroscope scale, FCHOICE = 2b11 (FCHOICE_b = 2b00)
//	delay(1);
//	writeReg(ACCEL_CONFIG, accFSsel);			//set accelerometer scale
//	delay(1);
//	writeReg(ACCEL_CONFIG_2, 0x03);			//accelerometer DLPF = 3, FCHOICE = 1 (FCHOICE_b = 0)
//	delay(1);
//	writeReg(INT_PIN_CFG, 0x30);				//latch INT pin untill any read operation
//	delay(1);
//	writeReg(USER_CTRL, 0x04);			//enable fifo (first input first output buffer) and reset it
//	delay(1);
//	writeReg(USER_CTRL, 0x40);
//	delay(1);
//	writeReg(FIFO_EN, 0x78);			//write accel and gyro data to FIFO
//	delay(5);
//	
//	setAccOffset(0, 0, 0);
//	setGyroOffset(0, 0, 0);
//	
//	if(_spi == _SPI1)											//set frequency to 17.75 MHz
//	{
//		SPI1->CR1 &= ~SPI_CR1_BR;
//		SPI1->CR1 |= SPI_CR1_BR_1;
//	}
//	else if(_spi == _SPI2)
//	{
//		SPI2->CR1 &= ~SPI_CR1_BR;
//		SPI2->CR1 |= SPI_CR1_BR_1;
//	}
//	else if(_spi == _SPI3)
//	{
//		SPI3->CR1 &= ~SPI_CR1_BR;
//		SPI3->CR1 |= SPI_CR1_BR_1;
//	}
//	correctionTimer = millis();
//}



//void mpu9250::readAcc( )						//reads data directly from accelerometer registers
//{
//	int data[6];
//	readReg(ACCEL_XOUT_H, data, 6);
//	xa = double(double(convert(data[0], data[1])) - xAccOffset) / ACC_DIV;
//	ya = double(double(convert(data[2], data[3])) - yAccOffset) / ACC_DIV;
//	za = double(double(convert(data[4], data[5])) - zAccOffset) / ACC_DIV;
//}



//void mpu9250::readGyro()					//reads data directly from gyroscope registers. Returns values in RAD/S
//{
//	int data[6];
//	readReg(GYRO_XOUT_H, data, 6);
//	xg = double(convert(data[0], data[1]) - xGyroOffset) / GYRO_DIV;
//	yg = double(convert(data[2], data[3]) - yGyroOffset) / GYRO_DIV;
//	zg = double(convert(data[4], data[5]) - zGyroOffset) / GYRO_DIV;
//}



//void mpu9250::readTemp()					//reads data directly from temperature registers
//{
//	temp = (convert(readReg(TEMP_OUT_H), readReg(TEMP_OUT_L)) - 21.0)/333.87 + 21.0;	//21.0 and 333.87 are some magic numbers
//}



//int mpu9250::readFIFOsize()					//reads quantity of bites in FIFO
//{
//	return convert((readReg(FIFO_COUNTH) & 0x1F), readReg(FIFO_COUNTL));
//}



//void mpu9250::readFIFO()					//reads accelerometer and gyroscope data from FIFO
//{
//	int d0 = readReg(FIFO_R_W);			//first writes values to variables because
//	int d1 = readReg(FIFO_R_W);			//just read from fifo rw register is too fast
//	int d2 = readReg(FIFO_R_W);			//(values in that buffer havent enough time to update)
//	int d3 = readReg(FIFO_R_W);			//da, eto kostyl`!
//	int d4 = readReg(FIFO_R_W);
//	int d5 = readReg(FIFO_R_W);
//	int d6 = readReg(FIFO_R_W);
//	int d7 = readReg(FIFO_R_W);
//	int d8 = readReg(FIFO_R_W);
//	int d9 = readReg(FIFO_R_W);
//	int d10 = readReg(FIFO_R_W);
//	int d11 = readReg(FIFO_R_W);
//	xa = double(convert(d0, d1) - xAccOffset) / ACC_DIV;
//	ya = double(convert(d2, d3) - yAccOffset) / ACC_DIV;
//	za = double(convert(d4, d5) - zAccOffset) / ACC_DIV;
//	
//	xg = double(convert(d6, d7) - xGyroOffset) / GYRO_DIV;
//	yg = double(convert(d8, d9) - yGyroOffset) / GYRO_DIV;
//	zg = double(convert(d10, d11) - zGyroOffset) / GYRO_DIV;
//}



//void mpu9250::setGyroOffset(int _xOffset, int _yOffset, int _zOffset)				//sets hardware gyroscope offset for +-1000°/s full-scale
//{
//	writeReg(XG_OFFSET_H, _xOffset >> 8);
//	writeReg(XG_OFFSET_L, _xOffset & 0xFF);
//	writeReg(YG_OFFSET_H, _yOffset >> 8);
//	writeReg(YG_OFFSET_L, _yOffset & 0xFF);
//	writeReg(ZG_OFFSET_H, _zOffset >> 8);
//	writeReg(ZG_OFFSET_L, _zOffset & 0xFF);
//}



//void mpu9250::setAccOffset(int xOffset, int yOffset, int zOffset)				//sets hardware accelerometer offset for +-16g full-scale
//{
//	writeReg(XG_OFFSET_H, xOffset >> 7);
//	writeReg(XG_OFFSET_L, (xOffset & 0x7F) << 1);
//	writeReg(YG_OFFSET_H, yOffset >> 7);
//	writeReg(YG_OFFSET_L, (yOffset & 0x7F) << 1);
//	writeReg(ZG_OFFSET_H, zOffset >> 7);
//	writeReg(ZG_OFFSET_L, (zOffset & 0x7F) << 1);
//}



////For calibration imu must stand still (for finding gyro offsets)
////and pointing acc Z axis straight downwards!!!
////
////If other axis points upperwards
////it`s (n is x, y or z) nAccOffset calculates as 
////_dan/_T - (32768 / ACC_FS) (or + (32768 / ACC_FS) if it points downwards)
//void mpu9250::calibrate(int _T)
//{
//	xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
//	xAccOffset = 0, yAccOffset = 0, zAccOffset = 0;
//	ACC_DIV = 1;
//	GYRO_DIV = 1;
//	long long int _dgx = 0, _dgy = 0, _dgz = 0;
//	long long int _dax = 0, _day = 0, _daz = 0;
//	
//	setGyroOffset(0, 0, 0);
//	setAccOffset(0, 0, 0);
//	
//	long long int realCalibrationTimer = millis();
//	
//	for(int i = 0; i < _T; i++)
//	{
//		readGyro();
//		readAcc();
//		
//		_dgx += xg;
//		_dgy += yg;
//		_dgz += zg;
//		
//		_dax += xa;
//		_day += ya;
//		_daz += za;
//		delay(1);
//	}
//	
//	int realCalibrationTime = millis() - realCalibrationTimer;
//	
//	correctionTimer = millis();
//	
//	ACC_DIV = 32768.0 / ACC_FS;
//	GYRO_DIV = 32768.0 / GYRO_FS / DEG2RAD;

//	xGyroOffset = double(_dgx)/realCalibrationTime;
//	yGyroOffset = double(_dgy)/realCalibrationTime;
//	zGyroOffset = double(_dgz)/realCalibrationTime;
//	
//	xAccOffset = double(_dax)/realCalibrationTime;
//	yAccOffset = double(_day)/realCalibrationTime;
//	zAccOffset = double(_daz)/realCalibrationTime + (32768 / ACC_FS);  //check
//	
//	yaw = 0, pitch = 0, roll = 0;
//}


//void mpu9250::correct()
//{
//	yawOffset -= yaw / (millis() - correctionTimer);
//	correctionTimer = millis();
//}


//void mpu9250::MadgwickAHRSupdateIMU(double _time)
//{
//	float recipNorm;
//	float s0, s1, s2, s3;
//	float qDot1, qDot2, qDot3, qDot4;
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

//	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1 * xg - q2 * yg - q3 * zg);
//	qDot2 = 0.5f * (q0 * xg + q2 * zg - q3 * yg);
//	qDot3 = 0.5f * (q0 * yg - q1 * zg + q3 * xg);
//	qDot4 = 0.5f * (q0 * zg + q1 * yg - q2 * xg);

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((xa == 0.0f) && (ya == 0.0f) && (za == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(xa * xa + ya * ya + za * za);
//		xa *= recipNorm;
//		ya *= recipNorm;
//		za *= recipNorm;   

//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0 = 2.0f * q0;
//		_2q1 = 2.0f * q1;
//		_2q2 = 2.0f * q2;
//		_2q3 = 2.0f * q3;
//		_4q0 = 4.0f * q0;
//		_4q1 = 4.0f * q1;
//		_4q2 = 4.0f * q2;
//		_8q1 = 8.0f * q1;
//		_8q2 = 8.0f * q2;
//		q0q0 = q0 * q0;
//		q1q1 = q1 * q1;
//		q2q2 = q2 * q2;
//		q3q3 = q3 * q3;

//		// Gradient decent algorithm corrective step
//		s0 = _4q0 * q2q2 + _2q2 * xa + _4q0 * q1q1 - _2q1 * ya;
//		s1 = _4q1 * q3q3 - _2q3 * xa + 4.0f * q0q0 * q1 - _2q0 * ya - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * za;
//		s2 = 4.0f * q0q0 * q2 + _2q0 * xa + _4q2 * q3q3 - _2q3 * ya - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * za;
//		s3 = 4.0f * q1q1 * q3 - _2q1 * xa + 4.0f * q2q2 * q3 - _2q2 * ya;
//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;

//		// Apply feedback stepcorrect
//		qDot1 -= beta * s0;
//		qDot2 -= beta * s1;
//		qDot3 -= beta * s2;
//		qDot4 -= beta * s3;
//	}

//	// Integrate rate of change of quaternion to yield quaternion
//	q0 += qDot1 * _time;
//	q1 += qDot2 * _time;
//	q2 += qDot3 * _time;
//	q3 += qDot4 * _time;

//	// Normalise quaternion
//	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//}



//float mpu9250::invSqrt(float x)
//{
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}



//void mpu9250::updateAngles(double _time, bool flag1)
//{
//	if(_time == -1)
//	{
//		_time = double(millis() - time) / 1000;
//		time = millis();
//		if(_time > 1) _time = 0;
//	}
//	
//	if(flag1)
//	{
//		readAcc();
//		readGyro();
//	}
//	
//	MadgwickAHRSupdateIMU(_time);

//	pitch = asin(2*(q0*q2 - q3*q1));
//	roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
//	yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
//	
//	pitch = pitch * RAD2DEG - pitchOffset;
//	roll = roll * RAD2DEG - rollOffset;
//	yaw = yaw * RAD2DEG - yawOffset;
//	/*
//	while (pitch < -180) pitch += 360;
//	while (pitch > 180) pitch -= 360;
//	while (roll < -180) roll += 360;
//	while (roll > 180) roll -= 360;
//	while (yaw < -180) yaw += 360;
//	while (yaw > 180) yaw -= 360;
//	*/
//}



//void mpu9250::updateAnglesFromFIFO()
//{
//	int FIFOsize = readFIFOsize() / 12;			//12 bytes is packet size (6 for gyro and 6 for acc)

//	for(int i = 0; i < FIFOsize; i++)
//	{
//		//readFIFO();
//		updateAngles(-1, 1);			//0.001s because gyro and acc sample rate is 1 kHz
//	}	
//}

double mpu9250::getXa()
{
	return xa;
}

double mpu9250::getYa()
{
	return ya;
}

double mpu9250::getZa()
{
	return za;
}

double mpu9250::getXg()
{
	return xg;
}

double mpu9250::getYg()
{
	return yg;
}

double mpu9250::getZg()
{
	return zg;
}

//////for stm32f407!!!

////#include "mpu9250.h"

////#define RAD2DEG	57.2957795130823208767
////#define DEG2RAD	0.01745329251994329576
////#define beta		0.1f		// 2 * proportional gain



////inline int mpu9250::convert(uint16_t msb, uint8_t lsb)
////{
////	uint16_t a = (msb<<8) | lsb;
////	if(a & 32768) return (a & 32767) - 32768;
////	else return a;
////}


///////volatile int32_t alucard = 0;
////int mpu9250::readReg(int regAddr, int * data, int q)
////{
////	setPin(mpuSSPin, 0);
////	writeSPI(mpuSPI, 0x80 | regAddr);
////	for(int i = 0; i < q; i++)
////		data[i] = writeSPI(mpuSPI, 0);
////	int a = writeSPI(mpuSPI, 0);
////	setPin(mpuSSPin, 1);
////	return a;
////}



////void mpu9250::writeReg(int regAddr, uint8_t data)
////{
////	setPin(mpuSSPin, 0);
////	writeSPI(mpuSPI, regAddr);
////	writeSPI(mpuSPI, data);
////	setPin(mpuSSPin, 1);
////}



////void mpu9250::initIMU(unsigned int _spi, int _ssPin)
////{
////	delay(10);
////	ACC_DIV = 32768.0 / ACC_FS;
////	GYRO_DIV = 32768.0 / GYRO_FS / DEG2RAD;
////	
////	mpuSPI = _spi;
////	mpuSSPin = _ssPin;
////	xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
////	xAccOffset = 0, yAccOffset = 0, zAccOffset = 0;
////	pitchOffset = 0, rollOffset = 0, yawOffset = 0;
////	
////	xa = 0, ya = 0, za = 0;
////	xg = 0, yg = 0, zg = 0;
////	temp = 0;
////	pitch = 0, roll = 0, yaw = 0;
////	q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
////	time = 0;
////	
////	int accFSsel, gyroFSsel;
////	if(ACC_FS == 2) accFSsel = 0x0;
////	else if(ACC_FS == 4) accFSsel = 0x8;
////	else if(ACC_FS == 8) accFSsel = 0x10;
////	else if(ACC_FS == 16) accFSsel = 0x18;
////	
////	if(GYRO_FS == 250) gyroFSsel = 0x0;
////	else if(GYRO_FS == 500) gyroFSsel = 0x8;
////	else if(GYRO_FS == 1000) gyroFSsel = 0x10;
////	else if(GYRO_FS == 2000) gyroFSsel = 0x18;

////		
////	initSPI(mpuSPI, MASTER, 8, 256);			//init spi
////	initPin(mpuSSPin, OUTPUTPP);
////	setPin(mpuSSPin, 1);
////	
////	writeReg(PWR_MGMT_1, 0x80);			//reset
////	delay(1);
////	writeReg(PWR_MGMT_1, 0x00);			//autoselect the best available clock source
////	delay(1);
////	writeReg(PWR_MGMT_2, 0x00);			//enable all axis of accelerometer and gyroscope
////	delay(1);
////	writeReg(CONFIG, 0x03);						//DLPF (digital low pass filter) = 3
////	delay(1);
////	writeReg(GYRO_CONFIG, 0x18);			//set gyroscope scale, FCHOICE = 2b11 (FCHOICE_b = 2b00)
////	delay(1);
////	writeReg(ACCEL_CONFIG, 0x00);			//set accelerometer scale
////	delay(1);
////	
////	writeReg(FIFO_EN, 0x00);			//write accel and gyro data to FIFO
////	delay(1);
////	
////	writeReg(INT_ENABLE, 0x00);
////	delay(1);
////	
////	writeReg(USER_CTRL, 0x04);			//enable fifo (first input first output buffer) and reset it
////	delay(1);
////	
////	writeReg(SMPLRT_DIV, 0x04);
////	delay(1);
////	//writeReg(ACCEL_CONFIG_2, 0x03);			//accelerometer DLPF = 3, FCHOICE = 1 (FCHOICE_b = 0)
////	//delay(1);
////	//writeReg(INT_PIN_CFG, 0x30);				//latch INT pin untill any read operation
////	//delay(1);
////	
////	
//////	writeReg(USER_CTRL, 0x40);
//////	delay(1);
////	for(int iter = 0; iter < 3062; iter++)
////	{
////		writeReg(0x6D, iter >> 8);
////		writeReg(0x6E, iter & 0xFF);
////		writeReg(0x6F, bank[iter]);
////	}
////	
////	delay(1);
////	writeReg(0x70, 0x400 >> 8);	
////	writeReg(0x71, 0x400 & 0xFF);
////	delay(1);
////	
////	//////////////////GYRO_MOUNT_MATRIX_CONFIG
////	writeReg(0x6D, 0x04);
////	delay(1);
////	writeReg(0x6E, 0x26);
////	delay(1);
////	
////	writeReg(0x6F, 0x4C);
////	delay(1);
////	
////	writeReg(0x6F, 0xCD);
////	delay(1);
////	
////	writeReg(0x6F, 0x6C);
////	delay(1);
////	
////	///////////////////GYRO_MOUNT_MATRIX_CONFIG_SIGN
////	writeReg(0x6D, 0x04);
////	delay(1);
////	writeReg(0x6E, 0x56);
////	delay(1);
////	
////	writeReg(0x6F, 0x36);
////	delay(1);
////	
////	writeReg(0x6F, 0x56);
////	delay(1);
////	
////	writeReg(0x6F, 0x76);
////	delay(1);
////	
////	////////////////////////ACCEL_MOUNT_MATRIX_CONFIG
////	writeReg(0x6D, 0x04);
////	delay(1);
////	writeReg(0x6E, 0x2A);
////	delay(1);
////	
////	writeReg(0x6F, 0x0C);
////	delay(1);
////	
////	writeReg(0x6F, 0xC9);
////	delay(1);
////	
////	writeReg(0x6F, 0x2C);
////	delay(1);
////	
////	/////////////////////////ACCEL_MOUNT_MATRIX_CONFIG_SIGN
////	writeReg(0x6D, 0x04);
////	delay(1);
////	writeReg(0x6E, 0x34);
////	delay(1);
////	
////	writeReg(0x6F, 0x26);
////	delay(1);
////	
////	writeReg(0x6F, 0x46);
////	delay(1);
////	
////	writeReg(0x6F, 0x66);
////	delay(1);
////	
////	writeReg(USER_CTRL, 0x40);			//enable fifo (first input first output buffer) and reset it
////	delay(1);
////	writeReg(USER_CTRL, 0x04);			//enable fifo (first input first output buffer) and reset it
////	delay(1);
////	writeReg(USER_CTRL, 0x80);			//enable fifo (first input first output buffer) and reset it
////	delay(1);
////	writeReg(USER_CTRL, 0x08);			//enable fifo (first input first output buffer) and reset it
////	delay(1);
////	
////	writeReg(INT_ENABLE, 0x02);
////	delay(1);	
//////	writeReg(0x6D, 0x04);
//////	delay(1);
//////	
//////	writeReg(0x6E, 0x26);
//////	delay(1);

//////	writeReg(0x6F, 0x4C);
//////	delay(1);
//////	writeReg(0x6F, 0xCD);
//////	delay(1);
//////	writeReg(0x6F, 0x6C);
//////	delay(1);
////	
////	////
//////	writeReg(CFG_8, 0x20);
//////	delay(1);
//////	writeReg(CFG_8, 0x28);
//////	delay(1);
//////	writeReg(CFG_8, 0x30);
//////	delay(1);
//////	writeReg(CFG_8, 0x38);
//////	delay(1);
//////	
//////	
//////	
//////	writeReg(D_0_22, 0x00);
//////	delay(1);
//////	writeReg(D_0_22, 0x00);
//////	delay(1);
//////	
//////	for(int iter = 0; iter < 12; iter++)
//////	{
//////		writeReg(CFG_6, regs_end[iter]);
//////		delay(1);
//////	}
////    
////	//writeReg(0x68, 0x38);
////	//delay(1);
////	
////	writeReg(FIFO_EN, 0x78);			//write accel and gyro data to FIFO
////	delay(5);
////	
////	
////	
////	
////	if(_spi == _SPI1)											//set frequency to 17.75 MHz
////	{
////		SPI1->CR1 &= ~SPI_CR1_BR;
////		SPI1->CR1 |= SPI_CR1_BR_1;
////	}
////	else if(_spi == _SPI2)
////	{
////		SPI2->CR1 &= ~SPI_CR1_BR;
////		SPI2->CR1 |= SPI_CR1_BR_1;
////	}
////	else if(_spi == _SPI3)
////	{
////		SPI3->CR1 &= ~SPI_CR1_BR;
////		SPI3->CR1 |= SPI_CR1_BR_1;
////	}
////	correctionTimer = millis();
////}



////void mpu9250::readAcc( )						//reads data directly from accelerometer registers
////{
////	int data[6];
////	readReg(ACCEL_XOUT_H, data, 6);
////	xa = double(convert(data[0], data[1]) - xAccOffset) / ACC_DIV;
////	ya = double(convert(data[2], data[3]) - yAccOffset) / ACC_DIV;
////	za = double(convert(data[4], data[5]) - zAccOffset) / ACC_DIV;
////}



////void mpu9250::readGyro()					//reads data directly from gyroscope registers. Returns values in RAD/S
////{
////	int data[6];
////	readReg(GYRO_XOUT_H, data, 6);
////	xg = double(convert(data[0], data[1]) - xGyroOffset) / GYRO_DIV;
////	yg = double(convert(data[2], data[3]) - yGyroOffset) / GYRO_DIV;
////	zg = double(convert(data[4], data[5]) - zGyroOffset) / GYRO_DIV;
////}



////void mpu9250::readTemp()					//reads data directly from temperature registers
////{
////	temp = (convert(readReg(TEMP_OUT_H), readReg(TEMP_OUT_L)) - 21.0)/333.87 + 21.0;	//21.0 and 333.87 are some magic numbers
////}



////int mpu9250::readFIFOsize()					//reads quantity of bites in FIFO
////{
////	return convert((readReg(FIFO_COUNTH) & 0x1F), readReg(FIFO_COUNTL));
////}



////void mpu9250::readFIFO()					//reads accelerometer and gyroscope data from FIFO
////{
////		volatile uint8_t data_fifo[48];
////		data_fifo[0] = readReg(FIFO_R_W);			//first writes values to variables because
////		data_fifo[1] = readReg(FIFO_R_W);			//just read from fifo rw register is too fast
////		data_fifo[2] = readReg(FIFO_R_W);			//(values in that buffer havent enough time to update)
////		data_fifo[3] = readReg(FIFO_R_W);			//da, eto kostyl`!
////		data_fifo[4] = readReg(FIFO_R_W);
////		data_fifo[5] = readReg(FIFO_R_W);
////		data_fifo[6] = readReg(FIFO_R_W);
////		data_fifo[7] = readReg(FIFO_R_W);
////		data_fifo[8] = readReg(FIFO_R_W);
////		data_fifo[9] = readReg(FIFO_R_W);
////		data_fifo[10] = readReg(FIFO_R_W);
////		data_fifo[11] = readReg(FIFO_R_W);
////		data_fifo[12] = readReg(FIFO_R_W);
////		data_fifo[13] = readReg(FIFO_R_W);
////		data_fifo[14] = readReg(FIFO_R_W);
////		data_fifo[15] = readReg(FIFO_R_W);
////	//

////		data_fifo[16]  = readReg(FIFO_R_W);			//first writes values to variables because
////		data_fifo[17]  = readReg(FIFO_R_W);			//just read from fifo rw register is too fast
////		data_fifo[18]  = readReg(FIFO_R_W);			//(values in that buffer havent enough time to update)
////		data_fifo[19]  = readReg(FIFO_R_W);			//da, eto kostyl`!
////		data_fifo[20]  = readReg(FIFO_R_W);
////		data_fifo[21]  = readReg(FIFO_R_W);
////		data_fifo[22]  = readReg(FIFO_R_W);
////		data_fifo[23]  = readReg(FIFO_R_W);
////		data_fifo[24]  = readReg(FIFO_R_W);
////		data_fifo[25]  = readReg(FIFO_R_W);
////		data_fifo[26]  = readReg(FIFO_R_W);
////		data_fifo[27]  = readReg(FIFO_R_W);
////		
////		data_fifo[28]  = readReg(FIFO_R_W);			//first writes values to variables because
////		data_fifo[29]  = readReg(FIFO_R_W);			//just read from fifo rw register is too fast
////		data_fifo[30]  = readReg(FIFO_R_W);			//(values in that buffer havent enough time to update)
////		data_fifo[31]  = readReg(FIFO_R_W);			//da, eto kostyl`!
////	
////		
////	//	q0 = ((long)d0 << 24) | ((long)d1 << 16) |
////	//				((long)d2 << 8) | d3;
////	//	q1 = ((long)d4 << 24) | ((long)d5 << 16) |
////	//				((long)d6 << 8) | d7;
////	//	q2 = ((long)d8 << 24) | ((long)d9 << 16) |
////	//				((long)d10 << 8) | d11;
////	//	q3 = ((long)d12 << 24) | ((long)d13 << 16) |
////	//				((long)d14 << 8) | d15;
////		xa = double(convert(data_fifo[16], data_fifo[17]) - xAccOffset) / ACC_DIV;
////		ya = double(convert(data_fifo[18], data_fifo[19]) - yAccOffset) / ACC_DIV;
////		za = double(convert(data_fifo[20], data_fifo[21]) - zAccOffset) / ACC_DIV;
////	//	
////		xg = double(convert(data_fifo[22], data_fifo[23]) - xGyroOffset) / GYRO_DIV;
////		yg = double(convert(data_fifo[24], data_fifo[25]) - yGyroOffset) / GYRO_DIV;
////		zg = double(convert(data_fifo[26], data_fifo[27]) - zGyroOffset) / GYRO_DIV;
////}



////void mpu9250::setGyroOffset(int _xOffset, int _yOffset, int _zOffset)				//sets hardware gyroscope offset for +-1000°/s full-scale
////{
////	writeReg(XG_OFFSET_H, _xOffset >> 8);
////	writeReg(XG_OFFSET_L, _xOffset & 0xFF);
////	writeReg(YG_OFFSET_H, _yOffset >> 8);
////	writeReg(YG_OFFSET_L, _yOffset & 0xFF);
////	writeReg(ZG_OFFSET_H, _zOffset >> 8);
////	writeReg(ZG_OFFSET_L, _zOffset & 0xFF);
////}



////void mpu9250::setAccOffset(int xOffset, int yOffset, int zOffset)				//sets hardware accelerometer offset for +-16g full-scale
////{
////	writeReg(XG_OFFSET_H, xOffset >> 7);
////	writeReg(XG_OFFSET_L, (xOffset & 0x7F) << 1);
////	writeReg(YG_OFFSET_H, yOffset >> 7);
////	writeReg(YG_OFFSET_L, (yOffset & 0x7F) << 1);
////	writeReg(ZG_OFFSET_H, zOffset >> 7);
////	writeReg(ZG_OFFSET_L, (zOffset & 0x7F) << 1);
////}



//////For calibration imu must stand still (for finding gyro offsets)
//////and pointing acc Z axis straight downwards!!!
//////
//////If other axis points upperwards
//////it`s (n is x, y or z) nAccOffset calculates as 
//////_dan/_T - (32768 / ACC_FS) (or + (32768 / ACC_FS) if it points downwards)
////void mpu9250::calibrate(int _T)
////{
////	xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
////	xAccOffset = 0, yAccOffset = 0, zAccOffset = 0;
////	ACC_DIV = 1;
////	GYRO_DIV = 1;
////	long long int _dgx = 0, _dgy = 0, _dgz = 0;
////	long long int _dax = 0, _day = 0, _daz = 0;
////	
////	setGyroOffset(0, 0, 0);
////	setAccOffset(0, 0, 0);
////	
////	long long int realCalibrationTimer = millis();
////	
////	for(int i = 0; i < _T; i++)
////	{
////		readGyro();
////		readAcc();
////		
////		_dgx += xg;
////		_dgy += yg;
////		_dgz += zg;
////		
////		_dax += xa;
////		_day += ya;
////		_daz += za;
////		delay(1);
////	}
////	
////	int realCalibrationTime = millis() - realCalibrationTimer;
////	
////	correctionTimer = millis();
////	
////	ACC_DIV = 32768 / ACC_FS;
////	GYRO_DIV = 32768 / GYRO_FS / DEG2RAD;

////	xGyroOffset = double(_dgx)/realCalibrationTime;
////	yGyroOffset = double(_dgy)/realCalibrationTime;
////	zGyroOffset = double(_dgz)/realCalibrationTime;
////	
////	xAccOffset = double(_dax)/realCalibrationTime;
////	yAccOffset = double(_day)/realCalibrationTime;
////	zAccOffset = double(_daz)/realCalibrationTime + (32768 / ACC_FS);
////	
////	yaw = 0, pitch = 0, roll = 0;
////}


////void mpu9250::correct()
////{
////	yawOffset -= yaw / (millis() - correctionTimer);
////	correctionTimer = millis();
////}


////float mpu9250::qToFloat(long number, unsigned char q)
////{
////	unsigned long mask;
////	for (int i=0; i<q; i++)
////	{
////		mask |= (1<<i);
////	}
////	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
////}

////void mpu9250::computeEulerAngles(bool degrees)
////{
////    float dqw = qToFloat(q0, 30);
////    float dqx = qToFloat(q1, 30);
////    float dqy = qToFloat(q2, 30);
////    float dqz = qToFloat(q3, 30);
////    
////    float ysqr = dqy * dqy;
////    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
////    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
////    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
////    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
////    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
////  
////	// Keep t2 within range of asin (-1, 1)
////    t2 = t2 > 1.0f ? 1.0f : t2;
////    t2 = t2 < -1.0f ? -1.0f : t2;
////  
////    pitch = asin(t2) * 2;
////    roll = atan2(t3, t4);
////    yaw = atan2(t1, t0);
////	
////	if (degrees)
////	{
////		pitch *= (180.0 / 3.14);
////		roll *= (180.0 / 3.14);
////		yaw *= (180.0 / 3.14);
////		if (pitch < 0) pitch = 360.0 + pitch;
////		if (roll < 0) roll = 360.0 + roll;
////		if (yaw < 0) yaw = 360.0 + yaw;	
////	}
////}



////void mpu9250::MadgwickAHRSupdateIMU(double _time)
////{
////	float recipNorm;
////	float s0, s1, s2, s3;
////	float qDot1, qDot2, qDot3, qDot4;
////	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

////	// Rate of change of quaternion from gyroscope
////	qDot1 = 0.5f * (-q1 * xg - q2 * yg - q3 * zg);
////	qDot2 = 0.5f * (q0 * xg + q2 * zg - q3 * yg);
////	qDot3 = 0.5f * (q0 * yg - q1 * zg + q3 * xg);
////	qDot4 = 0.5f * (q0 * zg + q1 * yg - q2 * xg);

////	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
////	if(!((xa == 0.0f) && (ya == 0.0f) && (za == 0.0f))) {

////		// Normalise accelerometer measurement
////		recipNorm = invSqrt(xa * xa + ya * ya + za * za);
////		xa *= recipNorm;
////		ya *= recipNorm;
////		za *= recipNorm;   

////		// Auxiliary variables to avoid repeated arithmetic
////		_2q0 = 2.0f * q0;
////		_2q1 = 2.0f * q1;
////		_2q2 = 2.0f * q2;
////		_2q3 = 2.0f * q3;
////		_4q0 = 4.0f * q0;
////		_4q1 = 4.0f * q1;
////		_4q2 = 4.0f * q2;
////		_8q1 = 8.0f * q1;
////		_8q2 = 8.0f * q2;
////		q0q0 = q0 * q0;
////		q1q1 = q1 * q1;
////		q2q2 = q2 * q2;
////		q3q3 = q3 * q3;

////		// Gradient decent algorithm corrective step
////		s0 = _4q0 * q2q2 + _2q2 * xa + _4q0 * q1q1 - _2q1 * ya;
////		s1 = _4q1 * q3q3 - _2q3 * xa + 4.0f * q0q0 * q1 - _2q0 * ya - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * za;
////		s2 = 4.0f * q0q0 * q2 + _2q0 * xa + _4q2 * q3q3 - _2q3 * ya - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * za;
////		s3 = 4.0f * q1q1 * q3 - _2q1 * xa + 4.0f * q2q2 * q3 - _2q2 * ya;
////		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
////		s0 *= recipNorm;
////		s1 *= recipNorm;
////		s2 *= recipNorm;
////		s3 *= recipNorm;

////		// Apply feedback stepcorrect
////		qDot1 -= beta * s0;
////		qDot2 -= beta * s1;
////		qDot3 -= beta * s2;
////		qDot4 -= beta * s3;
////	}

////	// Integrate rate of change of quaternion to yield quaternion
////	q0 += qDot1 * _time;
////	q1 += qDot2 * _time;
////	q2 += qDot3 * _time;
////	q3 += qDot4 * _time;

////	// Normalise quaternion
////	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
////	q0 *= recipNorm;
////	q1 *= recipNorm;
////	q2 *= recipNorm;
////	q3 *= recipNorm;
////}



////float mpu9250::invSqrt(float x)
////{
////	float halfx = 0.5f * x;
////	float y = x;
////	long i = *(long*)&y;
////	i = 0x5f3759df - (i>>1);
////	y = *(float*)&i;
////	y = y * (1.5f - (halfx * y * y));
////	return y;
////}



////void mpu9250::updateAngles(double _time, bool flag1)
////{
////	if(_time == -1)
////	{
////		_time = double(millis() - time) / 1000;
////		time = millis();
////		//if(_time > 1) _time = 0;
////	}
////	
////	if(flag1)
////	{
////		readAcc();
////		readGyro();
////	}
////	
////	MadgwickAHRSupdateIMU(_time);
////	//computeEulerAngles(true);
////	pitch = asin(2*(q0*q2 - q3*q1));
////	roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
////	yaw = -atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
////	
////	pitch = pitch * RAD2DEG - pitchOffset;
////	roll = roll * RAD2DEG - rollOffset;
////	yaw = yaw * RAD2DEG - yawOffset;
////	/*
////	while (pitch < -180) pitch += 360;
////	while (pitch > 180) pitch -= 360;
////	while (roll < -180) roll += 360;
////	while (roll > 180) roll -= 360;
////	while (yaw < -180) yaw += 360;
////	while (yaw > 180) yaw -= 360;
////	*/
////}



////void mpu9250::updateAnglesFromFIFO()
////{
////	int FIFOsize = readFIFOsize() / 32;			//12 bytes is packet size (6 for gyro and 6 for acc)

////	for(int i = 0; i < FIFOsize; i++)
////	{
////		readFIFO();
////		updateAngles(0.02, 1);			//0.001s because gyro and acc sample rate is 1 kHz
////	}	
////}


////double mpu9250::getXa()
////{
////	return xa;
////}

////double mpu9250::getYa()
////{
////	return ya;
////}

////double mpu9250::getZa()
////{
////	return za;
////}
