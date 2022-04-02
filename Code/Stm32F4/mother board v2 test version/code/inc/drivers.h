#ifndef DRIVERS_LIB
#define DRIVERS_LIB

int LOW_VOLTAGE_COUNTER = 0;

/* DRIVER_REGISTER */

#define D_WRITE 		3
#define D_READ 			6
#define D_WRITE_ANS 163
#define D_READ_ANS 	166
#define D_WRITE_ERR 227
#define D_READ_ERR  230


/* DRIVERS */ 
class Driver
{
	public:
		char uart, addr, reg[21];
		char msg[4];
	
		int velocity, current, temperature, voltage, cTime;
		uint64_t lastUpdateTime;
    int64_t last;	
		bool floating;
	
		Driver();
		void init(int cTime, char selfAddr, char uartAddr);
	  void write(char* data, char len, char messageType);
		void writeReg(char addr, char val);
		void updateFromReg();
	  void setVelocity(int spd);
	  void attemptTemp();
	  void attemptVelocity();
	  void attemptVoltage();
	  void attemptCurrent();
		bool isWorks();
		void logError();
	void enableFloating();
	void disableFloating();
};

bool Driver::isWorks()
{
	return (millis() - lastUpdateTime) < 200;
}

void Driver::logError()
{
	return;
	
	if (last == -1) last = millis() + cTime;
	/*
	if (cTime == 0)
		LOW_VOLTAGE_COUNTER = 0;
	*/
	if (millis() - last > 500)
	{
		last = millis() + cTime;
		delay(2);
		attemptVoltage();
		delay(2);
	}
	/*
	if (voltage <= 13 && voltage != 0)					//voltage is checked within Robot::updateBatteryVoltage() in functional.h
	{
		LOW_VOLTAGE_COUNTER++;
		if (LOW_VOLTAGE_COUNTER == 4)
		{
			GLOBAL_ERROR |= LOW_POWER;
		}
	}
	*/
	if (!isWorks()) 
	{
		GLOBAL_ERROR |= DRIVER_CONNECTION_ERROR;
		//printUART(DEBUG_UART, int(addr));
	}
}

void Driver::updateFromReg()
{
	lastUpdateTime = millis();
	/* Velocity setup */
	int r0 = reg[0], r1 = reg[1];
	if (r1 >= 128)
		velocity =  (-3) * ((256 - r0) + (255 - r1) * 256);
	else
		velocity = 3 * (r0 + r1 * 256);
	
	temperature = reg[3];
	voltage = reg[5];
	
	int r2 = reg[2];
	current = float(r2 >= 128 ? (255 - r2) : r2) / 1.28f;
}

Driver::Driver()
{
	addr = 127;
	velocity = 0;
	temperature = 0;
	current = 0;
	voltage = 16;
	last = -1;
	
	msg[0] = 128;
	msg[1] = 0;
	msg[2] = 129;
	msg[3] = 0;
	
	for (int i = 0; i < 21; i++) reg[i] = 0;
	
	cTime = 0;
	floating = false;
}

void Driver::init(int cTime, char selfAddr, char UARTAddr)
{
	this->cTime = cTime;
	uart = UARTAddr;
	addr = selfAddr;
}

void Driver::write(char* data, char len, char messageType)
{
	char wholeLen = len + 6;
	static char message[16];
	
	message[0] = 0xBB;
	message[1] = addr;
	message[2] = wholeLen;
	message[3] = crc8(message, 3);
	message[4] = messageType;
	
	for (char i = 0; i < len; i++) message[5 + i] = data[i];
	message[wholeLen - 1] = crc8(message, wholeLen - 1);
	
	for (char i = 0; i < wholeLen; i++) writeUART(uart, message[i]);
	
}

void Driver::enableFloating()
{
	if(!floating)
	{
		static char m[2] = {0x89, 1};
		write(m, 2, D_WRITE);
		floating = true;
	}
}

void Driver::disableFloating()
{	
	if(floating)
	{
		static char m[2] = {0x89, 0};
		write(m, 2, D_WRITE);
		floating = false;
	}
}

void Driver::setVelocity(int spd)
{
	if(floating) spd = 0;
	
	spd = ceil(float(spd) / 3.0f);
	if (spd > 2900) spd =  2900;
	if (spd < -2900) spd = -2900;
	
	msg[1] = abs(spd) % 255;
	msg[3] = (abs(spd) >> 8) % 128;
	
	if (spd < 0) 
	{
		msg[1] = -msg[1];
		msg[3] = -msg[3] - 1;
	}
	
	write(msg, 4, D_WRITE);
}

void Driver::attemptVelocity()
{
	static char msg[2] = {0, 1};
	write(msg, 2, D_READ);
}

void Driver::attemptTemp()
{
	static char msg[1] = {3};
	write(msg, 1, D_READ);
}

void Driver::attemptVoltage()
{
	static char msg[1] = {5};
	write(msg, 1, D_READ);
}

void Driver::attemptCurrent()
{
	static char msg[1] = {2};
	write(msg, 1, D_READ);
}

#endif
