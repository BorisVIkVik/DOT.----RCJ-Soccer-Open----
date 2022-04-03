class Robot
{
	public:
		mpu9250 imu;
		Display display;
		VimTransaction target;
		double angle, calibratedAngle, dx, dy;
		bool kick1, kick2;
		long long kicker1Timer, kicker2Timer;
		int driblerSpeed1, driblerSpeed2;
		double imuFloatValue, angleChange;
		long long int imuFloatTime;
		PL_ADC ADC_1, ADC_2;
		uint8_t line, lineBoardTx;
		Driver motors[4];
		Button button[3];
		double cells[4];
		int ball[2];
		int oldMotorSpeed[4];
		bool manualDribblerControl;
		double batteryVoltage;
		bool lowBatteryVoltageError;
		long long int lowBatteryVoltageTimer;
	
		void setMotors(int v1, int v2, int v3, int v4);
		void enableMotors();
		void disableMotors();
		void setMotor(int i, int vel);
		void updateIMU();
		void updateDrivers();
		void wait(uint32_t t);
		void logErrors();
		void init();
		void move(double velocity, double dir, double heading = 0, double acc = 4 /* m/(s^2) */, double smooth = 1 /* ob/s */);
		void setupIMU();
		void sendFlagToLine(uint8_t flag);
		void updateLine();
		void updateButtons();
		void beginLineCalibration();
		void endLineCalibration();
		void cycleEnd();
		void updateVIM();
		void sendCommandToVIM(char reg, bool val);
		void sync();
		void syncVIM();
		void calibrateIMU(uint32_t t);
		void updateMenu();
		void updateKickers();
		void updateDribblers(int8_t power1);
		void changePlayState();
		void setPlayState(bool a);
		bool playState();
		void updateBatteryVoltage();
		
		int ball1();
		int ball2();
		
		double cell1();
		double cell2();
		double cell3();
		double cell4();
		
		void updateCells();
		void updateBallSensors();
		
		void print(char* x, int row = 0, int column = 0);
		void print(int x, int row = 0, int column = 0, int next = 3);
		void print(unsigned x, int row = 0, int column = 0, int next = 3);
		void print(long long x, int row = 0, int column = 0, int next = 3);
		void print(unsigned long long x, int row = 0, int column = 0, int next = 3);
		void print(double x, int row = 0, int column = 0, int next = 3);
		void print(float x, int row = 0, int column = 0, int next = 3);
		
	private:
		bool playStateBool;
		long long int playStateTimer;
};

void Robot::init()
{
	//dribblers start sequence
	initPWM(DRIBLER_1_TIM, DRIBLER_1_CH, DRIBLER_1_PIN, SERVO_FREQ);
	initPWM(DRIBLER_2_TIM, DRIBLER_2_CH, DRIBLER_2_PIN, SERVO_FREQ);
	setPWM(DRIBLER_1_TIM, DRIBLER_1_CH, 180);
	setPWM(DRIBLER_2_TIM, DRIBLER_2_CH, 180);
	delay(250);
	setPWM(DRIBLER_1_TIM, DRIBLER_1_CH, 0);
	setPWM(DRIBLER_2_TIM, DRIBLER_2_CH, 0);
	delay(250);
	setPWM(DRIBLER_1_TIM, DRIBLER_1_CH, 90);
	setPWM(DRIBLER_2_TIM, DRIBLER_2_CH, 90);
	delay(1000);
	
	kick1 = 0;
	kick2 = 0;
	kicker1Timer = millis();
	kicker2Timer = millis();
	
	imuFloatTime = millis();
	imuFloatValue = 0;
	
	initLEDs();
	initButtons();
	initPin(IMU_PWR_EN, OUTPUTPP);
	setPin(IMU_PWR_EN, 1);
	delay(10);
	imu.initIMU(IMU_SPI, IMU_SPI_SS);
	delay(100);
	setupIMU(); // IMU calibrated angle estimating
	
	
	initUART(MOTOR_1_UART, 115200, 8, 1, NO_PARITY, 42);
	initUART(MOTOR_2_UART, 115200, 8, 1, NO_PARITY, 42);
	initUART(MOTOR_3_UART, 115200, 8, 1, NO_PARITY, 84);
	initUART(MOTOR_4_UART, 115200, 8, 1, NO_PARITY, 84);
	initUART(DEBUG_UART, 115200, 8, 0, NO_PARITY, 42);
	initUART(VIM_UART, 115200, 8, 1, NO_PARITY, 42);
	
	delay(300); // small delay for successfull setup of UARTs
	
	initSPI(LINE_SENSORS_SPI, MASTER, 8, 8);
	initPin(LINE_SENSORS_SS, OUTPUTPP);
	setPin(LINE_SENSORS_SS, 1);
	
	//initSPI(KICKER_MODULE_SPI, MASTER, 8, 8); same as LINE_SENSORS_SPI
	initPin(KICKER_MODULE_SS, OUTPUTPP);
	setPin(KICKER_MODULE_SS, 1);
	
	
	motors[0].init(0, MOTOR_1, MOTOR_1_UART);
	wait(3);
	motors[1].init(100, MOTOR_2, MOTOR_2_UART);
	wait(3);
	motors[2].init(200, MOTOR_3, MOTOR_3_UART);
	wait(3);
	motors[3].init(300, MOTOR_4, MOTOR_4_UART);
	wait(3);
	
	for (int i = 0; i < 4; i++)
	{
		motors[i].setVelocity(0);
		wait(5);
	}
	
	oldMotorSpeed[0] = 0;
	oldMotorSpeed[1] = 0;
	oldMotorSpeed[2] = 0;
	oldMotorSpeed[3] = 0;
	manualDribblerControl = 0;
	
	dx = 0;
	dy = 0;
	lineBoardTx = 0xBB;
	
	stm32f407_SSD1306 disp(DISPLAY_SPI, DISPLAY_DC, DISPLAY_RESET, DISPLAY_CS, DISPLAY_PWR_EN, DISPLAY_SCK, DISPLAY_MOSI);
	
	disp.begin();
	disp.clear();
	disp.invert(0);
	disp.rotate(2);
	this->display.display = disp;
	
	playStateBool = false;
	playStateTimer = millis();
	
	batteryVoltage = 10.0;
	lowBatteryVoltageTimer = millis();
	
	// Setup a bunch of some pins like power for ball sensors, smth about batteries pins etc
	initPin(BALL_SENSOR_2_EN, OUTPUTPP);
	setPin(BALL_SENSOR_2_EN, 0);
	initPin(BALL_SENSOR_1_EN, OUTPUTPP);
	setPin(BALL_SENSOR_1_EN, 0);
	
	initPin(BATTERY_S4_EN, OUTPUTPP);
	setPin(BATTERY_S4_EN, 1);
	initPin(BATTERY_S2_EN, OUTPUTPP);
	setPin(BATTERY_S2_EN, 1);
	initPin(BATTERY_S1_EN, OUTPUTPP);
	setPin(BATTERY_S1_EN, 1);
	initPin(BATTERY_S3_EN, OUTPUTPP);
	setPin(BATTERY_S3_EN, 1);
	
	
	ADC_1 = *(new PL_ADC(ADC1));
	ADC_1.init();
	ADC_1.add(BATTERY_S3);
	ADC_1.add(BATTERY_S2);
	ADC_1.add(BATTERY_S1);
	ADC_1.add(BATTERY_S4);
	ADC_1.start();
	
	ADC_2 = *(new PL_ADC(ADC2));
	ADC_2.init();
	ADC_2.add(BALL_SENSOR_2);
	ADC_2.add(BALL_SENSOR_1);
	ADC_2.start();
	
	
	driblerSpeed1 = 0;
	driblerSpeed2 = 0;
	
	angleChange = 0;
	
	disableButtons = false;
}



void Robot::changePlayState()
{
	if(playStateBool) setPlayState(0);
	else  setPlayState(1);
}

void Robot::setPlayState(bool a)
{
	if(a != playStateBool)
	{
		if(millis() - playStateTimer > 75)
		{
			playStateBool = a;
			playStateTimer = millis();
		}
	}
}

bool Robot::playState()
{
	return playStateBool;
}

int Robot::ball1()
{
	return ADC_2.read(BALL_SENSOR_1) > 2980;	
}

int Robot::ball2()
{
	return ADC_2.read(BALL_SENSOR_2) > 2900;	
}

double Robot::cell1()
{
	return double(ADC_1.read(BATTERY_S1)) * 0.001098;	//* 3.3 * double(1200 + 3300) / double(4096 * 3300);
}

double Robot::cell2()
{
	return double(ADC_1.read(BATTERY_S2)) * 0.002173;	//* 3.3 * double(5600 + 3300) / double(4096 * 3300);
}

double Robot::cell3()
{
	return double(ADC_1.read(BATTERY_S3)) * 0.003521;	//* 3.3 * double(9100 + 2700) / double(4096 * 2700);
}

double Robot::cell4()
{
	return double(ADC_1.read(BATTERY_S4)) * 0.004685;	//* 3.3 * double(13000 + 2700) / double(4096 * 2700);
}

void Robot::updateCells()
{
	double _c1 = cell1();
	double _c2 = cell2();
	double _c3 = cell3();
	double _c4 = cell4();
	cells[0] = _c1;
	cells[1] = _c2 - _c1;
	cells[2] = _c3 - _c2;
	cells[3] = _c4 - _c3;
}

void Robot::updateBatteryVoltage()
{
	batteryVoltage = cells[0] + cells[1] + cells[2] + cells[3];
	
	bool lowCellVoltage = 0;
	
	for (int i = 0; i < 4; i++)
		if (cells[i] < 3.3)
			lowCellVoltage = 1;
	
	if(batteryVoltage < 13.8 || lowCellVoltage)
	{
		if(GLOBAL_ERROR & LOW_BATTERY_POWER == 0) lowBatteryVoltageTimer = millis();
		
		if(millis() - lowBatteryVoltageTimer > 750) GLOBAL_ERROR |= LOW_BATTERY_POWER;
	}
	else
	{
		GLOBAL_ERROR &= ~LOW_BATTERY_POWER;
	}
}

void Robot::updateBallSensors()
{
	ball[0] = ball1();
	ball[1] = ball2();
}

void Robot::disableMotors()
{
	for (int i = 0; i < 4; i++)
	{
		motors[i].enableFloating();
	}
}

void Robot::enableMotors()
{
	for (int i = 0; i < 4; i++)
	{
		motors[i].disableFloating();
	}
}

void Robot::syncVIM()
{
	double dAngle = angle - calibratedAngle;
	while(dAngle < 0) dAngle += 360;
	while(dAngle >= 360) dAngle -= 360;
	
	unsigned char a1 = (int)(dAngle * 256.0 / 360.0);
	unsigned char a2 = (int)((dAngle * 256.0 / 360.0 - (int)a1) * 256.0 * 256.0 / 360.0);
	
	writeStrUART(DEBUG_UART, "data: ");
	printUART(DEBUG_UART, (int)a1);
	writeStrUART(DEBUG_UART, " ");
	printUART(DEBUG_UART, (int)a2);
	writeStrUART(DEBUG_UART, " | ");
	printUART(DEBUG_UART, dAngle * 256.0 / 360.0 - int(a1));
	writeStrUART(DEBUG_UART, "\n");
	
	
	char msg[] = {0xBB, a1, a2, int(playState()) + int(ball[0]) * 2 + int(ball[1]) * 4};
	for (int i = 0; i < 4; i++)
	    writeUART(VIM_UART, msg[i]);
	writeUART(VIM_UART, crc8(msg, 4));
	
	cycleEnd();
}

void Robot::setMotor(int i, int vel)
{
	if(vel != oldMotorSpeed[i])
	{
		oldMotorSpeed[i] = vel;
		motors[i].setVelocity(vel);
	}	
}

void Robot::setMotors(int v1, int v2, int v3, int v4)
{
	setMotor(0, v1);
	setMotor(1, v2);
	setMotor(2, v3);
	setMotor(3, v4);
	
	while(!UARTTxFinished(motors[0].uart) || !UARTTxFinished(motors[1].uart) || !UARTTxFinished(motors[2].uart) || !UARTTxFinished(motors[3].uart));
	wait(2);
}

void Robot::updateButtons()
{
	button[0].setState(readPin(BUTTON_1));
	button[1].setState(readPin(BUTTON_2));
	button[2].setState(readPin(BUTTON_3));
}

void Robot::updateKickers()
{
	uint8_t kickerTxData = 0;
	
	if (kick1 && millis() - kicker1Timer > 500)
	{
		kickerTxData = kickerTxData | 1;
		kick1 = 0;
		kicker1Timer = millis();
	}
	if (kick2 && millis() - kicker2Timer > 500)
	{
		kickerTxData = kickerTxData | 128;
		kick2 = 0;
		kicker2Timer = millis();
	}
	
	if(kickerTxData)
	{
		setPin(KICKER_MODULE_SS, 0);
		writeSPI(KICKER_MODULE_SPI, kickerTxData);
		setPin(KICKER_MODULE_SS, 1);
	}
}

void Robot::updateDribblers(int8_t power1)
{
	// int power1 = driblerSpeed1;
	// int power2 = driblerSpeed2;
	
	/*
	if(power1 == 0)
		TIM1 -> CCER &= ~TIM_CCER_CC3E;				//disable output
	else 
		TIM1 -> CCER |= TIM_CCER_CC3E;					//enable output
	
	if(power2 == 0)
		TIM1 -> CCER &= ~TIM_CCER_CC4E;
	else
		TIM1 -> CCER |= TIM_CCER_CC4E;
	*/
	
	if (abs(power1) > 30)
		power1 = sgn(power1) * 30;
	setPWM(DRIBLER_1_TIM, DRIBLER_1_CH, 90 + power1 * 0.9);
}

void Robot::updateLine()
{
	setPin(LINE_SENSORS_SS, 0);
	uint8_t res = writeSPI(LINE_SENSORS_SPI, lineBoardTx);
	
	if (res != 0)
		line = max2(uint8_t(res - 1), line);
	else
	{
		GLOBAL_ERROR |= LINE_BOARD_CONNECTION_ERROR;
		line = 0;
	}
	
	setPin(LINE_SENSORS_SS, 1);
}

void Robot::beginLineCalibration()
{
	lineBoardTx = CALIBRATION_START;
}

void Robot::endLineCalibration()
{
	lineBoardTx = CALIBRATION_END;
}

void Robot::move(double velocity, double dir, double heading, double acc, double smooth)
{
	double _dir = abs(dir);
	while (_dir > 90) _dir -= 90;
	_dir = min2(_dir, 90 - _dir);
	
	double mul = 1.0 / cos(_dir * DEG2RAD);
	
	velocity *= 326.5 * mul;
	
	double dAngle = angle - calibratedAngle;
	adduction(dAngle);
	
	static double oldErr = dAngle + heading, p, d, s, err, k;
	
	/* Delta time estimating */
	static uint32_t sTime = millis();	
	double dt = millis() - sTime;
	sTime = millis();
	
	
	err = dAngle + heading;	
	adduction(err);
	p = err * 2.0;//8
	s = 0;//sgn(err) * err * err * 0.3;
	d = (err - oldErr) * 10.0 / dt;//20
	if (abs(d) > 70)
		d = sgn(d) * 70;

	double u = (p + d + s) * -1;
	// 1 ob kolesa - 0.3441 ob robota
	if (abs(u) > 400) u = sgn(u) * 400;
	
	double cx = velocity * cos((dir) * DEG2RAD);
	double cy = velocity * sin((dir) * DEG2RAD);
	
	acc *= 5 * dt / 3.0;
	
	double ax = (cx - dx);
	double ay = (cy - dy);
	
	double velAngle = atan2(ay, ax);
	
	double dax = cos(velAngle);
	double day = sin(velAngle);
	
	dx += stp(dax * acc, abs(ax));
	dy += stp(day * acc, abs(ay));
	
	
	double vel = sqrt(dx * dx + dy * dy);
	double tAngle = atan2(dy, dx) * RAD2DEG;
	
	int v1, v2, v3, v4;
	v1 = vel * sin((tAngle + dAngle - 40.)  * DEG2RAD);
	v2 = vel * sin((tAngle + dAngle - 140.) * DEG2RAD);
	v3 = vel * sin((tAngle + dAngle + 140.) * DEG2RAD);
	v4 = vel * sin((tAngle + dAngle + 40.)  * DEG2RAD);
	
	k = 1;//abs(vel) / max5(abs(v1), abs(v2), abs(v3), abs(v4), 1);
 
	v1 = v1 * k + u;
  v2 = v2 * k + u;
  v3 = v3 * k + u;
  v4 = v4 * k + u;
	
	setMotors(v1, v2, v3, v4);
	
	oldErr = err;
}

void Robot::setupIMU()
{
	updateIMU();
	calibratedAngle = angle;
}

void Robot::updateIMU()
{
	angle = imu.yaw;	
	adduction(angle);
}


// 0xBB : speed / 80 : direction[0] : direction[1] : heading[0] : heading[1] : acc / 10 : playState : dribler1 : dribler2 : crc8
void Robot::updateVIM()
{
	static const int MSG_SIZE = 11;
	static int ptr = 0, upperBound = MSG_SIZE, msg[MSG_SIZE];
	static bool inSeq = false;
	static uint32_t lastUseTime = millis();
	
	if (UARTAvailable(VIM_UART))
	{
		lastUseTime = millis();
		
		int symb = ((int)(readUART(VIM_UART)) + 256) % 256;
		
		if (symb == 0xBB && !inSeq) // if start of message detected
		{
			inSeq = true;
			ptr = 0;
		}
		
		if (inSeq)
		{
			msg[ptr++] = symb;
			
			if (ptr == upperBound) // message completed
			{
				char crc = crc8(msg, MSG_SIZE - 1);
				if (crc == msg[MSG_SIZE - 1])
				{
					target.ballX = msg[1];
					target.ballY = msg[2]; 
					target.yGoalX = msg[3];
					target.yGoalY = msg[4];
                    target.bGoalX = msg[5];
                    target.bGoalY = msg[6];
				}
				else
				{
					GLOBAL_ERROR |= VIM_DATA_ERROR;
				}
				
				ptr = 0;
				inSeq = false;
			}
		}
	}
	
	if (millis() - lastUseTime > 100) 
	{
		GLOBAL_ERROR |= VIM_CONNECTION_ERROR;
	}
	
	if (ptr >= MSG_SIZE) 
	{
		inSeq = false;
		ptr = 0;
	}
}

void Robot::updateDrivers()
{
	static int ptr = 0, upperBound = 16, msg[16];
	static bool inSeq = false;
	
	for (int i = 0; i < 4; i++)
	{
		Driver motor = motors[i];
		while (UARTAvailable(motor.uart))
		{
			int symb = ((int)(readUART(motor.uart)) + 256) % 256;
			
			if (symb == 0xBB && !inSeq) // if start of message detected
			{
				inSeq = true;
				
				ptr = 0;
			}
			
			if (inSeq)
			{
				msg[ptr++] = symb;
				
				//printUART(DEBUG_UART, msg[ptr - 1]);
				//writeStrUART(DEBUG_UART, " ");
				
				if (ptr == 3) // detect size of the message
				{
					upperBound = symb;
				}
				
				if (ptr == upperBound) // message completed
				{
					char addr = msg[1];
					int type = msg[4];
					
					/* Checking crc sum */
					char crc_1 = crc8(msg, 3);
					char crc_2 = crc8(msg, upperBound - 1);
					
					if (msg[3] != crc_1 || msg[upperBound - 1] != crc_2)
					{
						GLOBAL_ERROR |= DRIVER_DATA_ERROR;
						ptr = 0;
						return;
					}	
					for (int i = 0; i < 4; i++) // Check every motor
					{
						if (addr == motors[i].addr) // For motor with correct address
						{
							if (type == 166) // If message is slave answer for reading
							{
								int len = (upperBound - 6);
								for (int j = 0; j < len; j += 2)
								{
									motors[i].reg[msg[j + 5]] = msg[j + 6];
								}
							}
							
							if (type == 227 || type == 230)
							{							
								GLOBAL_ERROR |= DRIVER_FATAL_ERROR;
							}
							
							motors[i].updateFromReg();
						}
					}
					
					inSeq = false;
					ptr = 0;
				}
				
				
				if (ptr >= 16) inSeq = false;
			}
		}
	}
}

void Robot::cycleEnd()
{
	line = 0;
}

void Robot::updateMenu()
{
	static int32_t lastUpdate = millis(); 
	static bool wasButtonUnpressed = true;
	
	if ((int32_t)millis() - lastUpdate < 250)
		return;
	
	if (!button[0].pressed)
		wasButtonUnpressed = true;
	
	if (button[0].pressed && wasButtonUnpressed)
	{
		display.execute();
		lastUpdate = millis();
		wasButtonUnpressed = false;
	}
	
	if (button[1].pressed)
	{
		display.scrollDown();
		lastUpdate = millis();
	}
	
	if (button[2].pressed)
	{
		display.scrollUp();
		lastUpdate = millis();
	}
}

void Robot::wait(uint32_t t)
{
	static uint32_t error_step_time = 0;
	if ((millis() + error_step_time) % 250 < 50) 
	{
		GLOBAL_ERROR = 0;
		error_step_time += 50;
	}
	
	int64_t tt = millis();
	do {
		updateBatteryVoltage();
		updateDrivers();
		updateIMU();
		updateVIM();
		updateButtons();
		updateLine();
		updateBallSensors();
		updateCells();
		updateMenu();
		logErrors();
		updateKickers();
		updateDribblers(0);
	} while(millis() - tt < t);
	
	if (GLOBAL_ERROR) setPin(LED_ERROR, 1);
	else setPin(LED_ERROR, millis() % 200 < 100);
}

void Robot::sendCommandToVIM(char reg, bool val)
{
	char begin = reg * 2 + (val ? 1 : 0);
	writeUART(VIM_UART, begin * 16 + begin);
}

void Robot::sync()
{
}

void Robot::calibrateIMU(uint32_t t)
{
	imu.calibrate(t);
}

void Robot::logErrors()
{
	for (int i = 0; i < 4; i++) motors[i].logError();
	/*
	if (GLOBAL_ERROR & IMU_DATA_ERROR)
		writeStrUART(DEBUG_UART, "IMU data error\r\n");
	if (GLOBAL_ERROR & DRIVER_DATA_ERROR)
		writeStrUART(DEBUG_UART, "Driver data error\r\n");
	if (GLOBAL_ERROR & DRIVER_CONNECTION_ERROR)
		writeStrUART(DEBUG_UART, "Driver connection error\r\n");
	if (GLOBAL_ERROR & DRIVER_FATAL_ERROR)
		writeStrUART(DEBUG_UART, "Driver kokoko error\r\n");
	if (GLOBAL_ERROR & IMU_CONNECTION_ERROR)
		writeStrUART(DEBUG_UART, "IMU connection error\r\n");
	if (GLOBAL_ERROR & LINE_BOARD_CONNECTION_ERROR)
		writeStrUART(DEBUG_UART, "Line board connection error\r\n");
	*/
	if (GLOBAL_ERROR != 0)
	{
		//display.clear();
		print(GLOBAL_ERROR, 0, 14);
		//display();
	}
}

void Robot::print(char* x, int row, int column)
{		
	drawString(display.display, x, row, column);
}

void Robot::print(int x, int row, int column, int next)
{
	printTml(display.display, x, next, row, column);
}

void Robot::print(unsigned x, int row, int column, int next)
{
	printTml(display.display, x, next, row, column);
}

void Robot::print(long long  x, int row, int column, int next)
{
	printTml(display.display, x, next, row, column);
}

void Robot::print(unsigned long long  x, int row, int column, int next)
{
	printTml(display.display, x, next, row, column);
}

void Robot::print(double x, int row, int column, int next)
{
	printTml(display.display, x, next, row, column);
}

void Robot::print(float x, int row, int column, int next)
{
	printTml(display.display, x, next, row, column);
}
#endif
