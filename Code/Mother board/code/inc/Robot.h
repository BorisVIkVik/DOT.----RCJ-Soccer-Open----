#ifndef FUNCTIONALITY_LIB
#define FUNCTIONALITY_LIB

#include "connectionList.h"
#include "InitImage.h"
#include "KickerModule.h"
#include "LineSensors.h"
#include "Camera.h"
#include "Battery.h"
#include "BallSensor.h"
#include "MotorDrivers.h"
#include "Display.h"
#include "IMU.h"
#include "Buttons.h"
#include <utility>

#include "Errors.h"

using namespace std;

/* ROBOT */
class Robot
{
	public:
		LineSensors lineSensors;
		Camera camera;
		KickerModule kickerModule;
		Battery battery;
		BallSensor ballSensor;
		MotorDrivers motorDrivers;
		Display display;
		IMU imu;
		Buttons buttons;
	
		double dx, dy;
		PL_ADC ADC_1, ADC_2;
		long long int batteryUpdateTimer;
		long long int cameraTimer;
		long long int driverTimer;
	
		void init();
		void initLEDs();
		void swithPowerOn();
		void swithPowerOff();
		void switch5vOn();
		void switch5vOff();
		void move(double velocity, double dir, double heading = 0, double acc = 4 /* m/(s^2) */, double smooth = 1 /* ob/s */);
		void updateMenu();
		void changePlayState();
		void setPlayState(bool a);
		bool playState();
		void updateSelfPos(pair<double, double> yellow, pair<double, double> blue);
		pair<double, double> getPos();
		pair<double, double> getV();
		void wait(uint32_t t);
		void handleErrors();
		void logErrors();
		
		
	private:
		bool playStateBool;
		long long int playStateTimer;
		pair<double, double> pos;
		pair<double, double> v;
};


void Robot::init()
{	
	//Turn on power switch
	initPin(POWER_SWITCH, OUTPUTPP);
	swithPowerOn();
	
	//Turn on 5V regulator
	initPin(REG_5V_EN, OUTPUTPP);
	switch5vOff();
	
	//Initialize OLED screen on mother board and show init image
	SSD1306 disp(DISPLAY_SPI, DISPLAY_DC, DISPLAY_RESET, DISPLAY_CS, DISPLAY_PWR_EN, DISPLAY_SCK, DISPLAY_MOSI);
	disp.switchOn();
	disp.begin();
	disp.clear();
	disp.display(initImageBuffer);
	disp.invert(0);
	disp.rotate(2);
	int initImageTimer = millis();
	
	//Initialize LEDs on mother board
	initLEDs();
	
	//Initialize buttons on mother board
	buttons.init(BUTTON_1_ENTER, BUTTON_2_DOWN, BUTTON_3_UP, BUTTON_4_ESC);
	
	//Initialize IMU
	imu.init(IMU_SPI, IMU_SPI_SS, IMU_PWR_EN);
	
	//Initialize all motor drivers
	motorDrivers.init(MOTOR_1_UART, MOTOR_1,
																		MOTOR_2_UART, MOTOR_2,
																		MOTOR_3_UART, MOTOR_3,
																		MOTOR_4_UART, MOTOR_4,
																		MOTOR_DRIBBLER_UART, MOTOR_DRIBBLER
																		);
	
	//Initialize UART for bluetooth and LIDAR
	initUART(BLUETOOTH_UART, 115200, 8, 0, NO_PARITY, 42);
	initUART(LIDAR_UART, 115200, 8, 1, NO_PARITY, 42);
	//Delay for successfull setup of UARTs
	delay(200);
	
	//Initialize camera and line sensors board
	initSPI(PERIPH_SPI, MASTER, 8, 8);
	lineSensors.init(PERIPH_SPI, LINE_SENSORS_SS);
	camera.init(PERIPH_SPI, CAMERA_SS);
	
	//Initialize kicker
	kickerModule.init(KICKER_BOOSTER_EN, KICKER_BOOSTER_DONE, KICKER_1, KICKER_2);
	
	//Initialize ADCs for battery voltage reading and ball sensor
	ADC_1 = *(new PL_ADC(ADC1));
	ADC_1.init();
	battery.init(&ADC_1, BATTERY_S1, BATTERY_S2, BATTERY_S3, BATTERY_S4);
	ADC_1.start();
	
	ADC_2 = *(new PL_ADC(ADC2));
	ADC_2.init();
	ballSensor.init(&ADC_2, BALL_SENSOR, BALL_SENSOR_EN);
	ADC_2.start();
	
	//Some variables
	dx = 0;
	dy = 0;

	pos.X = 0;
	pos.Y = 0;
	
	batteryUpdateTimer = millis();
	cameraTimer = millis();
	driverTimer = millis() - 125;
	
	playStateBool = false;
	playStateTimer = millis();
	
	//Show init image at least 1 second
	while(millis() - initImageTimer < 1000);
	//Clear display
	disp.clear();
	disp.display();
	//Initialize display
	display.init(&disp);
}


void Robot::initLEDs()
{
	initPin(LED_1, OUTPUTPP);
	initPin(LED_2, OUTPUTPP);
	initPin(LED_3, OUTPUTPP);
	
	setPin(LED_1, 0);
	setPin(LED_2, 0);
	setPin(LED_3, 0);
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


void Robot::move(double velocity, double dir, double heading, double acc, double smooth)
{
	double _dir = abs(dir);
	while (_dir > 90) _dir -= 90;
	_dir = min2(_dir, 90 - _dir);
	
	double mul = 1.0 / cos(_dir * DEG2RAD);
	
	velocity *= 326.5 * mul;
	
	double dAngle = imu.getAngle();
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
	v1 = vel * sin((tAngle + dAngle - 32.)  * DEG2RAD);
	v2 = vel * sin((tAngle + dAngle - 135.) * DEG2RAD);
	v3 = vel * sin((tAngle + dAngle + 135.) * DEG2RAD);
	v4 = vel * sin((tAngle + dAngle + 32.)  * DEG2RAD);
	
	k = 1;//abs(vel) / max5(abs(v1), abs(v2), abs(v3), abs(v4), 1);
 
	v1 = v1 * k + u;
  v2 = v2 * k + u;
  v3 = v3 * k + u;
  v4 = v4 * k + u;
	
	motorDrivers.setMotors(v1, v2, v3, v4);
	
	v.X = velocity*sin(dir*DEG2RAD);
	v.Y = velocity*cos(dir*DEG2RAD);

	oldErr = err;
}


void Robot::swithPowerOn()
{
	setPin(POWER_SWITCH, 0);
}


void Robot::swithPowerOff()
{
	setPin(POWER_SWITCH, 1);
}


void Robot::switch5vOn()
{
	setPin(REG_5V_EN, 1);
}


void Robot::switch5vOff()
{
	setPin(REG_5V_EN, 0);
}


void Robot::updateSelfPos(pair<double, double> yellow, pair<double, double> blue)
{	
	double K = 0.5 + 0.5;//- (sqrt(double(yellow.X*yellow.X + yellow.Y*yellow.Y)) - sqrt(double(blue.X*blue.X + blue.Y*blue.Y)))/134*0.5;
	if(K > 1.0) K = 1.0;
	if(K < 0.0) K = 0.0;
	display.print("K: ", 1, 1);
	display.print(K, 1, 5);
	#define GOAL_X	0
	#define GOAL_Y	103
	
	pos.X = (GOAL_X - yellow.X)*K + (GOAL_X - blue.X)*(1.0 - K);
	pos.Y = (-GOAL_Y - yellow.Y)*K + (GOAL_Y - blue.Y)*(1.0 - K);
}


pair<double, double> Robot::getPos()
{
	return pos;
}

pair<double, double> Robot::getV()
{
	return v;
}

void Robot::updateMenu()
{
	static int32_t lastUpdate = millis(); 
	static bool wasButtonUnpressed = true;
	
	if ((int32_t)millis() - lastUpdate < 250)
		return;
	
	if (!buttons.isPressed(ENTER_BUTTON))
		wasButtonUnpressed = true;
	
	if (buttons.isPressed(ENTER_BUTTON) && wasButtonUnpressed)
	{
		display.execute();
		lastUpdate = millis();
		wasButtonUnpressed = false;
	}
	
	if (buttons.isPressed(DOWN_BUTTON))
	{
		display.scrollDown();
		lastUpdate = millis();
	}
	
	if (buttons.isPressed(UP_BUTTON))
	{
		display.scrollUp();
		lastUpdate = millis();
	}
	
	if (buttons.isPressed(ESC_BUTTON))
	{
		display.escape();
		lastUpdate = millis();
	}
}


void Robot::wait(uint32_t t)
{
	/*
	static uint32_t error_step_time = 0;
	if ((millis() + error_step_time) % 250 < 50) 
	{
		GLOBAL_ERROR = 0;
		error_step_time += 50;
	}
	*/
	
	int64_t tt = millis();
	do {
		
		if(millis() - batteryUpdateTimer > 250)
		{
			GLOBAL_ERROR |= battery.update();
			batteryUpdateTimer = millis();
		}
		if(millis() - cameraTimer > 30)
		{
			GLOBAL_ERROR |= camera.update();
		}
		if(millis() - driverTimer > 250)
		{
			motorDrivers.attemptCurrent();
			driverTimer = millis();
		}
		GLOBAL_ERROR |= motorDrivers.update();
		GLOBAL_ERROR |= imu.update();
		GLOBAL_ERROR |= lineSensors.update();
		GLOBAL_ERROR |= ballSensor.update();
		GLOBAL_ERROR |= buttons.update();		
		//GLOBAL_ERROR |= lidar.update();
		
		updateMenu();
		
		handleErrors();
	} while(millis() - tt < t);
	
}


void Robot::handleErrors()
{
	logErrors();
	
	if (GLOBAL_ERROR) setPin(ERROR_LED, 1);
	else setPin(ERROR_LED, millis() % 200 < 100);
	
	if(GLOBAL_ERROR & LOW_BATTERY_POWER)
	{
		motorDrivers.setMotors(0, 0, 0, 0, 0);
		display.clear();
		display.print("LOW BATTERY VOLTAGE", 1, 0);
		display.print("TURNING OFF", 2, 0);
		display.show();
		swithPowerOff();
		while(1);
	}
}


void Robot::logErrors()
{
	//for (int i = 0; i < 4; i++) motors[i].logError();
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
		display.print(GLOBAL_ERROR, 4, 14);
		//display();
	}
}

#endif
