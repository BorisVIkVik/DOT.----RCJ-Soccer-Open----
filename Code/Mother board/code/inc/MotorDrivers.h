#ifndef MOTOR_DRIVERS
#define MOTOR_DRIVERS

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_UART.h"
#include "drivers.h"
#include "tools.h"

#include "Errors.h"


class MotorDrivers{
	public:
		void init(unsigned int uart1, uint8_t add1, 
									unsigned int uart2, uint8_t add2,
									unsigned int uart3, uint8_t add3,
									unsigned int uart4, uint8_t add4,
									unsigned int uartDB, uint8_t addDB);
		void setMotor(int n, int vel);
		void setMotors(int v1, int v2, int v3, int v4);
		void setMotors(int v1, int v2, int v3, int v4, int v5);
		void setDribbler(int v);
		void enableMotors();
		void disableMotors();
		unsigned int update();
		void attemptCurrent();
	
	private:
		unsigned int uart[5];
		uint8_t add[5];
		Driver drivers[5];
		int oldMotorSpeed[5];
};


void MotorDrivers::init(unsigned int uart1, uint8_t add1, 
														unsigned int uart2, uint8_t add2,
														unsigned int uart3, uint8_t add3,
														unsigned int uart4, uint8_t add4,
														unsigned int uartDB, uint8_t addDB)
{
	uart[0] = uart1;
	uart[1] = uart2;
	uart[2] = uart3;
	uart[3] = uart4;
	uart[4] = uartDB;
	
	add[0] = add1;
	add[1] = add2;
	add[2] = add3;
	add[3] = add4;
	add[4] = addDB;
	
	for(int i = 0; i < 5; i++)
		initUART(uart[i], 115200, 8, 1, NO_PARITY);
	//Delay for successfull setup of UARTs
	delay(200);
	
	for(int i = 0; i < 5; i++)
		drivers[i].init(add[i], uart[i]);

	for (int i = 0; i < 5; i++)
	{
		oldMotorSpeed[i] = 1;
		setMotor(i, 0);
	}
}


void MotorDrivers::setMotor(int n, int vel)
{
	if(vel != oldMotorSpeed[n])
	{
		oldMotorSpeed[n] = vel;
		drivers[n].setVelocity(vel);
	}	
}


void MotorDrivers::setDribbler(int vel)
{
	setMotor(4, vel);
}


void MotorDrivers::setMotors(int v1, int v2, int v3, int v4)
{
	setMotor(0, v1);
	setMotor(1, v2);
	setMotor(2, v3);
	setMotor(3, v4);
	//while(!UARTTxFinished(motors[0].uart) || !UARTTxFinished(motors[1].uart) || !UARTTxFinished(motors[2].uart) || !UARTTxFinished(motors[3].uart));
	//wait(2);
}


void MotorDrivers::setMotors(int v1, int v2, int v3, int v4, int v5)
{
	setMotors(v1, v2, v3, v4);
	setMotor(4, v5);
	//while(!UARTTxFinished(motors[4].uart));
	//wait(2);
}


void MotorDrivers::disableMotors()
{
	for (int i = 0; i < 4; i++)
	{
		drivers[i].enableFloating();
	}
}

void MotorDrivers::enableMotors()
{
	for (int i = 0; i < 4; i++)
	{
		drivers[i].disableFloating();
	}
}


unsigned int MotorDrivers::update()
{
	unsigned int error = 0;
	static int ptr = 0, upperBound = 16, msg[16];
	static bool inSeq = false;
	
	for (int i = 0; i < 5; i++)
	{
		Driver motor = drivers[i];
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
						ptr = 0;
						error |= DRIVER_DATA_ERROR;
						return error;
					}	
					for (int i = 0; i < 4; i++) // Check every motor
					{
						if (addr == drivers[i].addr) // For motor with correct address
						{
							if (type == 166) // If message is slave answer for reading
							{
								int len = (upperBound - 6);
								for (int j = 0; j < len; j += 2)
								{
									drivers[i].reg[msg[j + 5]] = msg[j + 6];
								}
							}
							
							if (type == 227 || type == 230)
							{							
								error |= DRIVER_FATAL_ERROR;
							}
							
							drivers[i].updateFromReg();
						}
					}
					
					inSeq = false;
					ptr = 0;
				}
				
				
				if (ptr >= 16) inSeq = false;
			}
		}
	}
	return error;
}


void MotorDrivers::attemptCurrent()
{
	for(int i = 0; i < 5; i++)
		drivers[i].attemptCurrent();
}

#endif
