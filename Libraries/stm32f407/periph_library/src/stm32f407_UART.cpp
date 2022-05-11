#include "stm32f407_UART.h"


char _bufRx[UARTQ][UARTbufSize];
char _bufTx[UARTQ][UARTbufSize];
uint8_t _bufRxHead[UARTQ], _bufRxTail[UARTQ];
uint8_t _bufTxHead[UARTQ], _bufTxTail[UARTQ];

//char _lidarBufRx[UARTbufSize];
//char _lidarBufTx[UARTbufSize];
//uint8_t _lidarRxHead, _lidarRxTail;
//uint8_t _lidarTxHead, _lidarTxTail;

T_RX_BUFF RxBuffer;


void initUART(unsigned int num, uint32_t baudrate, uint8_t wordLength, float _stopBits, uint8_t parity, unsigned int clk){
	
	//clear buffers
	_bufRxHead[num] = _bufRxTail[num] = _bufTxHead[num] = _bufTxTail[num] = 0;	
	for(int i = 0; i < UARTbufSize; i++){
		_bufRx[num][i] = 0;
		_bufTx[num][i] = 0;
	}
	
	if(num > 6) {
		#ifdef STM32F407_SOFTWARE_UART
		
		initSoftUART(baudrate, clk, wordLength);
		
		#endif
	}
	else {
		
		//select uart register name
		USART_TypeDef * uart;
		switch(num){																								
			default:
				uart = USART1;
				//set clk
				RCC->APB2ENR |= RCC_APB2ENR_USART1EN;										
				
				//init pins
				initPin(A, 8, AFPP, PU, 7);
				initPin(A, 9, AFPP, PU, 7);
				initPin(A, 10, AFPP, PU, 7);
			
				uart->CR2 |= USART_CR2_CLKEN;
			
				//enable interrupts
				NVIC_EnableIRQ(USART1_IRQn);						
				break;
			
			case _UART1:
				uart = USART1;
				//set clk
				RCC->APB2ENR |= RCC_APB2ENR_USART1EN;										
				
				//init pins
				initPin(A, 9, AFPP, PU, 7);
				initPin(A, 10, AFPP, PU, 7);
			
				//enable interrupts
				NVIC_EnableIRQ(USART1_IRQn);						
				break;
			
			case _UART2:
				uart = USART2;
				RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
				
				initPin(A, 2, AFPP, PU, 7);
				initPin(A, 3, AFPP, PU, 7);
			
				NVIC_EnableIRQ(USART2_IRQn);
				break;
			
			case _UART3:
				uart = USART3;
				RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
				
				initPin(B, 10, AFPP, PU, 7);
				initPin(B, 11, AFPP, PU, 7);
			
				NVIC_EnableIRQ(USART3_IRQn);
				break;
			
			case _UART4:
				uart = UART4;
				RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
				
				initPin(A, 0, AFPP, PU, 8);
				initPin(A, 1, AFPP, PU, 8);

				NVIC_EnableIRQ(UART4_IRQn);
				break;
			
			case _UART5:
				uart = UART5;
				RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
				
				initPin(C, 12, AFPP, PU, 8);
				initPin(D, 2, AFPP, PU, 8);

				NVIC_EnableIRQ(UART5_IRQn);
				break;
			
			case _UART6:
				uart = USART6;
				RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
				
				initPin(C, 6, AFPP, PU, 8);
				initPin(C, 7, AFPP, PU, 8);
			
				NVIC_EnableIRQ(USART6_IRQn);
				break;
		}
						
		
		//enable uart
		uart->CR1 |= USART_CR1_UE;
		
		//set word length (8 or 9 bits)
		if(wordLength == 9)uart->CR1 |= USART_CR1_M;		
		else uart->CR1 &= ~USART_CR1_M;
		
		//set stop bits
		int stopBits = _stopBits * 10;
		switch(stopBits){																
			case 10: uart->CR2 &= ~USART_CR2_STOP; break;										//1 stop bit
			case 5: uart->CR2 &= ~USART_CR2_STOP_1; uart->CR2 |= USART_CR2_STOP_0; break;		//0.5 stop bit
			case 20: uart->CR2 |= USART_CR2_STOP_1; uart->CR2 &= ~USART_CR2_STOP_0; break;		//2 stop bits
			case 15: uart->CR2 |= USART_CR2_STOP; break;										//1.5 stop bits
		}
		
		//set parity
		if(parity != NO_PARITY){																			
			uart->CR1 |= USART_CR1_PCE;
			if(parity == EVEN_PARITY) uart->CR1 &= ~USART_CR1_PS;
			else if(parity == ODD_PARITY)uart->CR1 |= USART_CR1_PS;
		}
		else 
			uart->CR1 &= ~USART_CR1_PCE;
		
		//set oversampling
		uart->CR1 &= ~USART_CR1_OVER8;
		
		
		if(clk == 0)
		{
			if(num == _UART1 || num == _UART6) clk = 84;
			else clk = 42;
		}
			
		//select baudrate
		uint16_t mantissa = 0, fraction = 0;
		/*if (clk == 42)												//for oversampling 8 (OVER8 = 1)
			switch (baudrate) {
				case 9600: mantissa = 546, fraction = 7; break;
				case 38400: mantissa = 136, fraction = 6; break;
				case 57600: mantissa = 91, fraction = 1; break;
				case 115200: mantissa = 45, fraction = 5; break;
			}
		else if(clk == 84)
			switch (baudrate) {
				case 9600: mantissa = 1093, fraction = 6; break;
				case 38400: mantissa = 273, fraction = 4; break;
				case 57600: mantissa = 182, fraction = 2; break;
				case 115200: mantissa = 91, fraction = 1; break;
			}*/
		if(clk == 42)
			switch(baudrate) {
				case 9600: mantissa = 273, fraction = 7; break;
				case 38400: mantissa = 68, fraction = 6; break;
				case 57600: mantissa = 45, fraction = 9; break;
				case 115200: mantissa = 22, fraction = 13; break;
				case 230400: mantissa = 11, fraction = 6; break;
		}
		else if(clk == 84)
			switch(baudrate) {
				case 9600: mantissa = 546, fraction = 14; break;
				case 38400: mantissa = 136, fraction = 12; break;
				case 57600: mantissa = 91, fraction = 2; break;
				case 115200: mantissa = 45, fraction = 9; break;
		}
		
		//set baudrate
		uart->BRR = 0;
		uart->BRR = (mantissa << 4) + fraction;
			
			
		//enable tx & rx
		uart->CR1 |= USART_CR1_RE;
		uart->CR1 |= USART_CR1_TE;
			
		//enable rx & tx interrupt
		uart->CR1 |= USART_CR1_RXNEIE;	
		uart->CR1 |= USART_CR1_TXEIE;		//may be disabled at start
		
	}
}

void writeUART(unsigned int num, char data){
	
	//add data to buffer
	_bufTx[num][_bufTxHead[num]] = data;											
	_bufTxHead[num] = (_bufTxHead[num] + 1) % UARTbufSize;
	
	//try to transmit data from buffer
	tryTransmit(num);				
	
}


void writeStrUART(unsigned int num, char* data){
	
	//transmit every symbol one by one
	uint8_t i = 0;							
	while(data[i]) writeUART(num, data[i++]);
	
	
}

void writeTrulyStrUART(unsigned int num, char* data, unsigned int len)
{
	for (unsigned i = 0; i < len; i++) writeUART(num, data[i]);
}

uint8_t UARTAvailable(unsigned int num){
	//return number of available bytes to read
	return (uint8_t)(UARTbufSize + _bufRxHead[num] - _bufRxTail[num]) % UARTbufSize;
}

bool UARTTxFinished(unsigned int num){
	
	USART_TypeDef * uart;								
	switch(num){
			case _UART2: uart = USART2; break;
			case _UART3: uart = USART3; break;
			case _UART4: uart = UART4; break;
			case _UART5: uart = UART5; break;
			case _UART6: uart = USART6; break;
			case _UART1: uart = USART1; break;
			default: uart = USART1; break;
		}
	
	if((uart->SR & USART_SR_TC)) return 1;
	else return 0;
}

char readUART(unsigned int num){	
	
	if(_bufRxTail[num] == _bufRxHead[num])				//if buffer is empty return 0
		return 0;
	
	else
	{												//if buffer is not empty return first byte
		char data = _bufRx[num][_bufRxTail[num]];
		_bufRxTail[num] = (_bufRxTail[num] + 1) % UARTbufSize;
		return data;
	}
	
}


void UARTInterruptHandler(unsigned int num){			//actual interrupt handler
	
	//select uart register name
	USART_TypeDef * uart;								
	switch(num){
			case _UART2: uart = USART2; break;
			case _UART3: uart = USART3; break;
			case _UART4: uart = UART4; break;
			case _UART5: uart = UART5; break;
			case _UART6: uart = USART6; break;
			case _UART1: uart = USART1; break;
			default: uart = USART1; break;
		}
	if (num != _UART2)
	{
		if((uart->SR & USART_SR_RXNE)){										//rx interrupt
			//check parity
			if(!(uart->SR & USART_SR_PE) && !(uart->SR & USART_SR_FE)){	
				//add recived byte to the buffer
				_bufRx[num][_bufRxHead[num]] = uart->DR;		
				_bufRxHead[num] = (_bufRxHead[num] + 1) % UARTbufSize;
			}
			
			//disable flag
			uart->SR &= ~USART_SR_FE;	
			uart->SR &= ~USART_SR_PE;	
			uart->SR &= ~USART_SR_RXNE;								
		}		
		else if((uart->SR & USART_SR_TXE))							//tx buffer empty interrupt
			//transmit another byte
			tryTransmit(num);
	}
	else
	{
		if((uart->SR & USART_SR_RXNE)){										//rx interrupt
			//check parity
			if(!(uart->SR & USART_SR_PE) && !(uart->SR & USART_SR_FE)){	
				//add recived byte to the buffer
					if(RxBuffer.Rdy == 0)
					{
						RxBuffer.Buff[RxBuffer.Len++] = uart->DR;
						if(RxBuffer.Len >= sizeof(RxBuffer.Buff))
							RxBuffer.Rdy = 1;
					}
				//_bufRx[num][_bufRxHead[num]] = uart->DR;		
				//_bufRxHead[num] = (_bufRxHead[num] + 1) % UARTbufSize;
			}
			
			//disable flag
			uart->SR &= ~USART_SR_FE;	
			uart->SR &= ~USART_SR_PE;	
			uart->SR &= ~USART_SR_RXNE;								
		}
		else if((uart->SR & USART_SR_TXE))							//tx buffer empty interrupt
			//transmit another byte
			tryTransmit(num);
	}
		

	
}


void tryTransmit(unsigned int num){
	
	//select uart register name
	USART_TypeDef * uart;							
	switch(num){
		case _UART2: uart = USART2; break;
		case _UART3: uart = USART3; break;
		case _UART4: uart = UART4; break;
		case _UART5: uart = UART5; break;
		case _UART6: uart = USART6; break;
		case _UART1: uart = USART1; break;
		default: uart = USART1; break;
	}
	
	//if buffer is empty
	if (_bufTxHead[num] == _bufTxTail[num]){
		uart->CR1 &= ~USART_CR1_TXEIE;
		return;
	}
	
	//wait till last byte is in transmit data register (if this function was not called because of interrupt)
	while(!(uart->SR & USART_SR_TXE)){};					
	
	//transmit another byte
	uart->DR = _bufTx[num][_bufTxTail[num]];				
	_bufTxTail[num] = (_bufTxTail[num] + 1) % UARTbufSize;
	
	//enable interrupt if buffer till is not empty
	if(_bufTxHead[num] == _bufTxTail[num]){				
		uart->CR1 &= ~USART_CR1_TXEIE;
	}
	else{ 
		uart->CR1 |= USART_CR1_TXEIE;
	}
	
}




/************************************ interrupt handlers ************************************/


#ifdef __cplusplus
extern "C"{
#endif
	
void USART1_IRQHandler(void){
	UARTInterruptHandler(_UART1);
}

void USART2_IRQHandler(void){
	UARTInterruptHandler(_UART2);
}

void USART3_IRQHandler(void){
	UARTInterruptHandler(_UART3);
}

void UART4_IRQHandler(void) {
	UARTInterruptHandler(_UART4);
}

void UART5_IRQHandler(void) {
	UARTInterruptHandler(_UART5);
}

void USART6_IRQHandler(void) {
	UARTInterruptHandler(_UART6);
}

#ifdef __cplusplus
}
#endif


/************************************ functions for send variable like ASCII number ************************************/


void printUART(unsigned int num, char data){
	printUART(num, (unsigned long) data);
}

void printUART(unsigned int num, int data){
	printUART(num, (long) data);
}

void printUART(unsigned int num, unsigned int data){
	printUART(num, (unsigned long) data);
}

void printUART(unsigned int num, long data){
	if(data < 0){
		writeUART(num, '-');
		data = -data;
	}
	printUART(num, (unsigned long) data);
}

void printUART(unsigned int num, unsigned long data){
	
	unsigned long _data = (double)data;
	unsigned int n = 0;
	
	while(_data >= 10){
		n++;
		_data /= 10;
	}
	
	_data = data;
	for(int i = 0; i <= n; i++){
		unsigned int a = pow((double)10, (int)(n - i));
		uint8_t b = _data / a;
		writeUART(num, b + 48);
		_data -= b * a;
	}
}

void printUART(unsigned int num, float data){
	printUART(num, (double)data);
}

void printUART(unsigned int num, double data){

	double _data = data;

	if (_data < 0.0)
  {
		writeUART(num, '-');
		_data = -_data;
  }

	printUART(num, int(_data));
	writeUART(num, '.');
	
	_data -= int(_data);
	_data += 1e-7;
   
	double eps = 1e-6;
	for (int i = 0; i < 6; i++)
	{
		eps *= 10;
		_data *= 10;
		printUART(num, int(_data) % 10);
		_data -= int(_data);
		if (_data < eps)
			break;
	}
	/*
  for (int i = 0; i < 9; i++)
  {
      _data *= 10;
      printUART(num, int(_data) % 10);
      if (abs(_data - int(_data)) < 1e-6)
				break;
  }
	*/
}
