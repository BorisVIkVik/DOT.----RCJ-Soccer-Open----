#include "stm32f103_UART.h"


char _bufRx[UARTQ][UARTbufSize];
char _bufTx[UARTQ][UARTbufSize];
uint8_t _bufRxHead[UARTQ], _bufRxTail[UARTQ];
uint8_t _bufTxHead[UARTQ], _bufTxTail[UARTQ];


void initUART(unsigned int num, uint32_t baudrate, uint8_t wordLength, float _stopBits, uint8_t parity, unsigned int clk){
	
	//clear buffers
	_bufRxHead[num] = _bufRxTail[num] = _bufTxHead[num] = _bufTxTail[num] = 0;	
	for(int i = 0; i < UARTbufSize; i++){
		_bufRx[num][i] = 0;
		_bufTx[num][i] = 0;
	}
	
	//calculate baudrate
	baudrate = (clk * 1000000 + baudrate / 2) / baudrate;				
	
	//select uart register name
	USART_TypeDef * uart;
	switch(num){																								
		default:
			uart = USART1;
			//set clk
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;										
			
			//init pins
			initPin(A, 9, OUTPUTAFPP);																								
			initPin(A, 10, INPUTFL);
			
			//check baudrate
			if(baudrate != 7500 || baudrate != 3750 || baudrate != 1875 || baudrate != 625) baudrate = 625;		
		
			//enable interrupts
			NVIC_EnableIRQ(USART1_IRQn);						
			break;
		
		case _UART2:
			uart = USART2;
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
			
			initPin(A, 2, OUTPUTAFPP);
			initPin(A, 3, INPUTFL);
		
			if(baudrate != 3750 || baudrate != 1875 || baudrate != 938 || baudrate != 313) baudrate = 313;
		
			NVIC_EnableIRQ(USART2_IRQn);
			break;
		
		case _UART3:
			uart = USART3;
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			
			initPin(B, 10, OUTPUTAFPP);
			initPin(B, 11, INPUTFL);
		
			if(baudrate != 3750 || baudrate != 1875 || baudrate != 938 || baudrate != 313) baudrate = 313;
		
			NVIC_EnableIRQ(USART3_IRQn);
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
		case 10: uart->CR2 &= ~USART_CR2_STOP; break;																		//1 stop bit
		case 5: uart->CR2 &= ~USART_CR2_STOP_1; uart->CR2 |= USART_CR2_STOP_0; break;		//0.5 stop bit
		case 20: uart->CR2 |= USART_CR2_STOP_1; uart->CR2 &= ~USART_CR2_STOP_0; break;	//2 stop bits
		case 15: uart->CR2 |= USART_CR2_STOP; break;																		//1.5 stop bits
	}
	
	//set parity
	if(parity != NO_PARITY){																			
		uart->CR1 |= USART_CR1_PCE;
		if(parity == EVEN_PARITY) uart->CR1 &= ~USART_CR1_PS;
		else if(parity == ODD_PARITY)uart->CR1 |= USART_CR1_PS;
	}
	else 
		uart->CR1 &= ~USART_CR1_PCE;
	
	//set baudrate
	uart->BRR = baudrate;							
		
	//enable tx & rx
	uart->CR1 |= USART_CR1_TE;
	uart->CR1 |= USART_CR1_RE;
		
	//enable rx & tx interrupt
	uart->CR1 |= USART_CR1_RXNEIE;			
	uart->CR1 |= USART_CR1_TXEIE;
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


uint8_t UARTAvailable(unsigned int num){
	//return number of available bytes to read
	return (uint8_t)(UARTbufSize + _bufRxHead[num] - _bufRxTail[num]) % UARTbufSize;
}

char readUART(unsigned int num){	
	
	if(_bufRxTail[num] == _bufRxHead[num])				//if buffer is empty return 0
		return 0;
	
	else{																					//if buffer is not empty return first byte
		char data = _bufRx[num][_bufRxTail[num]];
		_bufRxTail[num] = (_bufRxTail[num] + 1) % UARTbufSize;
		return data;
	}
	
}


void UARTInterruptHandler(unsigned int num){			//actual interrupt handler
	
	//select uart register name
	USART_TypeDef * uart;								
	switch(num){
			default: uart = USART1; break;
			case _UART2: uart = USART2; break;
			case _UART3: uart = USART3; break;
		}
	
	if(uart->SR & USART_SR_RXNE){										//rx interrupt
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


void tryTransmit(unsigned int num){
	
	//select uart register name
	USART_TypeDef * uart;							
	switch(num){
		default: uart = USART1; break;
		case _UART2: uart = USART2; break;
		case _UART3: uart = USART3; break;
	}
	
	//wait till last byte is in transmit data register (if this function was not called because of interrupt)
	while(!(uart->SR & USART_SR_TXE));					
	
	//transmit another byte
	uart->DR = _bufTx[num][_bufTxTail[num]];								
	_bufTxTail[num] = (_bufTxTail[num] + 1) % UARTbufSize;
	
	//enable interrupt if buffer till is not empty
	if(_bufTxHead[num] == _bufTxTail[num])				
		uart->CR1 &= ~USART_CR1_TXEIE;
	else 
		uart->CR1 |= USART_CR1_TXEIE;
	
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
		
	if(_data < 0.0){
		writeUART(num, '-');																//mozhesh poprobovat ponyat kak eto rabotaet
		_data = -_data;
	}
	
	printUART(num, (unsigned long)(_data));
	writeUART(num, '.');
	
	_data -= (unsigned long)(_data);
	unsigned long d = _data * 1000000000;
	
	while(d % 10 == 0 && d != 0)
		d /= 10;
	
	printUART(num, d);
	
}

