#ifndef STM32F407_SOFTWARE_UART
#define STM32F407_SOFTWARE_UART

#include <stm32f4xx.h>
#include <stm32f407_pin.h>

static void initUARTDMATx(void)
{
	setPin(A, 8, true);//Change
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
	DMA1_Stream0->CR |= (0x01<<25);
	DMA1_Stream0->CR |= (0x01<<6);
	DMA1_Stream0->CR |= (0x01<<14);
	DMA1_Stream0->CR |= (0x01<<12);
	DMA1_Stream0->CR |= (0x03<<16);
	DMA1_Stream0->CR |= (0x01<<10);

  //NVIC_SetPriority(DMA2_Stream1_IRQn, 3);	//Maybe
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

static void initUARTDMARx(void)
{
	setPin(A, 8, true);//Change
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
	DMA1_Stream0->CR |= (0x01<<25);
	DMA1_Stream0->CR |= (0x01<<14);
	DMA1_Stream0->CR |= (0x01<<12);
	DMA1_Stream0->CR |= (0x03<<16);
	DMA1_Stream0->CR |= (0x01<<10);

  //NVIC_SetPriority(DMA2_Stream1_IRQn, 3); //Maybe
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}



static void initEmulatedUART (uint32_t baudrate, unsigned int clk)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->ARR = ((uint32_t) ((clk*1000000/baudrate) - 1));	//Hz
	TIM2->PSC = 0;

	initUARTDMATx();
	initUARTDMARx();
}


///////////////////////////////////////////////////////////////////////////////ne hvat constant i ykazat nomer pina
/**
  * @brief  This function formats one Frame
  * @param  UART Emulation Handle
  * @param  pdata pinteur in data
  * @retval None
  */
static void emulatedUARTTransmitFormatFrame(uint8_t Data, uint32_t *pBuffer_Tx)
{
uint32_t counter = 0;
uint32_t bitmask = 0;
uint32_t length = 0;
uint32_t cntparity = 0;


  length = UART_WORDLENGTH_8B;

  /* Get the Pin Number */
  bitmask = (uint32_t)huart->Init.TxPinNumber;

/* with parity */
  for (counter = 0; counter < length-1; counter++)
  {
    if (((Data >> counter)&BitMask) != 0)
    {
      pBuffer_Tx[counter+1] = bitmask;
      cntparity ++;
    }
    else
    {
      pBuffer_Tx[counter+1] = (bitmask << 16);
    }
  }	
	
    /* Initialize Parity Bit */
  if ((cntparity % 2) != SET)
  {
    pBuffer_Tx[length] = bitmask;
  }
  else
  {
    pBuffer_Tx[length] = (bitmask << 16);
  }
  /* Initialize Bit Start */
  pBuffer_Tx[0] = (bitmask << 16);
    /* Initialize Bit Stop  */
  pBuffer_Tx[length+1] = bitmask;
  /* Reset counter parity */
  cntparity = 0;
}


int TxXferCount = 0;


/////////////////////////////////////////////////////////////////////////////////////////ne hvat bufer
static void emulatedUARTTransmitFrame()
{
  uint32_t tmp_sr = 0;
  uint32_t tmp_ds = 0;
  uint32_t tmp_size = 0;

	
  if ((TxXferCount % 2 ) != 0)
  {
    tmp_sr = (uint32_t)pFirstBuffer_Tx;
  }
  else
  {
    tmp_sr = (uint32_t)pSecondBuffer_Tx;
  }

	tmp_ds = (uint32_t) & (GPIOA->BSRR);
	
  tmp_size = 8 + 1 + 1;//__HAL_UART_EMUL_FRAME_LENGTH(huart);

  /* Configure DMA Stream data length */
  DMA1_Stream0->NDTR = tmp_size;

  /* Configure DMA Stream destination address */
  DMA1_Stream0->PAR = tmp_ds;

  /* Configure DMA Stream source address */
  DMA1_Stream0->M0AR = tmp_sr;

  /* Enable the transfer complete interrupt */
  DMA1_Stream0->CR |= DMA_SxCR_TCIE;//DMA_IT_TC;//__HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TC);

  /* Enable the transfer Error interrupt */
  DMA1_Stream0->CR |= DMA_SxCR_TEIE;//DMA_IT_TE;//__HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TE);

  /* Enable the Peripheral */
  DMA1_Stream0->CR |= DMA_SxCR_EN;//__HAL_DMA_ENABLE(&hdma_tx);

  /* Enable the TIM Update DMA request */
  TIM1->DIER |= TIM_DIER_CC1DE;//TIM_DMA_CC1;//__HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_CC1);

  /* Enable the Peripheral */
  TIM1->CR1 |= TIM_CR1_CEN;//__HAL_TIM_ENABLE(&TimHandle);

}


/////////////////////////////////////////////////////////////////////////////////////////ne hvat func
int State = 0;
/**
 * @brief  Sends an amount of data
 * @param  huart: UART Emulation handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Amount of data to be sent
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_UART_Emul_Transmit_DMA(uint8_t *pData, uint16_t Size)
{
  uint32_t tmp = 0;

  tmp = State;
  if ((tmp == HAL_UART_EMUL_STATE_READY) || (tmp == HAL_UART_EMUL_STATE_BUSY_RX))
  {
    huart->TxXferSize = Size;
    huart->pTxBuffPtr = pData;
    huart->TxXferCount = 1;
    huart->ErrorCode = HAL_UART_EMUL_ERROR_NONE;

    /* Check if a receive process is ongoing or not */
    if (State == HAL_UART_EMUL_STATE_BUSY_RX)
    {
      State = HAL_UART_EMUL_STATE_BUSY_TX_RX;
    }
    else
    {
      State = HAL_UART_EMUL_STATE_BUSY_TX;
    }

    /* Set the UART Emulation DMA transfer complete callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferCpltCallback = UART_Emul_DMATransmitCplt;

    /* Set the DMA error callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferErrorCallback = UART_Emul_DMAError;

    /* Format first Frame to be sent */
    if (huart->TxXferCount == FIRST_BYTE)
    {
      /* Format Frame to be sent */
      emulatedUARTTransmitFormatFrame(*(pData), (uint32_t*)pFirstBuffer_Tx);

      /* Enable the Capture compare channel */
      //TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);
      tmp = TIM_CCER_CC1E << Channel; // 

      /* Reset the CCxE Bit */
      TIM1->CCER &= ~tmp;

      /* Set or reset the CCxE Bit */ 
      TIM1->CCER |= (uint32_t)(ChannelState << Channel);

      /* Send Frames */
      emulatedUARTTransmitFrame();
    }

    if ((huart->TxXferCount == FIRST_BYTE) && (huart->TxXferCount < Size))
    { 
      /* Format Second Frame to be sent */
      emulatedUARTTransmitFormatFrame(huart, *(pData + huart->TxXferCount), (uint32_t*)pSecondBuffer_Tx);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

#endif
