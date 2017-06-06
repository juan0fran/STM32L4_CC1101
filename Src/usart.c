/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

uint8_t UartQueueBufferTx[ 128 ];
osStaticMessageQDef_t UartQueueTxControlBlock;
osMessageQId UartTxQueueHandle;

uint8_t UartQueueBufferRx[ 128 ];
osStaticMessageQDef_t UartQueueRxControlBlock;
osMessageQId UartRxQueueHandle;

static HAL_StatusTypeDef HAL_UART_ENABLE_Receive_IT(UART_HandleTypeDef *huart);

void usart_init_rx(void)
{
	osMessageQStaticDef(rx, 128, uint8_t, UartQueueBufferRx, &UartQueueRxControlBlock);
	UartRxQueueHandle = osMessageCreate(osMessageQ(rx), osThreadGetId());
	HAL_UART_ENABLE_Receive_IT(&huart1);
}

void usart_init_tx(void)
{
	osMessageQStaticDef(tx, 128, uint8_t, UartQueueBufferTx, &UartQueueTxControlBlock);
	UartTxQueueHandle = osMessageCreate(osMessageQ(tx), osThreadGetId());
}

static HAL_StatusTypeDef HAL_UART_ENABLE_Receive_IT(UART_HandleTypeDef *huart)
{
  /* Check that a Rx process is not already ongoing */
  if(huart->RxState == HAL_UART_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(huart);


    /* Computation of UART mask to apply to RDR register */
    UART_MASK_COMPUTATION(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Enable the UART Parity Error and Data Register not empty Interrupts */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

void HAL_UART_RxCallback(UART_HandleTypeDef *huart)
{
	/* message put, thats all */
	osMessagePut(UartRxQueueHandle, READ_REG(huart->Instance->RDR), 0);
}

void HAL_UART_TxEndCallback(UART_HandleTypeDef *huart)
{
	/* Disable the UART Transmit Complete Interrupt */
	CLEAR_BIT(huart1.Instance->CR1, USART_CR1_TCIE);
	/* Tx process is ended, restore huart->gState to Ready */
	huart1.gState = HAL_UART_STATE_READY;
	osSignalSet(UsartTxHandle, IFACE_NOTIFY_TX_END);
}

void HAL_UART_TxCallback(UART_HandleTypeDef *huart)
{
	/* message put, thats all */
  uint16_t* tmp;

  /* Check that a Tx process is ongoing */
  if (huart->gState == HAL_UART_STATE_BUSY_TX)
  {
	if(huart->TxXferCount == 0)
	{
	  /* Disable the UART Transmit Data Register Empty Interrupt */
	  CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);

	  /* Enable the UART Transmit Complete Interrupt */
	  SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);

	  return;
	}
	else
	{
	  if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
	  {
		tmp = (uint16_t*) huart->pTxBuffPtr;
		huart->Instance->TDR = (*tmp & (uint16_t)0x01FF);
		huart->pTxBuffPtr += 2;
	  }
	  else
	  {
		huart->Instance->TDR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0xFF);
	  }
	  huart->TxXferCount--;

	  return;
	}
  }
  else
  {
	return;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	/* reinit */
	uint32_t cr1its     = READ_REG(huart1.Instance->CR1);
	uint32_t isrflags   = READ_REG(huart1.Instance->ISR);
	uint32_t errorflags;
	uint32_t cr3its;

	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));

	  cr3its = READ_REG(huart->Instance->CR3);
	  if(   (errorflags != RESET)
		 && (   ((cr3its & USART_CR3_EIE) != RESET)
			 || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)) )
	  {
		/* UART parity error interrupt occurred -------------------------------------*/
		if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
		{
		  __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);

		  huart->ErrorCode |= HAL_UART_ERROR_PE;
		}

		/* UART frame error interrupt occurred --------------------------------------*/
		if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
		{
		  __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

		  huart->ErrorCode |= HAL_UART_ERROR_FE;
		}

		/* UART noise error interrupt occurred --------------------------------------*/
		if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
		{
		  __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

		  huart->ErrorCode |= HAL_UART_ERROR_NE;
		}

		/* UART Over-Run interrupt occurred -----------------------------------------*/
		if(((isrflags & USART_ISR_ORE) != RESET) &&
		   (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
		{
		  __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

		  huart->ErrorCode |= HAL_UART_ERROR_ORE;
		}

		/* Call UART Error Call back function if need be --------------------------*/
		if(huart->ErrorCode != HAL_UART_ERROR_NONE)
		{
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			HAL_UART_ENABLE_Receive_IT(huart);
		}
		return;
	  } /* End if some error occurs */
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
