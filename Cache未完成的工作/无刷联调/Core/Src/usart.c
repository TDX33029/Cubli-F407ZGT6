/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stm32f4xx.h"
#include <stdio.h>

unsigned char USART_RX_BUF[USART_REC_LEN];
unsigned short USART_RX_STA = 0;

int fputc(int ch, FILE *f)
{
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = (ch & 0xFF);
	return ch;
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  USART2->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void usart2_send_str(const char *str)
{
	while(str && *str)
	{
		while(!(USART2->SR & USART_SR_TXE));
		USART2->DR = (uint8_t)(*str++ & 0xFF);
	}
}

void USART2_IRQHandler_User(void)
{
	unsigned char Res;
	if(USART2->SR & USART_SR_RXNE)
	{
		Res = (unsigned char)(USART2->DR & 0xFF);
		if((USART_RX_STA & 0x8000) == 0) // 接收未完成
		{
			if(Res == 0x0D) // '\r'
			{
				USART_RX_STA |= 0x4000;
			}
			else if(Res == 0x0A) // '\n'
			{
				// 收到换行，无论前一个字符是否为 \r，均视为指令结束
				uint16_t len = USART_RX_STA & 0x3FFF;
				USART_RX_BUF[len] = '\0';
				USART_RX_STA |= 0x8000;
			}
			else
			{
				uint16_t len = USART_RX_STA & 0x3FFF;
				if(len < (USART_REC_LEN - 1))
				{
					USART_RX_BUF[len] = Res;
					USART_RX_STA = (USART_RX_STA & 0x4000) | (len + 1);
				}
				else
				{
					// 溢出丢弃重置
					USART_RX_STA = 0;
				}
			}
		}
	}
}
/* USER CODE END 1 */

