/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stm32f4xx.h"                  // Device header
#include <stdio.h>

/* 串口接收缓冲区 (参考正点原子) */
unsigned char USART_RX_BUF[USART_REC_LEN];     // 接收缓冲
unsigned short USART_RX_STA = 0;               // 接收状态标志
// bit15: 接收完成标志
// bit14: 接收到0x0D
// bit13~0: 接收的字节数

/* printf 重定向到USART2 (轮询发送, 寄存器级) */
int fputc(int ch, FILE *f)
{
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = (ch & 0xFF);
	return ch;
}

/* USER CODE END 0 */

USART_HandleTypeDef husart2;

/* USART2 init function */
void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* 使能USART2接收中断 (寄存器级) */
  USART2->CR1 |= USART_CR1_RXNEIE;

  /* USER CODE END USART2_Init 2 */

}

void HAL_USART_MspInit(USART_HandleTypeDef* usartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(usartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    PD7     ------> USART2_CK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_USART_MspDeInit(USART_HandleTypeDef* usartHandle)
{

  if(usartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    PD7     ------> USART2_CK
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USART2中断服务函数 (直接在stm32f4xx_it.c中调用) */
void USART2_IRQHandler_User(void)
{
	unsigned char Res;

	if(USART2->SR & USART_SR_RXNE)  // 接收到数据
	{
		Res = (unsigned char)(USART2->DR & 0xFF);

		if((USART_RX_STA & 0x8000) == 0)  // 接收未完成
		{
			if(USART_RX_STA & 0x4000)     // 已接收到0x0d
			{
				if(Res != 0x0a)
					USART_RX_STA = 0;       // 接收错误，重新开始
				else
				{
					USART_RX_STA |= 0x8000;  // 接收完成
					USART_RX_BUF[USART_RX_STA & 0x3FFF] = '\0';
				}
			}
			else  // 还没收到0x0d
			{
				if(Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA & 0x3FFF] = Res;
					USART_RX_STA++;
					if(USART_RX_STA > (USART_REC_LEN - 1))
						USART_RX_STA = 0;   // 溢出，重新开始
				}
			}
		}
	}
}

/* USER CODE END 1 */
