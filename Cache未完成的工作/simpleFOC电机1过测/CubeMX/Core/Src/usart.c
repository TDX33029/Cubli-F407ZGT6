/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
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

USART_HandleTypeDef husart2;

void MX_USART2_Init(void)
{
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  USART2->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART2_Init 2 */
}

void HAL_USART_MspInit(USART_HandleTypeDef* usartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(usartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

void HAL_USART_MspDeInit(USART_HandleTypeDef* usartHandle)
{
  if(usartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);
  }
}

/* USER CODE BEGIN 1 */
void USART2_IRQHandler_User(void)
{
	unsigned char Res;
	if(USART2->SR & USART_SR_RXNE)
	{
		Res = (unsigned char)(USART2->DR & 0xFF);
		if((USART_RX_STA & 0x8000) == 0)
		{
			if(USART_RX_STA & 0x4000)
			{
				if(Res != 0x0a) USART_RX_STA = 0;
				else { USART_RX_STA |= 0x8000; USART_RX_BUF[USART_RX_STA & 0x3FFF] = '\0'; }
			}
			else
			{
				if(Res == 0x0d) USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA & 0x3FFF] = Res;
					USART_RX_STA++;
					if(USART_RX_STA > (USART_REC_LEN - 1)) USART_RX_STA = 0;
				}
			}
		}
	}
}
/* USER CODE END 1 */
