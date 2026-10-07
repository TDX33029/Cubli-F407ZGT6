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

/* ==========================================================================
 * 调试链路串口选择 (printf 遥测与命令解析共用的物理通道)
 *   1 = USART1 (PA9=TX, PA10=RX) -> 板载 ESP32-WROOM32E 无线上位机链路
 *       (ESP32 IO26<-F407 PA9(TX), ESP32 IO25->F407 PA10(RX), 115200-8-N-1)
 *   2 = USART2 (PD5=TX, PD6=RX)  -> 原有线串口直连 PC (调试后备通道)
 * ========================================================================== */
#define DEBUG_LINK_USART   1

#if DEBUG_LINK_USART == 1
  #define DBG_LINK_UART    USART1
#else
  #define DBG_LINK_UART    USART2
#endif

/* 非阻塞高速环形发送缓冲区 (彻底杜绝 printf 阻塞 15ms 造成 FOC 控制环顿挫脉冲) */
#define UART_TX_BUF_SIZE 1024
static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;

int fputc(int ch, FILE *f)
{
	uint16_t next_head = (uart_tx_head + 1) % UART_TX_BUF_SIZE;
	while(next_head == uart_tx_tail)
	{
		DBG_LINK_UART->CR1 |= USART_CR1_TXEIE; // 满缓冲区时触发发送
	}
	uart_tx_buf[uart_tx_head] = (uint8_t)ch;
	uart_tx_head = next_head;
	DBG_LINK_UART->CR1 |= USART_CR1_TXEIE; // 使能发送中断，异步后台发送
	return ch;
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;

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
#if DEBUG_LINK_USART == 2
  USART2->CR1 |= USART_CR1_RXNEIE;   /* 仅当 USART2 被选为调试链路时才使能接收中断 */
#endif
  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN USART1_Custom */
/* USART1 init function: 板载 ESP32-WROOM32E 无线上位机链路 (PA9=TX->ESP32 IO26, PA10=RX<-ESP32 IO25) */
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
#if DEBUG_LINK_USART == 1
  USART1->CR1 |= USART_CR1_RXNEIE;   /* USART1 作为调试链路时使能接收中断 */
#endif
}
/* USER CODE END USART1_Custom */

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
  /* USER CODE BEGIN USART1_MspInit */
  if(uartHandle->Instance==USART1)
  {
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX (接 ESP32 IO26, ESP32 侧为 RX)
    PA10    ------> USART1_RX (接 ESP32 IO25, ESP32 侧为 TX)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  /* USER CODE END USART1_MspInit */
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

/* 调试链路公共收发中断处理 (作用于 DEBUG_LINK_UART 选定的 USART1 或 USART2)
 * 1. 发送空中断: 异步高效消费环形发送缓冲区，零等待
 * 2. 接收中断: \r\n 结尾的 ASCII 命令行组包 -> USART_RX_BUF / USART_RX_STA
 */
void USART_Link_IRQHandler(void)
{
	unsigned char Res;

	// 1. 发送空中断 (异步消费环形发送缓冲区)
	if((DBG_LINK_UART->CR1 & USART_CR1_TXEIE) && (DBG_LINK_UART->SR & USART_SR_TXE))
	{
		if(uart_tx_head != uart_tx_tail)
		{
			DBG_LINK_UART->DR = uart_tx_buf[uart_tx_tail];
			uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUF_SIZE;
		}
		else
		{
			DBG_LINK_UART->CR1 &= ~USART_CR1_TXEIE; // 发送完毕关闭发送中断
		}
	}

	// 2. 接收中断
	if(DBG_LINK_UART->SR & USART_SR_RXNE)
	{
		Res = (unsigned char)(DBG_LINK_UART->DR & 0xFF);
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

/* 兼容旧中断文件命名 (stm32f4xx_it.c 的 USART2_IRQHandler 调用) */
void USART2_IRQHandler_User(void)
{
	USART_Link_IRQHandler();
}
/* USER CODE END 1 */

