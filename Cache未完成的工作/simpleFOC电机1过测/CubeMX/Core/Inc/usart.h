/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */
#define USART_REC_LEN 256
extern unsigned char USART_RX_BUF[USART_REC_LEN];
extern unsigned short USART_RX_STA;
void USART2_IRQHandler_User(void);
/* USER CODE END Includes */

extern USART_HandleTypeDef husart2;

void MX_USART2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
