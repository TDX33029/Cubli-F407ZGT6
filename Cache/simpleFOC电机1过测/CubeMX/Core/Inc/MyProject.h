#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"

#include "delay.h"
#include "timer.h"

#include "foc_utils.h"
#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "CurrentSense.h"

/* 调试串口重定向 */
#include <stdio.h>

/******************************************************************************/
/* DRV8313 使能引脚: PD0=M1, PD1=M2, PD2=M3 */
#define M1_Enable    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET)
#define M1_Disable   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET)
#define M2_Enable    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET)
#define M2_Disable   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET)
#define M3_Enable    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)
#define M3_Disable   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)

/* 外部变量声明 */
extern uint32_t time1_cntr;

#endif
