/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "MyProject.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float target;  // SimpleFOC target value (shared with commander)
uint32_t debug_timer;  // 调试输出计时
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void commander_run(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USART2_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	printf("\r\n--- STM32F407 System Clock: %ld Hz ---\r\n", HAL_RCC_GetHCLKFreq());
	printf("--- APB1 Timer Clock: %ld Hz ---\r\n", HAL_RCC_GetPCLK1Freq() * 2);
	printf("--- APB2 Timer Clock: %ld Hz ---\r\n", HAL_RCC_GetPCLK2Freq() * 2);

	/* 初始化SimpleFOC相关硬件 */
	/* TIM1 configured by MX_TIM1_Init() */
	TIM6_1ms_Init();           // 1ms定时器中断

	delay_ms(1000);            // 等待系统稳定

	/* SimpleFOC参数配置 */
	voltage_power_supply = 12.0f;   // V
	voltage_limit = 2.5f;           // V，最大值需小于12/1.732=6.9
	velocity_limit = 20.0f;         // rad/s
	controller = Type_velocity_openloop;
	pole_pairs = 7;                 // 极对数

	M1_Enable;                      // 使能DRV8313-1
	printf("Motor ready.\r\n");

	systick_CountMode();            // SysTick循环计数模式(不能再调用delay_us/ms和HAL_Delay)
	target = 1.0f;                  // 上电后以 1 rad/s 转动

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
		if(time1_cntr >= 200)  // 0.2s LED闪烁
		{
			time1_cntr = 0;
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);  // PG0 LED
			debug_timer++;
			/* 每1秒打印一次状态 */
			if(debug_timer >= 5)
			{
				debug_timer = 0;
				printf("angle=%.3f  CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
					shaft_angle,
					TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
			}
		}
		move(target);
		commander_run();
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/******************************************************************************/
/* 串口命令解析 (来自原F1项目的loop222) */
void commander_run(void)
{
	if((USART_RX_STA & 0x8000) != 0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'U':   // U5.0 - set voltage_limit
				voltage_limit = atof((const char *)(USART_RX_BUF + 1));
				printf("voltage_limit=%.4f\r\n", voltage_limit);
				break;
			case 'T':   // T6.28  — 设置目标值
				target = atof((const char *)(USART_RX_BUF + 1));
				printf("RX=%.4f\r\n", target);
				break;
		}
		USART_RX_STA = 0;
	}
}
/******************************************************************************/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
