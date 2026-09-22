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
float target = 0.0f;       // SimpleFOC global target (sets all motors)
float target_m1 = 0.0f;    // M1 individual target
float target_m2 = 0.0f;    // M2 individual target
float target_m3 = 0.0f;    // M3 individual target
uint8_t m1_enabled = 1;    // M1 驱动使能标志
uint8_t m2_enabled = 1;    // M2 驱动使能标志
uint8_t m3_enabled = 1;    // M3 驱动使能标志
uint8_t telemetry_enabled = 1; // 周期遥测上报开关 (默认开启)
uint32_t tele_counter = 0; // 遥测计时分频计数器

/* 陀螺仪与加速度计数据缓存 */
float imu_accel[3] = {0.0f, 0.0f, 0.0f}; // 单位: g
float imu_gyro[3]  = {0.0f, 0.0f, 0.0f}; // 单位: °/s (dps)
uint8_t imu_online = 0;                  // 传感器在线状态 (1: 在线, 0: 离线)
uint8_t imu_active_type = ACTIVE_IMU;    // 1: LSM6DSRTR, 2: MPU6050

/* 三轴霍尔/磁编码角度读数 (MT6701-1, MT6701-2, MT6701-3) 为闭环做准备 */
int16_t hall_raw[3] = {0, 0, 0};         // 原始 14-bit 数值 (0 ~ 16383)
float hall_angle_deg[3] = {0.0f, 0.0f, 0.0f}; // 角度 (0.00 ~ 359.99°)
uint8_t hall_online[3] = {0, 0, 0};      // 传感器在线标志 (1: 正常, 0: 离线)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void commander_run(void);
void set_motor_enable(uint8_t motor, uint8_t en);
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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	printf("\r\n--- STM32F407 System Clock: %ld Hz ---\r\n", HAL_RCC_GetHCLKFreq());
	printf("--- APB1 Timer Clock: %ld Hz ---\r\n", HAL_RCC_GetPCLK1Freq() * 2);
	printf("--- APB2 Timer Clock: %ld Hz ---\r\n", HAL_RCC_GetPCLK2Freq() * 2);

	/* 硬件安全配置检查(确保PD0/PD1/PD2被正确配置为推挽输出) */
	Motor_GPIO_Safety_Check();

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

	set_motor_enable(0, 1);         // 使能三路 DRV8313 (M1, M2, M3)
	printf("3 motors ready.\r\n");

	/* 初始化陀螺仪/IMU */
	if (imu_active_type == IMU_TYPE_LSM6DSR)
	{
		printf("Init LSM6DSRTR on SPI2 (PB12=CS, PB13=SCK, PB14=MISO, PB15=MOSI)...\r\n");
		if (LSM6DSR_Init() == 0)
		{
			imu_online = 1;
			printf("LSM6DSRTR ready! WHO_AM_I=0x%02X\r\n", LSM6DSR_Read_ID());
		}
		else
		{
			imu_online = 0;
			printf("LSM6DSRTR init failed or not responding (WHO_AM_I=0x%02X)\r\n", LSM6DSR_Read_ID());
		}
	}
	else if (imu_active_type == IMU_TYPE_MPU6050)
	{
		printf("Init MPU-6050 on soft I2C (PB10=SCL, PB11=SDA)...\r\n");
		if (MPU6050_Init() == 0)
		{
			imu_online = 1;
			printf("MPU-6050 ready! WHO_AM_I=0x%02X\r\n", MPU6050_Read_ID());
		}
		else
		{
			imu_online = 0;
			printf("MPU-6050 init failed (WHO_AM_I=0x%02X)\r\n", MPU6050_Read_ID());
		}
	}

	/* MT6701 磁编码器上电自检 */
	printf("Init MT6701 encoders on I2C1 (PB6/PB7), I2C2 (PF1/PF0), I2C3 (PA8/PC9)...\r\n");
	hall_online[0] = (i2c_mt6701_1_get_angle(&hall_raw[0], &hall_angle_deg[0]) == 0);
	hall_online[1] = (i2c_mt6701_2_get_angle(&hall_raw[1], &hall_angle_deg[1]) == 0);
	hall_online[2] = (i2c_mt6701_3_get_angle(&hall_raw[2], &hall_angle_deg[2]) == 0);
	printf("MT6701: M1=%s(%.1f deg), M2=%s(%.1f deg), M3=%s(%.1f deg)\r\n",
		hall_online[0] ? "ONLINE" : "OFFLINE", hall_angle_deg[0],
		hall_online[1] ? "ONLINE" : "OFFLINE", hall_angle_deg[1],
		hall_online[2] ? "ONLINE" : "OFFLINE", hall_angle_deg[2]);

	systick_CountMode();            // SysTick循环计数模式(不能再调用delay_us/ms和HAL_Delay)
	target = 0.0f;                  // 上电初始保持静止，等待上位机或终端设定
	target_m1 = target;
	target_m2 = target;
	target_m3 = target;

	/* 启动TIM3/TIM4 PWM输出 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
		/* 25ms (40Hz) 高速周期任务：读取陀螺仪与MT6701编码器、遥测上报与工作指示 */
		if(time1_cntr >= 25)
		{
			time1_cntr = 0;
			tele_counter++;

			/* 读取六轴 IMU 数据 */
			if (imu_online)
			{
				if (imu_active_type == IMU_TYPE_LSM6DSR)
				{
					LSM6DSR_Get_Data(imu_accel, imu_gyro);
				}
				else if (imu_active_type == IMU_TYPE_MPU6050)
				{
					MPU6050_Get_Data(imu_accel, imu_gyro);
				}
			}

			/* 读取三个电机霍尔/磁编码角度 (MT6701-1, MT6701-2, MT6701-3) */
			hall_online[0] = (i2c_mt6701_1_get_angle(&hall_raw[0], &hall_angle_deg[0]) == 0);
			hall_online[1] = (i2c_mt6701_2_get_angle(&hall_raw[1], &hall_angle_deg[1]) == 0);
			hall_online[2] = (i2c_mt6701_3_get_angle(&hall_raw[2], &hall_angle_deg[2]) == 0);

			if(telemetry_enabled)
			{
				/* 扩展标准化遥测格式 (包含三电机目标速度、相电压、使能状态、电角度，以及六轴陀螺仪角速度°/s与加速度g，以及三轴霍尔/磁编码角度H1/H2/H3) */
				printf("$TELE,M1:%.2f,M2:%.2f,M3:%.2f,Vq:%.2f,EN:%d%d%d,A1:%.2f,A2:%.2f,A3:%.2f,Gx:%.1f,Gy:%.1f,Gz:%.1f,Ax:%.2f,Ay:%.2f,Az:%.2f,H1:%.1f,H2:%.1f,H3:%.1f#\r\n",
					target_m1, target_m2, target_m3,
					voltage_limit,
					m1_enabled, m2_enabled, m3_enabled,
					shaft_angle[0], shaft_angle[1], shaft_angle[2],
					imu_gyro[0], imu_gyro[1], imu_gyro[2],
					imu_accel[0], imu_accel[1], imu_accel[2],
					hall_angle_deg[0], hall_angle_deg[1], hall_angle_deg[2]);
			}

			/* 200ms (8次) 翻转LED指示灯 */
			if((tele_counter % 8) == 0)
			{
				HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);  // PG0 LED 心跳
				if(m2_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);  // PG1 M2 指示
				if(m3_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);  // PG2 M3 指示
			}

			/* 1000ms (40次) 输出原有控制台可读格式(仅在关闭TELE时输出，避免报文混淆) */
			if(tele_counter >= 40)
			{
				tele_counter = 0;
				if(!telemetry_enabled)
				{
					printf("M1:%.3f[%lu %lu %lu]  M2:%.3f[%lu %lu %lu]  M3:%.3f[%lu %lu %lu]  Vq=%.2f\r\n",
						shaft_angle[0],
						TIM1->CCR1, TIM1->CCR2, TIM1->CCR3,
						shaft_angle[1],
						TIM3->CCR1, TIM3->CCR2, TIM3->CCR3,
						shaft_angle[2],
						TIM4->CCR1, TIM4->CCR2, TIM4->CCR3,
						voltage_limit);
				}
			}
		}

		/* 无刷电机三轴驱动更新 (若使能则输出FOC开环矢量PWM，若失能则关闭PWM输出) */
		if(m1_enabled) move(target_m1, 0); else { TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; }
		if(m2_enabled) move(target_m2, 1); else { TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0; }
		if(m3_enabled) move(target_m3, 2); else { TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0; }

		/* 串口命令解析处理 */
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
/* 控制三路电机 DRV8313 驱动使能状态 (motor: 0=全部, 1=M1, 2=M2, 3=M3) */
void set_motor_enable(uint8_t motor, uint8_t en)
{
	if(motor == 0 || motor == 1)
	{
		m1_enabled = en ? 1 : 0;
		if(m1_enabled) { M1_Enable; }
		else { M1_Disable; TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; }
	}
	if(motor == 0 || motor == 2)
	{
		m2_enabled = en ? 1 : 0;
		if(m2_enabled) { M2_Enable; }
		else { M2_Disable; TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0; }
	}
	if(motor == 0 || motor == 3)
	{
		m3_enabled = en ? 1 : 0;
		if(m3_enabled) { M3_Enable; }
		else { M3_Disable; TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0; }
	}
}

/******************************************************************************/
/* 串口控制指令解析引擎 */
void commander_run(void)
{
	if((USART_RX_STA & 0x8000) != 0)
	{
		char *cmd = (char *)USART_RX_BUF;
		while(*cmd == ' ' || *cmd == '\t') cmd++; // 去除前导空格

		if(cmd[0] != '\0')
		{
			/* 1. 三轴联合或独立速度指令: M <v1> <v2> <v3> 或 M1 <v>, M2 <v>, M3 <v> */
			if(cmd[0] == 'M' || cmd[0] == 'm')
			{
				if(cmd[1] == '1')
				{
					target_m1 = (float)atof(cmd + 2);
					printf("OK M1:%.2f\r\n", target_m1);
				}
				else if(cmd[1] == '2')
				{
					target_m2 = (float)atof(cmd + 2);
					printf("OK M2:%.2f\r\n", target_m2);
				}
				else if(cmd[1] == '3')
				{
					target_m3 = (float)atof(cmd + 2);
					printf("OK M3:%.2f\r\n", target_m3);
				}
				else
				{
					float v1 = 0.0f, v2 = 0.0f, v3 = 0.0f;
					int n = sscanf(cmd + 1, "%f %f %f", &v1, &v2, &v3);
					if(n == 3)
					{
						target_m1 = v1;
						target_m2 = v2;
						target_m3 = v3;
						printf("OK M1:%.2f,M2:%.2f,M3:%.2f\r\n", target_m1, target_m2, target_m3);
					}
					else if(n == 1)
					{
						target = v1;
						target_m1 = v1;
						target_m2 = v1;
						target_m3 = v1;
						printf("OK ALL:%.2f\r\n", target);
					}
					else if(n == 2)
					{
						target_m1 = v1;
						target_m2 = v2;
						printf("OK M1:%.2f,M2:%.2f\r\n", target_m1, target_m2);
					}
					else
					{
						printf("ERR M format (Usage: M <v1> <v2> <v3> or M1 <val>)\r\n");
					}
				}
			}
			/* 2. 紧急全停: STOP 或 S */
			else if(strncmp(cmd, "STOP", 4) == 0 || strncmp(cmd, "stop", 4) == 0 ||
			        ((cmd[0] == 'S' || cmd[0] == 's') && (cmd[1] == '\0' || cmd[1] == ' ')))
			{
				target = 0.0f;
				target_m1 = 0.0f;
				target_m2 = 0.0f;
				target_m3 = 0.0f;
				printf("OK STOP\r\n");
			}
			/* 3. 驱动使能控制: EN <e1> <e2> <e3> 或 EN1 <e>, EN2 <e>, EN3 <e> */
			else if(strncmp(cmd, "EN", 2) == 0 || strncmp(cmd, "en", 2) == 0)
			{
				if(cmd[2] == '1')
				{
					int e = atoi(cmd + 3);
					set_motor_enable(1, (uint8_t)e);
					printf("OK EN1:%d\r\n", m1_enabled);
				}
				else if(cmd[2] == '2')
				{
					int e = atoi(cmd + 3);
					set_motor_enable(2, (uint8_t)e);
					printf("OK EN2:%d\r\n", m2_enabled);
				}
				else if(cmd[2] == '3')
				{
					int e = atoi(cmd + 3);
					set_motor_enable(3, (uint8_t)e);
					printf("OK EN3:%d\r\n", m3_enabled);
				}
				else
				{
					int e1 = 1, e2 = 1, e3 = 1;
					int n = sscanf(cmd + 2, "%d %d %d", &e1, &e2, &e3);
					if(n == 3)
					{
						set_motor_enable(1, (uint8_t)e1);
						set_motor_enable(2, (uint8_t)e2);
						set_motor_enable(3, (uint8_t)e3);
						printf("OK EN:%d%d%d\r\n", m1_enabled, m2_enabled, m3_enabled);
					}
					else if(n == 1)
					{
						set_motor_enable(0, (uint8_t)e1);
						printf("OK EN:%d%d%d\r\n", m1_enabled, m2_enabled, m3_enabled);
					}
					else
					{
						printf("ERR EN format (Usage: EN <e1> <e2> <e3>)\r\n");
					}
				}
			}
			/* 4. 兼容单电机原始命令: 1<val>, 2<val>, 3<val> */
			else if(cmd[0] == '1')
			{
				target_m1 = (float)atof(cmd + 1);
				printf("OK M1:%.2f\r\n", target_m1);
			}
			else if(cmd[0] == '2')
			{
				target_m2 = (float)atof(cmd + 1);
				printf("OK M2:%.2f\r\n", target_m2);
			}
			else if(cmd[0] == '3')
			{
				target_m3 = (float)atof(cmd + 1);
				printf("OK M3:%.2f\r\n", target_m3);
			}
			/* 5. 兼容全局目标设定: T<val> */
			else if(cmd[0] == 'T' || cmd[0] == 't')
			{
				target = (float)atof(cmd + 1);
				target_m1 = target;
				target_m2 = target;
				target_m3 = target;
				printf("OK ALL:%.2f\r\n", target);
			}
			/* 6. 相电压上限设定: U<val> */
			else if(cmd[0] == 'U' || cmd[0] == 'u')
			{
				float v = (float)atof(cmd + 1);
				if(v < 0.1f) v = 0.1f;
				if(v > 6.0f) v = 6.0f;
				voltage_limit = v;
				printf("OK Vq:%.2f\r\n", voltage_limit);
			}
			/* 7. 速度上限设定: L<val> */
			else if(cmd[0] == 'L' || cmd[0] == 'l')
			{
				float l = (float)atof(cmd + 1);
				if(l < 1.0f) l = 1.0f;
				if(l > 50.0f) l = 50.0f;
				velocity_limit = l;
				printf("OK Vlim:%.2f\r\n", velocity_limit);
			}
			/* 8. 遥测输出开关: TELE <0/1> */
			else if(strncmp(cmd, "TELE", 4) == 0 || strncmp(cmd, "tele", 4) == 0)
			{
				telemetry_enabled = (uint8_t)(atoi(cmd + 4) != 0);
				printf("OK TELE:%d\r\n", telemetry_enabled);
			}
			/* 9. 陀螺仪与加速度计状态查询: IMU */
			else if(strncmp(cmd, "IMU", 3) == 0 || strncmp(cmd, "imu", 3) == 0)
			{
				uint8_t id = 0;
				if (imu_active_type == IMU_TYPE_LSM6DSR) id = LSM6DSR_Read_ID();
				else if (imu_active_type == IMU_TYPE_MPU6050) id = MPU6050_Read_ID();
				printf("--- IMU Sensor Status ---\r\n"
				       "Type: %s, Online: %d, WHO_AM_I: 0x%02X\r\n"
				       "Gyro (dps):  Gx=%.2f, Gy=%.2f, Gz=%.2f\r\n"
				       "Accel (g):   Ax=%.3f, Ay=%.3f, Az=%.3f\r\n",
				       imu_active_type == IMU_TYPE_LSM6DSR ? "LSM6DSRTR (SPI2)" : "MPU-6050 (Soft-I2C)",
				       imu_online, id,
				       imu_gyro[0], imu_gyro[1], imu_gyro[2],
				       imu_accel[0], imu_accel[1], imu_accel[2]);
			}
			/* 10. 切换活动传感器: SENSOR <1/2> (1=LSM6DSR, 2=MPU6050) */
			else if(strncmp(cmd, "SENSOR", 6) == 0 || strncmp(cmd, "sensor", 6) == 0)
			{
				int t = atoi(cmd + 6);
				if (t == 1 || t == 2)
				{
					imu_active_type = (uint8_t)t;
					if (imu_active_type == IMU_TYPE_LSM6DSR) imu_online = (LSM6DSR_Init() == 0);
					else imu_online = (MPU6050_Init() == 0);
					printf("OK SENSOR:%d online=%d\r\n", imu_active_type, imu_online);
				}
				else
				{
					printf("ERR SENSOR format (Usage: SENSOR 1 for LSM6DSR, SENSOR 2 for MPU6050)\r\n");
				}
			}
			/* 11. 霍尔/磁编码角度查询: HALL 或 ENC */
			else if(strncmp(cmd, "HALL", 4) == 0 || strncmp(cmd, "hall", 4) == 0 ||
			        strncmp(cmd, "ENC", 3) == 0 || strncmp(cmd, "enc", 3) == 0)
			{
				printf("--- Motor Hall/Encoder Sensors (MT6701) ---\r\n"
				       "M1 (I2C1): Raw=%d (0x%04X), Angle=%.2f deg, Online=%d\r\n"
				       "M2 (I2C2): Raw=%d (0x%04X), Angle=%.2f deg, Online=%d\r\n"
				       "M3 (I2C3): Raw=%d (0x%04X), Angle=%.2f deg, Online=%d\r\n",
				       hall_raw[0], (uint16_t)hall_raw[0], hall_angle_deg[0], hall_online[0],
				       hall_raw[1], (uint16_t)hall_raw[1], hall_angle_deg[1], hall_online[1],
				       hall_raw[2], (uint16_t)hall_raw[2], hall_angle_deg[2], hall_online[2]);
			}
			/* 12. 帮助信息与当前状态: H / ? / HELP */
			else if(cmd[0] == 'H' || cmd[0] == 'h' || cmd[0] == '?' ||
			        strncmp(cmd, "HELP", 4) == 0 || strncmp(cmd, "help", 4) == 0)
			{
				printf("--- Cubli 3-Motor Controller ---\r\n"
				       "M <v1> <v2> <v3>  Set 3 motor speeds (rad/s)\r\n"
				       "M1/M2/M3 <val>   Set M1/M2/M3 speed\r\n"
				       "STOP             Emergency stop (all 0)\r\n"
				       "EN <e1> <e2> <e3> Enable/disable DRV8313 (1/0)\r\n"
				       "U<val>           Set voltage limit (0.1~6.0V) [curr=%.2f]\r\n"
				       "L<val>           Set velocity limit [curr=%.2f]\r\n"
				       "TELE <0/1>       Toggle telemetry stream [curr=%d]\r\n"
				       "IMU              Query 6-axis gyroscope and accelerometer\r\n"
				       "HALL / ENC       Query 3 motor Hall/encoder angles (MT6701)\r\n"
				       "SENSOR <1/2>     Switch sensor (1:LSM6DSR, 2:MPU6050) [curr=%d]\r\n"
				       "State: M1=%.2f(en=%d) M2=%.2f(en=%d) M3=%.2f(en=%d)\r\n",
				       voltage_limit, velocity_limit, telemetry_enabled, imu_active_type,
				       target_m1, m1_enabled, target_m2, m2_enabled, target_m3, m3_enabled);
			}
			else
			{
				printf("ERR Unknown command: %s\r\n", cmd);
			}
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
#ifdef USE_FULL_ASSERT
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
