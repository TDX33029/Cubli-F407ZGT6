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
void Boot_PG4_Blink_5s(void);
void Motor_Encoder_OpenLoop_SelfTest(void);
void Motor_Measure_PolePairs(uint8_t motor);
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

		/* 初始化TIM5 32位全硬件微秒定时器与TIM6 1ms时基 */
		TIM5_Micros_Init();
		TIM6_1ms_Init();           // 1ms定时器中断

		/* 上电快速指示 (PG4亮200ms后熄灭，避免开机长时间阻塞，静候上位机指令) */
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
		delay_ms(200);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);

				/* SimpleFOC参数配置 */
				voltage_power_supply = 12.0f;   // V (DRV8313 12V 硬件供电)
				voltage_limit = 6.0f;           // V，相电压上限拉到最高 6.0V (最大安全限幅 6.8V)，极大充沛发挥电磁力矩
				velocity_limit = 60.0f;         // rad/s (最高转速提升至 60.0 rad/s)
				controller = Type_velocity_openloop; // 默认开环安全待机，避免未标定零位时闭环自激扰动
			pole_pairs = 7;                 // 极对数

			SimpleFOC_PID_Init();           // 初始化速度闭环 PID 与低通滤波器
			set_motor_enable(0, 0);         // 上电默认失能三路电机，等待上位机自检指令解锁
			printf("3 motors initialized and IDLE (Safe Lock Mode).\r\n");

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

				/* 启动TIM3/TIM4 PWM输出 (TIM1在MX_TIM1_Init后由Motor_GPIO_Safety_Check使能) */
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

				/* 上电默认保持静止与SysTick微秒时基，不自动执行电机开环自检，等待上位机下发 TEST 检测指令 */
				systick_CountMode();
				printf("System initialized. Motors idle, waiting for UpperComputer Self-Test (TEST)...\r\n");

			target = 0.0f;                  // 上电初始保持静止，等待上位机或终端设定
		target_m1 = target;
		target_m2 = target;
		target_m3 = target;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
			/* 40ms (25Hz) 周期任务：读取陀螺仪、遥测上报与工作指示 (降低串口阻塞比重，保障FOC高频连续换向) */
			if(time1_cntr >= 40)
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

				/* 依据动态映射表获取每个电机对应的编码器实时机械角度 (直接读取FOC核心更新的角度，避免总线重复阻塞) */
				float m_angle_deg[3];
				m_angle_deg[0] = angle_prev[0] * 180.0f / _PI;
				m_angle_deg[1] = angle_prev[1] * 180.0f / _PI;
				m_angle_deg[2] = angle_prev[2] * 180.0f / _PI;

				if(telemetry_enabled)
				{
					/* 扩展标准化遥测格式 (包含三电机目标速度、相电压、使能状态、电角度、六轴陀螺仪加速度、通道映射后MT6701角度H1/H2/H3，以及实测滤波转速S1/S2/S3) */
					printf("$TELE,M1:%.2f,M2:%.2f,M3:%.2f,Vq:%.2f,EN:%d%d%d,A1:%.2f,A2:%.2f,A3:%.2f,Gx:%.1f,Gy:%.1f,Gz:%.1f,Ax:%.2f,Ay:%.2f,Az:%.2f,H1:%.1f,H2:%.1f,H3:%.1f,S1:%.2f,S2:%.2f,S3:%.2f#\r\n",
						target_m1, target_m2, target_m3,
						voltage_limit,
						m1_enabled, m2_enabled, m3_enabled,
						shaft_angle[0], shaft_angle[1], shaft_angle[2],
						imu_gyro[0], imu_gyro[1], imu_gyro[2],
						imu_accel[0], imu_accel[1], imu_accel[2],
						m_angle_deg[0], m_angle_deg[1], m_angle_deg[2],
						shaft_velocity[0], shaft_velocity[1], shaft_velocity[2]);
				}

				/* 200ms (5次) 翻转LED指示灯: PG0对应M1, PG1对应M2, PG2对应M3 */
				if((tele_counter % 5) == 0)
				{
					if(m1_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
					else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);

					if(m2_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
					else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

					if(m3_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
					else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
				}

				/* 1000ms (25次) 输出控制台可读格式(仅在关闭TELE时输出) */
				if(tele_counter >= 25)
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
				/* 1. 自检指令: TEST 或 CHECK (最高优先级匹配) */
				if(strncmp(cmd, "TEST", 4) == 0 || strncmp(cmd, "test", 4) == 0 ||
				   strncmp(cmd, "CHECK", 5) == 0 || strncmp(cmd, "check", 5) == 0)
				{
					printf("\r\n[CMD] Triggering Motor & Encoder Self-Test...\r\n");
					Motor_Encoder_OpenLoop_SelfTest();
				}
				/* 1.1 极对数精确标定指令: PP [1/2/3] */
				else if(strncmp(cmd, "PP", 2) == 0 || strncmp(cmd, "pp", 2) == 0)
				{
					int m_target = 0;
					if(cmd[2] >= '1' && cmd[2] <= '3') m_target = cmd[2] - '1';
					else if(cmd[2] == ' ' && cmd[3] >= '1' && cmd[3] <= '3') m_target = cmd[3] - '1';
					Motor_Measure_PolePairs((uint8_t)m_target);
				}
			/* 2. 遥测流开关: TELE <0/1> */
			else if(strncmp(cmd, "TELE", 4) == 0 || strncmp(cmd, "tele", 4) == 0)
			{
				telemetry_enabled = (uint8_t)(atoi(cmd + 4) != 0);
				printf("OK TELE:%d\r\n", telemetry_enabled);
			}
			/* 3. 控制模式切换: MODE <0/1> (0=开环, 1=闭环) */
			else if(strncmp(cmd, "MODE", 4) == 0 || strncmp(cmd, "mode", 4) == 0)
			{
				int m = atoi(cmd + 4);
				if(m == 1)
				{
					controller = Type_velocity;
					printf("OK MODE:1 (Closed-loop Speed)\r\n");
				}
				else if(m == 0)
				{
					controller = Type_velocity_openloop;
					printf("OK MODE:0 (Open-loop Speed)\r\n");
				}
				else
				{
					printf("ERR MODE format (Usage: MODE 1 for Closed-loop, MODE 0 for Open-loop)\r\n");
				}
			}
			/* 4. 紧急全停: STOP 或 S */
			else if(strncmp(cmd, "STOP", 4) == 0 || strncmp(cmd, "stop", 4) == 0 ||
			        ((cmd[0] == 'S' || cmd[0] == 's') && (cmd[1] == '\0' || cmd[1] == ' ' || cmd[1] == '\t')))
			{
				target = 0.0f;
				target_m1 = 0.0f;
				target_m2 = 0.0f;
				target_m3 = 0.0f;
				printf("OK STOP\r\n");
			}
			/* 5. 传感器切换: SENSOR <1/2> */
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
				/* 6. 霍尔/磁编码角度查询: HALL 或 ENC */
					else if(strncmp(cmd, "HALL", 4) == 0 || strncmp(cmd, "hall", 4) == 0 ||
					        strncmp(cmd, "ENC", 3) == 0 || strncmp(cmd, "enc", 3) == 0)
					{
							int16_t r[3];
							float d[3];
							uint8_t o1 = (i2c_mt6701_1_get_angle(&r[0], &d[0]) == 0);
							uint8_t o2 = (i2c_mt6701_2_get_angle(&r[1], &d[1]) == 0);
							uint8_t o3 = (i2c_mt6701_3_get_angle(&r[2], &d[2]) == 0);
							printf("--- Motor Hall/Encoder Sensors (MT6701) ---\r\n"
							       "M1 (I2C1): Raw=%d (0x%04X), Angle=%.2f deg, Spd=%.2f rad/s, Online=%d\r\n"
							       "M2 (I2C2): Raw=%d (0x%04X), Angle=%.2f deg, Spd=%.2f rad/s, Online=%d\r\n"
							       "M3 (I2C3): Raw=%d (0x%04X), Angle=%.2f deg, Spd=%.2f rad/s, Online=%d\r\n",
							       r[0], (uint16_t)r[0], d[0], shaft_velocity[0], o1,
							       r[1], (uint16_t)r[1], d[1], shaft_velocity[1], o2,
							       r[2], (uint16_t)r[2], d[2], shaft_velocity[2], o3);
					}
			/* 7. 六轴陀螺仪与加速度计状态查询: IMU */
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
			/* 8. I2C 总线硬件扫描: SCAN */
			else if(strncmp(cmd, "SCAN", 4) == 0 || strncmp(cmd, "scan", 4) == 0)
			{
				int addr;
				printf("--- I2C Bus Scan (1-127) ---\r\nI2C1: ");
				for(addr = 1; addr < 128; addr++)
				{
					if(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 5) == HAL_OK)
						printf("0x%02X ", addr);
				}
				printf("\r\nI2C2: ");
				for(addr = 1; addr < 128; addr++)
				{
					if(HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 2, 5) == HAL_OK)
						printf("0x%02X ", addr);
				}
				printf("\r\nI2C3: ");
				for(addr = 1; addr < 128; addr++)
				{
					if(HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(addr << 1), 2, 5) == HAL_OK)
						printf("0x%02X ", addr);
				}
				printf("\r\n--- End Scan ---\r\n");
			}
			/* 9. 电角度零位校准: ALIGN 或 CALIB */
			else if(strncmp(cmd, "ALIGN", 5) == 0 || strncmp(cmd, "align", 5) == 0 ||
			        strncmp(cmd, "CALIB", 5) == 0 || strncmp(cmd, "calib", 5) == 0)
			{
				printf("Starting Motor Alignment...\r\n");
				Motor_alignAll();
				printf("OK ALIGN\r\n");
			}
			/* 9. PID 参数设定: PID <p> <i> <d> 或 PID1/2/3 <p> <i> <d> */
			else if(strncmp(cmd, "PID", 3) == 0 || strncmp(cmd, "pid", 3) == 0)
			{
				float p_val = 0.0f, i_val = 0.0f, d_val = 0.0f;
				int motor_idx = 0;
				if(cmd[3] == '1' || cmd[3] == '2' || cmd[3] == '3')
				{
					motor_idx = cmd[3] - '1';
					int n = sscanf(cmd + 4, "%f %f %f", &p_val, &i_val, &d_val);
					if(n >= 2)
					{
						pid_velocity[motor_idx].P = p_val;
						pid_velocity[motor_idx].I = i_val;
						if(n == 3) pid_velocity[motor_idx].D = d_val;
						printf("OK PID%d: P=%.3f, I=%.3f, D=%.4f\r\n", motor_idx+1, pid_velocity[motor_idx].P, pid_velocity[motor_idx].I, pid_velocity[motor_idx].D);
					}
					else
					{
						printf("ERR PID format (Usage: PID%d <P> <I> [D])\r\n", motor_idx+1);
					}
				}
				else
				{
					int n = sscanf(cmd + 3, "%f %f %f", &p_val, &i_val, &d_val);
					if(n >= 2)
					{
						int k;
						for(k = 0; k < 3; k++)
						{
							pid_velocity[k].P = p_val;
							pid_velocity[k].I = i_val;
							if(n == 3) pid_velocity[k].D = d_val;
						}
						printf("OK ALL PID: P=%.3f, I=%.3f, D=%.4f\r\n", p_val, i_val, d_val);
					}
					else
					{
						printf("ERR PID format (Usage: PID <P> <I> [D])\r\n");
					}
				}
			}
			/* 10. 驱动使能控制: EN <e1> <e2> <e3> 或 EN1 <e>, EN2 <e>, EN3 <e> */
			else if((strncmp(cmd, "EN", 2) == 0 || strncmp(cmd, "en", 2) == 0) &&
			        (cmd[2] == ' ' || cmd[2] == '\t' || cmd[2] == '1' || cmd[2] == '2' || cmd[2] == '3'))
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
			/* 11. 三轴联合或独立速度指令: M <v1> <v2> <v3> 或 M1 <v>, M2 <v>, M3 <v> */
			else if((cmd[0] == 'M' || cmd[0] == 'm') &&
			        (cmd[1] == ' ' || cmd[1] == '\t' || cmd[1] == '1' || cmd[1] == '2' || cmd[1] == '3'))
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
			/* 12. 兼容全局目标设定: T<val> */
			else if((cmd[0] == 'T' || cmd[0] == 't') &&
			        (cmd[1] == ' ' || cmd[1] == '\t' || cmd[1] == '+' || cmd[1] == '-' ||
			         (cmd[1] >= '0' && cmd[1] <= '9') || cmd[1] == '\0'))
			{
				target = (float)atof(cmd + 1);
				target_m1 = target;
				target_m2 = target;
				target_m3 = target;
				printf("OK ALL:%.2f\r\n", target);
			}
				/* 13. 相电压上限设定: U<val> */
				else if(cmd[0] == 'U' || cmd[0] == 'u')
				{
					float v = (float)atof(cmd + 1);
					if(v < 0.1f) v = 0.1f;
					if(v > 6.8f) v = 6.8f;
					voltage_limit = v;
					printf("OK Vq:%.2f\r\n", voltage_limit);
				}
				/* 14. 速度上限设定: L<val> */
				else if(cmd[0] == 'L' || cmd[0] == 'l')
				{
					float l = (float)atof(cmd + 1);
					if(l < 1.0f) l = 1.0f;
					if(l > 80.0f) l = 80.0f;
					velocity_limit = l;
					printf("OK Vlim:%.2f\r\n", velocity_limit);
				}
			/* 15. 兼容单电机原始命令: 1<val>, 2<val>, 3<val> */
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
			/* 16. 帮助信息与当前状态: H / ? / HELP */
			else if(cmd[0] == 'H' || cmd[0] == 'h' || cmd[0] == '?' ||
			        strncmp(cmd, "HELP", 4) == 0 || strncmp(cmd, "help", 4) == 0)
			{
				printf("--- Cubli 3-Motor Controller (Closed-loop Speed) ---\r\n"
				       "M <v1> <v2> <v3>  Set 3 motor speeds (rad/s)\r\n"
				       "M1/M2/M3 <val>   Set M1/M2/M3 speed\r\n"
				       "STOP             Emergency stop (all 0)\r\n"
				       "EN <e1> <e2> <e3> Enable/disable DRV8313 (1/0)\r\n"
				       "MODE <0/1>       Switch Open-loop(0) / Closed-loop(1) [curr=%s]\r\n"
				       "ALIGN / CALIB    Re-align electrical zero angle\r\n"
				       "TEST / CHECK     Run 3-axis open-loop motor & encoder test\r\n"
				       "PID <P> <I> [D]  Set velocity loop PID gains\r\n"
				       "U<val>           Set voltage limit (0.1~6.0V) [curr=%.2f]\r\n"
				       "L<val>           Set velocity limit [curr=%.2f]\r\n"
				       "TELE <0/1>       Toggle telemetry stream [curr=%d]\r\n"
				       "IMU              Query 6-axis gyroscope and accelerometer\r\n"
				       "HALL / ENC       Query 3 motor Hall/encoder angles (MT6701)\r\n"
				       "SENSOR <1/2>     Switch sensor (1:LSM6DSR, 2:MPU6050) [curr=%d]\r\n"
				       "State: Mode=%s M1=%.2f(en=%d,act=%.2f) M2=%.2f(en=%d,act=%.2f) M3=%.2f(en=%d,act=%.2f)\r\n",
				       controller == Type_velocity ? "CLOSED_LOOP" : "OPEN_LOOP",
				       voltage_limit, velocity_limit, telemetry_enabled, imu_active_type,
				       controller == Type_velocity ? "CLOSED_LOOP" : "OPEN_LOOP",
				       target_m1, m1_enabled, shaft_velocity[0],
				       target_m2, m2_enabled, shaft_velocity[1],
				       target_m3, m3_enabled, shaft_velocity[2]);
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
/* 上电闪烁 PG4 的 LED 5 秒 (周期200ms: 100ms亮, 100ms灭, 翻转50次) */
void Boot_PG4_Blink_5s(void)
{
	int i;
	printf("\r\n[BOOT] Flashing PG4 LED for 5 seconds...\r\n");
	for(i = 0; i < 50; i++)
	{
		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
		delay_ms(100);
	}
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET); // 闪烁结束恢复低电平熄灭
	printf("[BOOT] PG4 LED flashing complete. Starting motor & encoder diagnostics...\r\n\r\n");
}

/******************************************************************************/
/* 编码器诊断结果结构体 */
typedef struct {
	uint8_t status;         // 0: PASS, 1: FAIL_I2C_OFFLINE, 2: FAIL_NO_ROTATION
	uint16_t total_reads;   // 采样总次数
	uint16_t success_reads; // 通信成功次数
	float start_deg;        // 起始机械角度
	float end_deg;          // 结束机械角度
	float travel_deg;       // 累计绝对角位移 (多圈解包累加)
} EncoderDiagResult_t;

/******************************************************************************/
/* 低速开环旋转三个电机，全通道同步检测MT6701磁编码器/霍尔工作状态与接线映射 */
void Motor_Encoder_OpenLoop_SelfTest(void)
{
	int m, i, k;
	float saved_voltage_limit = voltage_limit;
	EncoderDiagResult_t diag[3];
	uint8_t all_pass = 1;

	printf("\r\n==============================================================\r\n");
	printf("       STARTING 3-AXIS MOTOR & ENCODER OPEN-LOOP TEST         \r\n");
	printf("==============================================================\r\n");

	/* 1. 自检起始阶段：PG4 LED 以 5Hz 闪烁 1 秒 (100ms亮, 100ms灭, 翻转10次 = 1000ms) */
	printf("[TEST] Phase 1: PG4 LED blinking for 1 second...\r\n");
	for(i = 0; i < 10; i++)
	{
		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
		delay_ms(100);
	}
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
	printf("[TEST] Phase 2: Starting 3-axis sequential motor open-loop rotation...\r\n");

	/* 为测试配置开环参数与安全有效电压 (3.0V 保证克服转子动量轮静态摩擦力与惯量) */
	voltage_limit = 3.0f;
	controller = Type_velocity_openloop;

	/* 确保 SysTick 处于递减计数模式以提供准确微秒时基 */
	systick_CountMode();

	/* 依次测试电机 M1, M2, M3 (m = 0, 1, 2) */
	for(m = 0; m < 3; m++)
	{
		uint16_t motor_led_pin = (m == 0 ? GPIO_PIN_0 : (m == 1 ? GPIO_PIN_1 : GPIO_PIN_2));
		int16_t r[3] = {0, 0, 0};
		float cur_d[3] = {0.0f, 0.0f, 0.0f};
		float prev_d[3] = {0.0f, 0.0f, 0.0f};
		float travel_accum[3] = {0.0f, 0.0f, 0.0f};
		float signed_travel[3] = {0.0f, 0.0f, 0.0f};
		uint8_t enc_ok[3] = {0, 0, 0};
		uint16_t enc_succ[3] = {0, 0, 0};
		uint16_t samples_total = 0;
		uint32_t start_tick, last_sample_tick, last_led_tick;
		float test_velocity = 2.0f;    // 开环目标角速度 2.0 rad/s (约 20 RPM，平稳静音起旋)

		diag[m].status = 0;
		diag[m].total_reads = 0;
		diag[m].success_reads = 0;
		diag[m].start_deg = 0.0f;
		diag[m].end_deg = 0.0f;
		diag[m].travel_deg = 0.0f;

		/* 输出上位机单步进度报文 */
		printf("$TEST_STEP,M%d#\r\n", m + 1);
		printf("[TEST] Testing Motor %d (DRV8313 #%d, PWM Timer TIM%d, LED: PG%d)...\r\n",
		       m + 1, m + 1, (m == 0 ? 1 : (m == 1 ? 3 : 4)), m);

		/* 1. 静态采样三路编码器初始角度 */
		enc_ok[0] = (i2c_mt6701_1_get_angle(&r[0], &cur_d[0]) == 0);
		enc_ok[1] = (i2c_mt6701_2_get_angle(&r[1], &cur_d[1]) == 0);
		enc_ok[2] = (i2c_mt6701_3_get_angle(&r[2], &cur_d[2]) == 0);

		for(k = 0; k < 3; k++)
		{
			prev_d[k] = cur_d[k];
		}
		diag[m].start_deg = cur_d[m];

		/* 2. 仅使能当前被测电机，关闭其他电机 */
		set_motor_enable(0, 0); // 先全关
		set_motor_enable(m + 1, 1); // 开启当前轴
		delay_ms(20);

		/* 重置该电机开环时间戳与角度 */
		open_loop_timestamp[m] = SysTick->VAL;
		shaft_angle[m] = 0.0f;

		/* 3. 开环低速旋转 2000ms，期间高频调制SVPWM，对应电机LED同步闪烁，并每20ms三路编码器全采样 */
		start_tick = HAL_GetTick();
		last_sample_tick = start_tick;
		last_led_tick = start_tick;

		while((HAL_GetTick() - start_tick) < 2000)
		{
			/* 驱动开环旋转换向 */
			velocityOpenloop(test_velocity, m);

			/* 每 100ms 翻转当前正在校准电机的对应指示灯 (M1对应PG0, M2对应PG1, M3对应PG2) */
			if((HAL_GetTick() - last_led_tick) >= 100)
			{
				last_led_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOG, motor_led_pin);
			}

			/* 20ms (50Hz) 全通道采样编码器角度 */
			if((HAL_GetTick() - last_sample_tick) >= 20)
			{
				last_sample_tick = HAL_GetTick();
				samples_total++;

				enc_ok[0] = (i2c_mt6701_1_get_angle(&r[0], &cur_d[0]) == 0);
				enc_ok[1] = (i2c_mt6701_2_get_angle(&r[1], &cur_d[1]) == 0);
				enc_ok[2] = (i2c_mt6701_3_get_angle(&r[2], &cur_d[2]) == 0);

				for(k = 0; k < 3; k++)
				{
					if(enc_ok[k])
					{
						enc_succ[k]++;
						/* 跨越 0/360 度多圈解包计算转过机械角度 */
						float diff = cur_d[k] - prev_d[k];
						if(diff > 180.0f) diff -= 360.0f;
						else if(diff < -180.0f) diff += 360.0f;
						travel_accum[k] += fabsf(diff);
						signed_travel[k] += diff;
						prev_d[k] = cur_d[k];
					}
				}
			}
		}

		/* 4. 停机与相电压清零 */
		setPhaseVoltage(0.0f, 0.0f, 0.0f, m);
		set_motor_enable(m + 1, 0);
		HAL_GPIO_WritePin(GPIOG, motor_led_pin, GPIO_PIN_RESET);

		printf("  [M%d Spin Done] Enc1 moved: %.1f* (comm %d%%), Enc2 moved: %.1f* (comm %d%%), Enc3 moved: %.1f* (comm %d%%)\r\n",
		       m + 1,
		       travel_accum[0], samples_total ? (enc_succ[0] * 100 / samples_total) : 0,
		       travel_accum[1], samples_total ? (enc_succ[1] * 100 / samples_total) : 0,
		       travel_accum[2], samples_total ? (enc_succ[2] * 100 / samples_total) : 0);

		diag[m].total_reads = samples_total;
		diag[m].success_reads = enc_succ[m];
		diag[m].end_deg = cur_d[m];
		diag[m].travel_deg = travel_accum[m];

		/* 5. 判定健康状态：自动识别驱动通道与编码器对应关系及旋向 */
		int matched_enc = -1;
		for(k = 0; k < 3; k++)
		{
			if(travel_accum[k] >= 20.0f && enc_succ[k] >= (samples_total * 8 / 10))
			{
				matched_enc = k;
				break;
			}
		}

		if(matched_enc >= 0)
		{
			motor_sensor_map[m] = (uint8_t)matched_enc;
			diag[m].status = 0; // PASS
			diag[m].travel_deg = travel_accum[matched_enc];
			diag[m].success_reads = enc_succ[matched_enc];
			diag[m].end_deg = cur_d[matched_enc];

				// 辨识旋向: 严格对应开环正压旋转方向
				if(signed_travel[matched_enc] > 0.0f) sensor_direction[m] = 1;
				else sensor_direction[m] = -1;

			printf("  [M%d Calib] Driver %d -> Encoder %d | Dir: %s | Travel: %.1f deg\r\n",
			       m + 1, m + 1, matched_enc + 1,
			       sensor_direction[m] == 1 ? "CW(+1)" : "CCW(-1)",
			       travel_accum[matched_enc]);

				// 立即使能当前电机并精确定位 3PI/2 测定转子零电角 (期间保持对应指示灯闪烁)
				set_motor_enable(m + 1, 1);
				delay_ms(20);
				setPhaseVoltage(voltage_sensor_align, 0, _3PI_2, m);
				for(i = 0; i < 8; i++)
				{
					HAL_GPIO_TogglePin(GPIOG, motor_led_pin);
					delay_ms(100);
				}
				updateSensor(m);
				zero_electric_angle[m] = 0.0f;
				zero_electric_angle[m] = electricalAngle(m);
				setPhaseVoltage(0, 0, 0, m);
				set_motor_enable(m + 1, 0); // 测定后安全失能
				motor_aligned[m] = 1;
				HAL_GPIO_WritePin(GPIOG, motor_led_pin, GPIO_PIN_RESET);

			printf("  [M%d Calib] Zero Elec Angle: %.2f rad (%.1f deg) [CALIBRATED]\r\n",
			       m + 1, zero_electric_angle[m], zero_electric_angle[m] * 180.0f / _PI);
		}
		else
		{
			// 本驱动未造成任何编码器转动
			// 判断是否为未安装通道 (完全无通信或编码器未连接)
			if(enc_succ[m] == 0)
			{
				diag[m].status = 3; // SKIP 未安装
				printf("  [M%d Notice] Motor %d / Encoder is not installed (skip).\r\n", m + 1, m + 1);
			}
			else
			{
				diag[m].status = 2; // FAIL_NO_ROTATION (通信在线但电机未旋转)
				all_pass = 0;
			}
		}

		delay_ms(100);
	}

	/* 计算整体通过状态: 所有已安装的电机全部正常通过即判定通过 */
	all_pass = 1;
	int installed_count = 0;
	for(m = 0; m < 3; m++)
	{
		if(diag[m].status == 0) installed_count++;
		else if(diag[m].status == 1 || diag[m].status == 2)
		{
			all_pass = 0;
			break;
		}
	}
	if(installed_count == 0) all_pass = 0;

	/* 恢复原模式与参数 */
	voltage_limit = saved_voltage_limit;
	systick_CountMode();

	/* 6. 打印格式化诊断报告 */
	printf("\r\n");
	printf("==============================================================\r\n");
	printf("               MOTOR & ENCODER DIAGNOSTICS REPORT             \r\n");
	printf("==============================================================\r\n");
	printf("Motor | Result |  I2C Comm Rate  | Start Deg | End Deg | Travel Deg\r\n");
	printf("--------------------------------------------------------------\r\n");
	for(m = 0; m < 3; m++)
	{
		const char *status_str = "PASS";
		if(diag[m].status == 1) status_str = "FAIL(I2C)";
		else if(diag[m].status == 2) status_str = "FAIL(STALL)";
		else if(diag[m].status == 3) status_str = "SKIP(NONE)";

		printf(" M%d   | %-10s| %3d%% (%2d/%2d)  |  %6.1f*  | %6.1f*  |  %6.1f*\r\n",
		       m + 1,
		       status_str,
		       diag[m].total_reads ? (diag[m].success_reads * 100 / diag[m].total_reads) : 0,
		       diag[m].success_reads, diag[m].total_reads,
		       diag[m].start_deg,
		       diag[m].end_deg,
		       diag[m].travel_deg);
	}
	printf("--------------------------------------------------------------\r\n");

	/* 7. 输出标准结构化遥测报文供上位机解析 */
	printf("$TEST_REPORT,M1:%s:%d:%.1f,M2:%s:%d:%.1f,M3:%s:%d:%.1f,ALL:%s#\r\n",
	       diag[0].status == 0 ? "PASS" : (diag[0].status == 3 ? "SKIP" : (diag[0].status == 1 ? "FAIL_I2C" : "FAIL_STALL")),
	       diag[0].total_reads ? (diag[0].success_reads * 100 / diag[0].total_reads) : 0,
	       diag[0].travel_deg,
	       diag[1].status == 0 ? "PASS" : (diag[1].status == 3 ? "SKIP" : (diag[1].status == 1 ? "FAIL_I2C" : "FAIL_STALL")),
	       diag[1].total_reads ? (diag[1].success_reads * 100 / diag[1].total_reads) : 0,
	       diag[1].travel_deg,
	       diag[2].status == 0 ? "PASS" : (diag[2].status == 3 ? "SKIP" : (diag[2].status == 1 ? "FAIL_I2C" : "FAIL_STALL")),
	       diag[2].total_reads ? (diag[2].success_reads * 100 / diag[2].total_reads) : 0,
	       diag[2].travel_deg,
	       all_pass ? "PASS" : "FAIL");

	if(all_pass)
	{
		printf("Result: >>> ALL INSTALLED MOTORS CALIBRATED & HEALTHY (PASS) <<<\r\n");
		printf("==============================================================\r\n\r\n");
			/* 全部通过：PG4 LED 常亮 1 秒并慢闪 3 次 */
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
			delay_ms(1000);
			for(m = 0; m < 6; m++)
			{
				HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
				delay_ms(150);
			}
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);

			/* 目标转速与 PID 积分器全面复位清零，杜绝切入闭环瞬间产生自检历史积分冲击 */
			target = 0.0f;
			target_m1 = 0.0f;
			target_m2 = 0.0f;
			target_m3 = 0.0f;
			for(m = 0; m < 3; m++)
			{
				pid_velocity[m].error_prev = 0.0f;
				pid_velocity[m].output_prev = 0.0f;
				pid_velocity[m].integral_prev = 0.0f;
				pid_velocity[m].timestamp_prev = 0;
				lpf_velocity[m].y_prev = 0.0f;
				lpf_velocity[m].timestamp_prev = 0;
				shaft_velocity[m] = 0.0f;
			}

			/* 使能已安装通过的电机，并切换到速度闭环模式 */
			controller = Type_velocity;
			for(m = 0; m < 3; m++)
			{
				if(diag[m].status == 0) set_motor_enable(m + 1, 1);
				else set_motor_enable(m + 1, 0);
			}
			printf("Control mode switched to CLOSED_LOOP for calibrated motors.\r\n");
	}
	else
	{
		printf("Result: >>> WARNING: ENCODER/MOTOR ANOMALY DETECTED (FAIL) <<<\r\n");
		printf("Tips: FAIL(I2C) -> Check I2C wiring, 3.3V/GND, pull-ups.\r\n");
		printf("      FAIL(STALL) -> Check magnet distance/polarity, motor wiring or mechanical stall.\r\n");
		printf("==============================================================\r\n\r\n");
		/* 存在异常：PG4 LED 快速急促闪烁 10 次报警 */
		for(m = 0; m < 20; m++)
		{
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
			delay_ms(60);
		}
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
		set_motor_enable(0, 0); // 异常时切断电机输出以策安全
	}

		systick_CountMode();
		printf("OK TEST\r\n");
	}
	/******************************************************************************/
	/* 自动步进微调高精度测量电机物理极对数 (Pole Pairs) */
	void Motor_Measure_PolePairs(uint8_t motor)
	{
		int s;
		float test_voltage = 3.0f;
		int total_cycles = 11;
		int total_steps = 2200;
		float el_step;
		float cur_el;
		int16_t raw_init = 0;
		float deg_init = 0.0f;
		float prev_deg = 0.0f;
		float total_mech_travel = 0.0f;
		uint8_t sens_idx;
		float abs_travel;
		float total_elec_deg;
		float measured_pp;

		if(motor > 2) motor = 0;
		sens_idx = motor_sensor_map[motor];

		printf("\r\n============================================\r\n");
		printf("  MOTOR %d POLE PAIRS MEASUREMENT (PP)\r\n", motor + 1);
		printf("============================================\r\n");

		// 1. 使能当前被测电机
		set_motor_enable(0, 0);
		set_motor_enable(motor + 1, 1);
		delay_ms(50);

		// 2. 定位到电角度 0 弧度 (吸合 1.5 秒确保完全静止稳定)
		printf("1. Aligning rotor to electrical zero (U=%.1fV)...\r\n", test_voltage);
		setPhaseVoltage(test_voltage, 0, _3PI_2, motor);
		delay_ms(1500);

		// 3. 读取初始机械角度
		if(sens_idx == 0) i2c_mt6701_1_get_angle(&raw_init, &deg_init);
		else if(sens_idx == 1) i2c_mt6701_2_get_angle(&raw_init, &deg_init);
		else if(sens_idx == 2) i2c_mt6701_3_get_angle(&raw_init, &deg_init);
		printf("   Initial Mechanical Angle: %.2f deg (Raw: %d)\r\n", deg_init, raw_init);

		// 4. 以极微小步进旋转整整 11 个电周期
		el_step = (float)total_cycles * _2PI / (float)total_steps;
		cur_el = _3PI_2;
		prev_deg = deg_init;
		total_mech_travel = 0.0f;

		printf("2. Rotating stator field smoothly for %d electrical cycles (%d*360 deg)...\r\n", total_cycles, total_cycles);
		for(s = 0; s < total_steps; s++)
		{
			cur_el += el_step;
			setPhaseVoltage(test_voltage, 0, cur_el, motor);
			delay_us(1500);

			if((s % 50) == 0)
			{
				int16_t r_now = 0;
				float d_now = 0.0f;
				if(sens_idx == 0) i2c_mt6701_1_get_angle(&r_now, &d_now);
				else if(sens_idx == 1) i2c_mt6701_2_get_angle(&r_now, &d_now);
				else if(sens_idx == 2) i2c_mt6701_3_get_angle(&r_now, &d_now);

				float diff = d_now - prev_deg;
				if(diff > 180.0f) diff -= 360.0f;
				else if(diff < -180.0f) diff += 360.0f;
				total_mech_travel += diff;
				prev_deg = d_now;
			}
		}

		delay_ms(500);
		{
			int16_t raw_end = 0;
			float deg_end = 0.0f;
			if(sens_idx == 0) i2c_mt6701_1_get_angle(&raw_end, &deg_end);
			else if(sens_idx == 1) i2c_mt6701_2_get_angle(&raw_end, &deg_end);
			else if(sens_idx == 2) i2c_mt6701_3_get_angle(&raw_end, &deg_end);

			float diff = deg_end - prev_deg;
			if(diff > 180.0f) diff -= 360.0f;
			else if(diff < -180.0f) diff += 360.0f;
			total_mech_travel += diff;
		}

		// 5. 卸载输出
		setPhaseVoltage(0, 0, 0, motor);
		set_motor_enable(motor + 1, 0);

		abs_travel = fabsf(total_mech_travel);
		total_elec_deg = (float)total_cycles * 360.0f;
		measured_pp = (abs_travel > 1.0f) ? (total_elec_deg / abs_travel) : 0.0f;

		printf("3. Measurement Finished:\r\n");
		printf("   Total Electrical Rotation: %.1f deg (%d full cycles)\r\n", total_elec_deg, total_cycles);
		printf("   Total Mechanical Rotation: %.2f deg\r\n", total_mech_travel);
		printf("   Calculated Pole Pairs:     %.3f (Closest Integer: %d)\r\n", measured_pp, (int)(measured_pp + 0.5f));
		if(fabsf(measured_pp - 7.0f) < 0.8f) {
			printf("   >> CONCLUSION: Motor has 7 POLE PAIRS (14 magnets) <<\r\n");
		} else if(fabsf(measured_pp - 11.0f) < 0.8f) {
			printf("   >> CONCLUSION: Motor has 11 POLE PAIRS (22 magnets) <<\r\n");
		} else {
			printf("   >> CONCLUSION: Motor has %d POLE PAIRS <<\r\n", (int)(measured_pp + 0.5f));
		}
		printf("============================================\r\n\r\n");
		systick_CountMode();
		printf("OK PP\r\n");
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
