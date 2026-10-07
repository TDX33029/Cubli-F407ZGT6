/**
  ******************************************************************************
  * @file           : enc_quad.c
  * @brief          : MT6701 编码器双通道读取实现 (I2C 绝对式 / 定时器硬件正交解码)
  ******************************************************************************
  */

#include "enc_quad.h"
#include "i2c.h"

/* ========================================================================== */
/* 原方法: 三路 I2C MT6701 绝对角度读取 (始终编译保留, 不受宏影响)              */
/* ========================================================================== */
uint8_t ENC_I2C_GetAngle(uint8_t idx, int16_t *raw, float *deg)
{
	if (idx == 0) return i2c_mt6701_1_get_angle(raw, deg);
	if (idx == 1) return i2c_mt6701_2_get_angle(raw, deg);
	if (idx == 2) return i2c_mt6701_3_get_angle(raw, deg);
	return 1;
}

/* ========================================================================== */
/* 新方法: F407 三定时器硬件正交解码 (TIM2=M1, TIM5=M2, TIM8=M3)               */
/* 寄存器级初始化, 不依赖 CubeMX 生成的 TIM 句柄                               */
/* ========================================================================== */
void ENC_Quad_Init(void)
{
	GPIO_InitTypeDef gpio_init = {0};

	/* ---------- TIM2: M1 编码器 PA5=CH1(A), PB3=CH2(B), AF1 ---------- */
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

	gpio_init.Pin = GPIO_PIN_5;             /* PA5 -> TIM2_CH1 (A相) */
	gpio_init.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &gpio_init);
	gpio_init.Pin = GPIO_PIN_3;             /* PB3 -> TIM2_CH2 (B相, 需脱离JTAG AF0) */
	gpio_init.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &gpio_init);

	TIM2->CR1 = 0;
	TIM2->SMCR = (3U << TIM_SMCR_SMS_Pos);          /* 编码器模式3: TI1&TI2 双边沿 x4 计数 */
	TIM2->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos)        /* IC1 -> TI1 */
				| (1U << TIM_CCMR1_CC2S_Pos)        /* IC2 -> TI2 */
				| (3U << TIM_CCMR1_IC1F_Pos)        /* N=8 采样滤波, 抑制毛刺 */
				| (3U << TIM_CCMR1_IC2F_Pos);
	TIM2->CCER = 0;                                  /* TI1FP1/TI2FP2 均不反相 */
	TIM2->ARR = 0xFFFF;
	TIM2->CNT = 0;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->CR1 = TIM_CR1_CEN;

	/* ---------- TIM5: M2 编码器 PA0=CH1(A), PA1=CH2(B), AF2 ---------- */
	__HAL_RCC_TIM5_CLK_ENABLE();

	gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	gpio_init.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	TIM5->CR1 = 0;
	TIM5->SMCR = (3U << TIM_SMCR_SMS_Pos);
	TIM5->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos)
				| (1U << TIM_CCMR1_CC2S_Pos)
				| (3U << TIM_CCMR1_IC1F_Pos)
				| (3U << TIM_CCMR1_IC2F_Pos);
	TIM5->CCER = 0;
	TIM5->ARR = 0xFFFF;
	TIM5->CNT = 0;
	TIM5->EGR = TIM_EGR_UG;
	TIM5->CR1 = TIM_CR1_CEN;

	/* ---------- TIM8: M3 编码器 PC6=CH1(A), PC7=CH2(B), AF3 ---------- */
	__HAL_RCC_TIM8_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	gpio_init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio_init.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOC, &gpio_init);

	TIM8->CR1 = 0;
	TIM8->SMCR = (3U << TIM_SMCR_SMS_Pos);
	TIM8->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos)
				| (1U << TIM_CCMR1_CC2S_Pos)
				| (3U << TIM_CCMR1_IC1F_Pos)
				| (3U << TIM_CCMR1_IC2F_Pos);
	TIM8->CCER = 0;
	TIM8->ARR = 0xFFFF;
	TIM8->CNT = 0;
	TIM8->EGR = TIM_EGR_UG;
	TIM8->CR1 = TIM_CR1_CEN;
}

uint8_t ENC_Quad_GetAngle(uint8_t idx, int16_t *raw, float *deg)
{
	uint32_t cnt;
	TIM_TypeDef *tim;

	if (idx == 0) tim = TIM2;
	else if (idx == 1) tim = TIM5;
	else if (idx == 2) tim = TIM8;
	else return 1;

	cnt = tim->CNT & 0xFFFFU;

	*raw = (int16_t)(cnt % ENC_QUAD_COUNTS_PER_REV);
	*deg = (float)(cnt % ENC_QUAD_COUNTS_PER_REV) * 360.0f / (float)ENC_QUAD_COUNTS_PER_REV;
	return 0;
}

/* ========================================================================== */
/* 统一路由: 依据编译期宏决定使用 I2C 绝对式还是定时器正交解码                  */
/* ========================================================================== */
uint8_t ENC_GetAngle(uint8_t idx, int16_t *raw, float *deg)
{
#if ENC_USE_HW_QUAD
	return ENC_Quad_GetAngle(idx, raw, deg);
#else
	return ENC_I2C_GetAngle(idx, raw, deg);
#endif
}
