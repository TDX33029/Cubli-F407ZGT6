
#include "timer.h"

/***************************************************************************/
/* TIM1已由CubeMX的MX_TIM1_Init()初始化 (Core/Src/tim.c)
 * TIM1: PE9=CH1, PE11=CH2, PE13=CH3, 25KHz, 中心对齐, deadtime=60ns
 * 这里只保留TIM6初始化
 */

/***************************************************************************/
/* TIM6 1ms interrupt
 * TIM6 clock = APB1 timer clock = 84MHz
 * Prescaler = 84-1 -> 1MHz, Period = 1000-1 -> 1ms
 *
 * 使用寄存器级操作 (不依赖 HAL TIM 驱动)
 */
void TIM6_1ms_Init(void)
{
	__HAL_RCC_TIM6_CLK_ENABLE();

	/* Enable TIM6 interrupt in NVIC */
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	/* 关闭定时器 (配置期间) */
	TIM6->CR1 = 0;

	/* 预分频器: 84-1 -> 1MHz (84MHz / 84 = 1MHz) */
	TIM6->PSC = 84 - 1;
	/* 自动重装载: 1000-1 -> 1ms */
	TIM6->ARR = 1000 - 1;

	/* 使能更新中断 */
	TIM6->DIER = TIM_DIER_UIE;

	/* 生成更新事件 */
	TIM6->EGR |= TIM_EGR_UG;

	/* 使能定时器 */
	TIM6->CR1 |= TIM_CR1_CEN;
}
/***************************************************************************/

/***************************************************************************/
