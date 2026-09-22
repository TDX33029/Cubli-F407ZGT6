
#include "delay.h"


/******************************************************************************/
/* HCLK=168MHz, SysTick clock = HCLK/8 = 21MHz */

//延时nus
void delay_us(unsigned long nus)
{
	unsigned long temp;

	SysTick->LOAD = nus * 21;   //21 = 针对168MHz (168MHz/8 = 21MHz)
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;   //HCLK/8
	do
	{
		temp = SysTick->CTRL;
	}while((temp & 0x01) && !(temp & (1 << 16)));

	SysTick->CTRL = 0;
	SysTick->VAL = 0;
}
/******************************************************************************/
//延时nms
//最大延时时间=0xFFFFFF/21MHz=798ms
void delay_ms(unsigned short nms)
{
	unsigned long temp;

	SysTick->LOAD = (uint32_t)nms * 21000;   //21000 = 针对168MHz
	SysTick->VAL = 0;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //HCLK/8
	do
	{
		temp = SysTick->CTRL;
	}while((temp & 0x01) && !(temp & (1 << 16)));

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0;
}
/******************************************************************************/
//0xFFFFFF到0循环计数 (用于velocityOpenloop时间戳)
void systick_CountMode(void)
{
	SysTick->LOAD = 0xFFFFFF - 1;      //set reload register
	SysTick->VAL  = 0;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk; //Enable SysTick Timer, HCLK/8
}
/******************************************************************************/
