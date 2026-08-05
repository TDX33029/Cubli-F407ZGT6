
#include "MyProject.h"


/************************************************
main中调用的接口函数都在当前文件中
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/119220638
创建日期：20210801
作    者：loop222 @郑州
  F4移植: 适配168MHz, TIM1 PWM (PE9/PE11/PE13)
************************************************/
/******************************************************************************/
float voltage_power_supply;
float voltage_limit;
int  pole_pairs;
unsigned long open_loop_timestamp[3];
float velocity_limit;
/******************************************************************************/
float velocityOpenloop(float target_velocity, int motor);
float angleOpenloop(float target_angle, int motor);
/******************************************************************************/
void move(float new_target, int motor)
{
	switch(controller)
	{
		case Type_velocity_openloop:
			// velocity control in open loop
      shaft_velocity_sp[motor] = new_target;
      voltage[motor].q = velocityOpenloop(shaft_velocity_sp[motor], motor); // returns the voltage that is set to the motor
      voltage[motor].d = 0;
			break;
		case Type_angle_openloop:
			// angle control in open loop
      shaft_angle_sp[motor] = new_target;
      voltage[motor].q = angleOpenloop(shaft_angle_sp[motor], motor); // returns the voltage that is set to the motor
      voltage[motor].d = 0;
			break;
		default:
			break;
	}
}
/******************************************************************************/
void setPhaseVoltage(float Uq, float Ud, float angle_el, int motor)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;

	if(Ud) // only if Ud and Uq set
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2f(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}

	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*_sin(angle_el - (sector-1.0f)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;

	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}

	switch(motor)
	{
		case 0:  /* M1: TIM1 通道1~3 (PE9, PE11, PE13) */
			TIM1->CCR1 = (uint32_t)(Ta * (PWM_Period_TIM1 + 1));
			TIM1->CCR2 = (uint32_t)(Tb * (PWM_Period_TIM1 + 1));
			TIM1->CCR3 = (uint32_t)(Tc * (PWM_Period_TIM1 + 1));
			break;
		case 1:  /* M2: TIM3 通道1~3 (PA6, PA7, PB0) */
			TIM3->CCR1 = (uint32_t)(Ta * (PWM_Period_APB1 + 1));
			TIM3->CCR2 = (uint32_t)(Tb * (PWM_Period_APB1 + 1));
			TIM3->CCR3 = (uint32_t)(Tc * (PWM_Period_APB1 + 1));
			break;
		case 2:  /* M3: TIM4 通道1~3 (PD12, PD13, PD14) */
			TIM4->CCR1 = (uint32_t)(Ta * (PWM_Period_APB1 + 1));
			TIM4->CCR2 = (uint32_t)(Tb * (PWM_Period_APB1 + 1));
			TIM4->CCR3 = (uint32_t)(Tc * (PWM_Period_APB1 + 1));
			break;
	}
}
/******************************************************************************/
/* 使用SysTick获取微秒级时间戳
 * HCLK=168MHz, SysTick时钟 = HCLK/8 = 21MHz
 * 计数器递减频率 = 21MHz
 * 因此1个计数值 = 1/21 us  (但计算时取整)
 * 本函数使用 SysTick->VAL 获取当前计数值
 * 配合 0xFFFFFF 重装载值（24位计数器）
 */
static unsigned long _micros(void)
{
	/* SysTick已配置为24位递减计数模式(重装载0xFFFFFF)在main中初始化 */
	return SysTick->VAL;
}

/******************************************************************************/
float velocityOpenloop(float target_velocity, int motor)
{
	unsigned long now_us;
	float Ts,Uq;

	now_us = _micros();
	if(now_us < open_loop_timestamp[motor])
		Ts = (float)(open_loop_timestamp[motor] - now_us) * 1e-6f / 21.0f;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp[motor]) * 1e-6f / 21.0f;
	open_loop_timestamp[motor]=now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

	// calculate the necessary angle to achieve target velocity
  shaft_angle[motor] = _normalizeAngle(shaft_angle[motor] + target_velocity*Ts);

	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle[motor], pole_pairs), motor);

	return Uq;
}
/******************************************************************************/
float angleOpenloop(float target_angle, int motor)
{
	unsigned long now_us;
	float Ts,Uq;

	now_us = _micros();
	if(now_us < open_loop_timestamp[motor])
		Ts = (float)(open_loop_timestamp[motor] - now_us) * 1e-6f / 21.0f;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp[motor]) * 1e-6f / 21.0f;
	open_loop_timestamp[motor] = now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

	// calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(fabsf( target_angle - shaft_angle[motor] ) > velocity_limit*Ts)
	{
    shaft_angle[motor] += _sign(target_angle - shaft_angle[motor]) * velocity_limit * Ts;
  }
	else
	{
    shaft_angle[motor] = target_angle;
  }

	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle[motor], pole_pairs), motor);

  return Uq;
}
/******************************************************************************/




