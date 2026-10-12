
#include "MyProject.h"

extern void set_motor_enable(uint8_t motor, uint8_t en);
extern uint8_t Control_Aborted(void);
extern void Control_RequestStop(uint8_t reason);

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
float alignment_travel_deg[3]={0,0,0};
float voltage_power_supply;
float voltage_limit;
int  pole_pairs;
unsigned long open_loop_timestamp[3];
float velocity_limit;
float voltage_sensor_align = 1.8f; // 校准相电压下调至 1.8V，充足对准同时防止线圈过热
/******************************************************************************/
float velocityOpenloop(float target_velocity, int motor);
float angleOpenloop(float target_angle, int motor);
/******************************************************************************/
/* 对单个电机进行电角度零位对齐与旋转方向辨识 (严格遵循官方 SimpleFOC 闭环校准算法) */
uint8_t Motor_alignSensor(int motor)
{
	int i;
	float start_ang = 0.0f;
	float mid_ang = 0.0f;
	float d_rot = 0.0f;
	float target_align_voltage = voltage_sensor_align;

	if(motor < 0 || motor > 2) return 1;
    motor_aligned[motor]=0;
    alignment_travel_deg[motor]=0;

	// 检查该电机绑定的编码器是否在线，未安装则静默跳过
	uint8_t sens_idx = motor_sensor_map[motor];
	int16_t test_raw = 0;
	float test_deg = 0.0f;
	uint8_t online = 0;
	online = (ENC_GetAngle(sens_idx, &test_raw, &test_deg) == 0);

	if(!online)
	{
		printf("Align M%d: Encoder on I2C%d is not installed (skip).\r\n", motor + 1, sens_idx + 1);
		motor_aligned[motor] = 0;
		return 1;
	}

	set_motor_enable(motor + 1, 1);
	delay_ms(50);

	// 1. 定位到电角度 3PI/2 (定子合成磁矢量在 0 度) 并吸合稳定
	setPhaseVoltage(target_align_voltage, 0, _3PI_2, motor);
	delay_ms(700);

	// 2. 像官方 SimpleFOC 一样缓慢正向扫过 1 个完整电周期 (360度电角)
	updateSensor(motor);
	start_ang = shaftAngle(motor);
    if(!sensor_valid[motor] || Control_Aborted()) goto align_failed;

	for(i = 0; i <= 200; i++)
	{
		float a = _3PI_2 + _2PI * ((float)i / 200.0f);
		setPhaseVoltage(target_align_voltage, 0, a, motor);
		updateSensor(motor);
        if(!sensor_valid[motor] || Control_Aborted()) goto align_failed;
		delay_us(2000);
	}
	updateSensor(motor);
	mid_ang = shaftAngle(motor);

	// 缓慢反向扫过 1 个完整电周期返回
	for(i = 200; i >= 0; i--)
	{
		float a = _3PI_2 + _2PI * ((float)i / 200.0f);
		setPhaseVoltage(target_align_voltage, 0, a, motor);
		updateSensor(motor);
        if(!sensor_valid[motor] || Control_Aborted()) goto align_failed;
		delay_us(2000);
	}
	updateSensor(motor);
	delay_ms(200);

	// 计算正向旋转期间转子实际走过的机械弧度辨识旋向
	d_rot = mid_ang - start_ang;
	if(d_rot > _PI) d_rot -= _2PI;
	else if(d_rot < -_PI) d_rot += _2PI;

	if(!sensor_valid[motor] || Control_Aborted() || fabsf(d_rot)<0.02f ||
       fabsf(fabsf(d_rot)*(float)pole_pairs-_2PI)>_PI) goto align_failed;

	if(d_rot > 0.0f)
	{
		sensor_direction[motor] = 1;  // CW
	}
	else
	{
		sensor_direction[motor] = -1; // CCW
	}

	// 3. 再次静止定位到 3PI/2 准确测量零电角
	setPhaseVoltage(target_align_voltage, 0, _3PI_2, motor);
	delay_ms(700);
	updateSensor(motor);
	if(!sensor_valid[motor] || Control_Aborted()) goto align_failed;
	zero_electric_angle[motor] = 0.0f;
	zero_electric_angle[motor] = electricalAngle(motor);

	setPhaseVoltage(0, 0, 0, motor);
	set_motor_enable(motor + 1, 0); // 对准完成立即关闭通道使能，杜绝等待期间静止发热
	motor_aligned[motor] = 1;
    alignment_travel_deg[motor]=fabsf(d_rot)*180.0f/_PI;

	printf("Align M%d OK (using I2C%d): Dir=%s, Zero_Elec=%.2f rad (%.1f deg)\r\n",
		motor + 1,
		sens_idx + 1,
		sensor_direction[motor] == 1 ? "CW(+1)" : "CCW(-1)",
		zero_electric_angle[motor],
		zero_electric_angle[motor] * 180.0f / _PI);

	return 0;
align_failed:
    set_motor_enable(motor+1,0);
    motor_aligned[motor]=0;
    printf("ERR ALIGN M%d (encoder/movement/abort)\r\n",motor+1);
    return 1;
}

/******************************************************************************/
/* 对所有三轴电机执行一键顺序对齐校准 */
void Motor_alignAll(void)
{
	printf("\r\n=== Starting 3-Axis MT6701 Sensor Align ===\r\n");
	Motor_alignSensor(0);
	Motor_alignSensor(1);
	Motor_alignSensor(2);

	// 复位三轴速度环与闭环同步电角度状态，杜绝切入闭环瞬间产生扰动
	// 校准完毕后，强制切断全部三轴电机驱动硬件使能与相电压输出，处于安全待机状态
	set_motor_enable(0, 0);
	setPhaseVoltage(0.0f, 0.0f, 0.0f, 0);
	setPhaseVoltage(0.0f, 0.0f, 0.0f, 1);
	setPhaseVoltage(0.0f, 0.0f, 0.0f, 2);

	reset_closed_loop_state(-1);

	printf("=== 3-Axis Sensor Align Finished ===\r\n\r\n");
}

/******************************************************************************/
/* FOC 内环：读取传感器并更新电机力矩/相电压换向 */
void loopFOC(int motor)
{
	if(motor < 0 || motor > 2) return;
	// 更新编码器多圈绝对角度与角速度
	updateSensor(motor);
	// 计算转子电角度
	electrical_angle[motor] = electricalAngle(motor);
}

/******************************************************************************/
float open_loop_angle[3] = {0.0f, 0.0f, 0.0f};
static uint32_t velocity_ramp_timestamp[3] = {0, 0, 0};
static uint32_t locked_rotor_since[3] = {0, 0, 0};

static float ramp_velocity_target(float requested, int motor)
{
	uint32_t now_us = micros();
	float Ts = 0.001f;
	float max_delta;
	float delta;

	if(requested != requested) requested = 0.0f;
	requested = _constrain(requested, -velocity_limit, velocity_limit);
	if(velocity_ramp_timestamp[motor] != 0)
	{
		Ts = (float)(now_us - velocity_ramp_timestamp[motor]) * 1e-6f;
		if(Ts <= 0.0f || Ts > 0.02f) Ts = 0.02f;
	}
	velocity_ramp_timestamp[motor] = now_us;

	max_delta = 100.0f * Ts; /* 100 rad/s^2 command slew limit */
	delta = requested - shaft_velocity_sp[motor];
	if(delta > max_delta) delta = max_delta;
	else if(delta < -max_delta) delta = -max_delta;
	shaft_velocity_sp[motor] += delta;
	return shaft_velocity_sp[motor];
}

/******************************************************************************/
void move(float new_target, int motor)
{
	if(motor < 0 || motor > 2) return;
    if((controller==Type_velocity || controller==Type_velocity_openloop) &&
       fabsf(new_target)>0.1f && fabsf(voltage[motor].q)>1.8f &&
       fabsf(shaft_velocity[motor])<0.1f) {
        uint32_t now=micros();
        if(!locked_rotor_since[motor]) locked_rotor_since[motor]=now;
        else if(now-locked_rotor_since[motor]>800000U) {Control_RequestStop(7);return;}
    } else locked_rotor_since[motor]=0;

	switch(controller)
	{
		case Type_velocity:
		{
			// 速度闭环控制模式 (FOC 矢量速度控制: 齿槽摩擦前馈 + 反电势解耦 + 抗饱和高动态 PID)
			loopFOC(motor);
			if(!motor_aligned[motor] || !sensor_valid[motor])
			{
                Control_RequestStop(6);
				pid_velocity[motor].integral_prev = 0.0f;
				pid_velocity[motor].error_prev = 0.0f;
				pid_velocity[motor].output_prev = 0.0f;
				pid_velocity[motor].timestamp_prev = 0;
				voltage[motor].q = 0.0f;
				voltage[motor].d = 0.0f;
				setPhaseVoltage(0.0f, 0.0f, electrical_angle[motor], motor);
				break;
			}
			float ramped_target = ramp_velocity_target(new_target, motor);

			// 仅在目标与实测速度都接近零时断输出，停车阶段保留反向力矩制动。
			if(fabsf(ramped_target) < 0.05f && fabsf(shaft_velocity[motor]) < 0.15f)
			{
				move_timestamp_prev[motor] = micros();
				pid_velocity[motor].integral_prev = 0.0f;
				pid_velocity[motor].error_prev = 0.0f;
				pid_velocity[motor].output_prev = 0.0f;
				voltage[motor].q = 0.0f;
				voltage[motor].d = 0.0f;
				setPhaseVoltage(0.0f, 0.0f, electrical_angle[motor], motor);
				break;
			}

			// 2. 计算控制周期 Ts
			uint32_t now_us = micros();
			float Ts;
			if(move_timestamp_prev[motor] == 0) Ts = 1e-3f;
			else Ts = (float)(now_us - (uint32_t)move_timestamp_prev[motor]) * 1e-6f;
			if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
			move_timestamp_prev[motor] = now_us;

			float speed_err = ramped_target - shaft_velocity[motor];

			// 3. 极低速丝滑前馈 (Coulomb Friction & Detent Bias Feedforward):
			// 克服齿槽转矩死区卡顿，确保正向转动时推进力矩平滑向前，不在零点来回刹车跳变
			float uq_coulomb = 0.45f * tanhf(ramped_target / 0.5f);
			float uq_bemf = 0.045f * ramped_target;
			float uq_ff = _constrain(uq_coulomb + uq_bemf, -voltage_limit, voltage_limit);

			// 持续饱和且误差较大时限幅，直到速度恢复或目标降至低速区。
			static uint32_t stall_timer[3] = {0, 0, 0};
			static uint8_t stall_latched[3] = {0, 0, 0};
			if(fabsf(speed_err) < 3.0f || fabsf(ramped_target) < 1.0f)
			{
				stall_timer[motor] = 0;
				stall_latched[motor] = 0;
			}
			else if(fabsf(voltage[motor].q) >= (voltage_limit - 0.2f))
			{
				if(stall_timer[motor] == 0) stall_timer[motor] = now_us;
				else if((now_us - stall_timer[motor]) > 800000) // 800ms
				{
					stall_latched[motor] = 1;
				}
			}
			else
			{
				stall_timer[motor] = 0;
			}

			float output_limit = (stall_latched[motor] && voltage_limit > 1.8f) ? 1.8f : voltage_limit;
			pid_velocity[motor].limit = output_limit;
			float uq_pid = PID_operator_offset(&pid_velocity[motor], speed_err, uq_ff);
			voltage[motor].q = _constrain(uq_ff + uq_pid, -output_limit, output_limit);
			voltage[motor].d = 0.0f;

			// 驱动相电压输出
			setPhaseVoltage(voltage[motor].q, voltage[motor].d, electrical_angle[motor], motor);
			break;
		}

		case Type_torque:
				// 力矩控制模式 (平衡控制器相电压输出模式)
				loopFOC(motor);
				if(!motor_aligned[motor] || !sensor_valid[motor])
				{
					voltage[motor].q = 0.0f;
					voltage[motor].d = 0.0f;
					setPhaseVoltage(0.0f, 0.0f, electrical_angle[motor], motor);
					break;
				}
				voltage[motor].q = _constrain(new_target, -voltage_limit, voltage_limit);
				voltage[motor].d = 0.0f;
				setPhaseVoltage(voltage[motor].q, voltage[motor].d, electrical_angle[motor], motor);
				break;

			case Type_velocity_openloop:
			// 速度开环模式 (同时读取传感器，保证上位机与遥测实时监测真实机械角度)
			updateSensor(motor);
			voltage[motor].q = velocityOpenloop(ramp_velocity_target(new_target, motor), motor);
			voltage[motor].d = 0.0f;
			break;

		case Type_angle_openloop:
			updateSensor(motor);
			shaft_angle_sp[motor] = new_target;
			voltage[motor].q = angleOpenloop(shaft_angle_sp[motor], motor);
			voltage[motor].d = 0.0f;
			break;

		default:
			break;
	}
}
/******************************************************************************/
/* 官方 SimpleFOC 标准 SpaceVectorPWM (Midpoint Clamp 鞍形波空间矢量调制)
 * 无扇区边界断点，全浮点硬件加速，相电压谐波最低，极其平滑连续
 */
void setPhaseVoltage(float Uq, float Ud, float angle_el, int motor)
{
    if(Control_Aborted() || !isfinite(Uq) || !isfinite(Ud) || !isfinite(angle_el) || voltage_power_supply<=0) {
        if(!Control_Aborted()) Control_RequestStop(6);
        return;
    }
	float _ca = cosf(angle_el);
	float _sa = sinf(angle_el);

	// 1. Inverse Park transform (反 Park 变换)
	float Ualpha = _ca * Ud - _sa * Uq;
	float Ubeta  = _sa * Ud + _ca * Uq;

	// 2. Clarke transform (Clarke 变换)
	float Ua = Ualpha;
	float Ub = -0.5f * Ualpha + 0.8660254f * Ubeta;
	float Uc = -0.5f * Ualpha - 0.8660254f * Ubeta;

	// 3. Space Vector PWM: 中点注入鞍形波 (Midpoint Clamp)
	float Umin = Ua < Ub ? (Ua < Uc ? Ua : Uc) : (Ub < Uc ? Ub : Uc);
	float Umax = Ua > Ub ? (Ua > Uc ? Ua : Uc) : (Ub > Uc ? Ub : Uc);
	float center = (voltage_power_supply * 0.5f) - (Umax + Umin) * 0.5f;

	Ua += center;
	Ub += center;
	Uc += center;

	// 4. 占空比归一化 [0.0, 1.0]
	float Ta = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
	float Tb = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
	float Tc = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

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
/* 使用 TIM5 32位全硬件微秒时间戳 (1MHz) */
static unsigned long _micros(void)
{
	return (unsigned long)micros();
}

/******************************************************************************/
float velocityOpenloop(float target_velocity, int motor)
{
	unsigned long now_us = _micros();
	float Ts;

	if(open_loop_timestamp[motor] == 0)
	{
		Ts = 1e-3f;
	}
	else
	{
		Ts = (float)(now_us - open_loop_timestamp[motor]) * 1e-6f;
	}
	open_loop_timestamp[motor] = now_us;

	if(Ts <= 0.0f || Ts > 0.02f) Ts = 1e-3f;
	if(fabsf(target_velocity) < 0.05f)
	{
		setPhaseVoltage(0.0f, 0.0f, _normalizeAngle(open_loop_angle[motor] * (float)pole_pairs), motor);
		return 0.0f;
	}

	// 专用独立开环角度累加器 (不污染 shaft_angle 多圈传感器真值)
	open_loop_angle[motor] = _normalizeAngle(open_loop_angle[motor] + target_velocity * Ts);

	// 电角度 = 机械角度 * 极对数
	setPhaseVoltage(voltage_limit, 0, _normalizeAngle(open_loop_angle[motor] * (float)pole_pairs), motor);

	return voltage_limit;
}
/******************************************************************************/
float angleOpenloop(float target_angle, int motor)
{
	unsigned long now_us;
	float Ts,Uq;

	now_us = _micros();
	if(open_loop_timestamp[motor] == 0)
	{
		Ts = 1e-3f;
	}
	else
	{
		Ts = (float)(now_us - open_loop_timestamp[motor]) * 1e-6f;
	}
	open_loop_timestamp[motor] = now_us;

	if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

	if(fabsf(target_angle - shaft_angle[motor]) > velocity_limit * Ts)
	{
		shaft_angle[motor] += _sign(target_angle - shaft_angle[motor]) * velocity_limit * Ts;
	}
	else
	{
		shaft_angle[motor] = target_angle;
	}

	Uq = voltage_limit;
	setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle[motor], pole_pairs), motor);

	return Uq;
}
/******************************************************************************/




