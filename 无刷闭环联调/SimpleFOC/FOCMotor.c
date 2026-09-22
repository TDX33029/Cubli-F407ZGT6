
#include "MyProject.h"

/******************************************************************************/
float shaft_angle[3] = {0.0f, 0.0f, 0.0f};      //!< current motor continuous angle (rad)
float electrical_angle[3] = {0.0f, 0.0f, 0.0f};
float shaft_velocity[3] = {0.0f, 0.0f, 0.0f};   //!< measured filtered velocity (rad/s)
float current_sp = 0.0f;
float shaft_velocity_sp[3] = {0.0f, 0.0f, 0.0f};
float shaft_angle_sp[3] = {0.0f, 0.0f, 0.0f};
DQVoltage_s voltage[3];
DQCurrent_s current;

TorqueControlType torque_controller = Type_voltage;
MotionControlType controller = Type_velocity; // 默认启动速度闭环控制模式

float sensor_offset[3] = {0.0f, 0.0f, 0.0f};
float zero_electric_angle[3] = {0.0f, 0.0f, 0.0f};
int   sensor_direction[3] = {1, 1, 1}; // 默认 CW
uint8_t motor_aligned[3] = {0, 0, 0};

/* 编码器跟踪状态内部结构 */
static float angle_prev[3] = {0.0f, 0.0f, 0.0f};     // 上一次单圈角度 [0, 2PI)
static long  full_rotations[3] = {0, 0, 0};          // 跨圈整周计数
static unsigned long angle_prev_ts[3] = {0, 0, 0};   // 上一次角度计算时间戳
static unsigned long vel_prev_ts[3] = {0, 0, 0};     // 上一次速度计算时间戳
static float shaft_angle_prev[3] = {0.0f, 0.0f, 0.0f}; // 上一次多圈角度

PIDController_t pid_velocity[3];
LowPassFilter_t lpf_velocity[3];

/******************************************************************************/
/* 初始化速度环 PID 和低通滤波器参数 */
void SimpleFOC_PID_Init(void)
{
	int i;
	for(i = 0; i < 3; i++)
	{
		// 速度环 PID 默认参数 (适合三轴无刷动量轮)
		pid_velocity[i].P = 0.20f;
		pid_velocity[i].I = 1.20f;
		pid_velocity[i].D = 0.001f;
		pid_velocity[i].output_ramp = 100.0f; // 100 V/s
		pid_velocity[i].limit = 2.5f;         // 默认与 voltage_limit 对齐
		pid_velocity[i].error_prev = 0.0f;
		pid_velocity[i].output_prev = 0.0f;
		pid_velocity[i].integral_prev = 0.0f;
		pid_velocity[i].timestamp_prev = 0;

		// 速度测量低通滤波器 (时间常数 15ms，抑制 14 位磁编阶跃噪声)
		lpf_velocity[i].Tf = 0.015f;
		lpf_velocity[i].y_prev = 0.0f;
		lpf_velocity[i].timestamp_prev = 0;
	}
}

/******************************************************************************/
/* 从 MT6701 读取并更新多圈角度与滤波后速度 */
void updateSensor(int motor)
{
	int16_t raw = 0;
	float deg = 0.0f;
	uint8_t res = 1;
	float rad = 0.0f;
	float d_angle = 0.0f;
	unsigned long now_us = SysTick->VAL;
	float Ts = 0.0f;
	float raw_vel = 0.0f;

	if(motor < 0 || motor > 2) return;

	// 1. 读取对应 I2C 通道编码器
	if(motor == 0) res = i2c_mt6701_1_get_angle(&raw, &deg);
	else if(motor == 1) res = i2c_mt6701_2_get_angle(&raw, &deg);
	else if(motor == 2) res = i2c_mt6701_3_get_angle(&raw, &deg);

	if(res != 0) return; // 读取失败则沿用上次值

	// 转换到 [0, 2PI)
	rad = (float)raw * _2PI / 16384.0f;

	// 2. 跟踪整圈溢出 (处理 0 <-> 2PI 跨越)
	if(angle_prev_ts[motor] != 0)
	{
		d_angle = rad - angle_prev[motor];
		if(fabsf(d_angle) > (0.8f * _2PI))
		{
			full_rotations[motor] += (d_angle > 0) ? -1 : 1;
		}
	}
	angle_prev[motor] = rad;
	angle_prev_ts[motor] = now_us;

	// 3. 计算连续多圈绝对机械角位移 (rad)
	shaft_angle[motor] = (float)full_rotations[motor] * _2PI + rad;

	// 4. 计算实时角速度并滤波
	if(vel_prev_ts[motor] == 0)
	{
		vel_prev_ts[motor] = now_us;
		shaft_angle_prev[motor] = shaft_angle[motor];
		return;
	}

	if(now_us < vel_prev_ts[motor])
		Ts = (float)(vel_prev_ts[motor] - now_us) * 1e-6f / 21.0f;
	else
		Ts = (float)(0xFFFFFF - now_us + vel_prev_ts[motor]) * 1e-6f / 21.0f;

	if(Ts > 0.0005f && Ts < 0.2f)
	{
		raw_vel = (shaft_angle[motor] - shaft_angle_prev[motor]) / Ts;
		vel_prev_ts[motor] = now_us;
		shaft_angle_prev[motor] = shaft_angle[motor];

		// 一阶低通滤波
		shaft_velocity[motor] = LPF_operator(&lpf_velocity[motor], raw_vel);
	}
}

/******************************************************************************/
float shaftAngle(int motor)
{
	if(motor >= 0 && motor < 3) return shaft_angle[motor];
	return 0.0f;
}

/******************************************************************************/
float shaftVelocity(int motor)
{
	if(motor >= 0 && motor < 3) return shaft_velocity[motor];
	return 0.0f;
}

/******************************************************************************/
/* 计算电机当前转子电角度 (考虑旋转方向和对齐零电角) */
float electricalAngle(int motor)
{
	if(motor < 0 || motor > 2) return 0.0f;
	return _normalizeAngle((float)(sensor_direction[motor]) * (shaft_angle[motor] * (float)pole_pairs) - zero_electric_angle[motor]);
}
/******************************************************************************/


