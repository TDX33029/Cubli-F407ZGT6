
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
uint8_t motor_sensor_map[3] = {0, 1, 2}; // 默认映射表：驱动0(TIM1)->I2C1(MT6701-1), 驱动1(TIM3)->I2C2(MT6701-2), 驱动2(TIM4)->I2C3(MT6701-3)

/* 编码器跟踪状态内部结构 */
float angle_prev[3] = {0.0f, 0.0f, 0.0f};           // 当前单圈机械角度 [0, 2PI)
static unsigned long angle_prev_ts[3] = {0, 0, 0};   // 上一次角度采样时间戳
static float angle_vel_prev[3] = {0.0f, 0.0f, 0.0f}; // 上一次计算速度时的单圈角度 (严格时间对齐)
static unsigned long vel_prev_ts[3] = {0, 0, 0};     // 上一次计算速度时的时间戳

PIDController_t pid_velocity[3];
LowPassFilter_t lpf_velocity[3];

/******************************************************************************/
/* 初始化速度环 PID 和低通滤波器参数 */
void SimpleFOC_PID_Init(void)
{
	int i;
	for(i = 0; i < 3; i++)
	{
		// 速度环 PID 参数：配合摩擦前馈，比例适度，积分稳健消除残差
		pid_velocity[i].P = 0.25f;
		pid_velocity[i].I = 0.80f;
		pid_velocity[i].D = 0.000f;          // 速度环严格置零 D 项
		pid_velocity[i].output_ramp = 150.0f; // 150 V/s 充沛动态响应
		pid_velocity[i].limit = 5.0f;         // 默认与 voltage_limit (5.0V) 对齐
		pid_velocity[i].error_prev = 0.0f;
		pid_velocity[i].output_prev = 0.0f;
		pid_velocity[i].integral_prev = 0.0f;
		pid_velocity[i].timestamp_prev = 0;

		// 速度测量低通滤波器 (时间常数 80ms，完美滤除 30 度定子齿槽谐波与磁编微小噪声)
		lpf_velocity[i].Tf = 0.080f;
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
	uint32_t now_us = micros();
	uint32_t d_us = 0;
	float Ts = 0.0f;
	float raw_vel = 0.0f;

	if(motor < 0 || motor > 2) return;
	uint8_t sens_idx = motor_sensor_map[motor];

	// 1. 读取映射对应的 I2C 通道编码器
	if(sens_idx == 0) res = i2c_mt6701_1_get_angle(&raw, &deg);
	else if(sens_idx == 1) res = i2c_mt6701_2_get_angle(&raw, &deg);
	else if(sens_idx == 2) res = i2c_mt6701_3_get_angle(&raw, &deg);

	if(res != 0) return; // 读取失败则沿用上次值

	// 转换到单圈绝对弧度 [0, 2PI)
	rad = (float)raw * _2PI / 16384.0f;

	// 2. 初始帧采样
	if(angle_prev_ts[motor] == 0)
	{
		angle_prev[motor] = rad;
		angle_vel_prev[motor] = rad;
		angle_prev_ts[motor] = now_us;
		vel_prev_ts[motor] = now_us;
		shaft_angle[motor] = rad;
		shaft_velocity[motor] = 0.0f;
		return;
	}

	// 3. 经典圆周跨界最短角位移计算 (0 <-> 2PI 跨越保护)
	d_angle = rad - angle_prev[motor];
	if (d_angle > _PI) d_angle -= _2PI;
	else if (d_angle < -_PI) d_angle += _2PI;

	angle_prev[motor] = rad;
	angle_prev_ts[motor] = now_us;

	// 4. 连续多圈绝对机械角位移平滑累加
	shaft_angle[motor] += d_angle;

		// 5. 严格同步的速度差分与平滑滤波 (定频 12ms 速度计算窗口: 12000us，提升低速信噪比)
		d_us = now_us - (uint32_t)vel_prev_ts[motor];

		if(d_us >= 200000)
		{
			// 超时停顿重置：同步基准点，消除长停顿跨越，等待下一个 12ms 计算
			angle_vel_prev[motor] = rad;
			vel_prev_ts[motor] = now_us;
		}
		else if(d_us >= 12000) // 12ms ~ 200ms 计算区间 (在 0.5 rad/s 下每个采样包含约 16 个 LSB，信噪比更高)
		{
			Ts = (float)d_us * 1e-6f;

			// 计算自上一次 vel_prev 以来真实转过的全部角位移
			float d_vel = rad - angle_vel_prev[motor];
			if (d_vel > _PI) d_vel -= _2PI;
			else if (d_vel < -_PI) d_vel += _2PI;

			// 速度带旋向归一化：保证无论编码器正装/反装，电机正向转动时反馈速度恒为正
			raw_vel = (float)sensor_direction[motor] * (d_vel / Ts);

			angle_vel_prev[motor] = rad;
			vel_prev_ts[motor] = now_us;

			// 极值限幅保护
			if(raw_vel > 100.0f) raw_vel = 100.0f;
			else if(raw_vel < -100.0f) raw_vel = -100.0f;

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
/* 计算电机当前转子电角度 (考虑旋转方向和对齐零电角)
 * 直接使用当前单圈高精度机械弧度 angle_prev[motor]，无多圈累积误差与跨圈跳变
 */
float electricalAngle(int motor)
{
	if(motor < 0 || motor > 2) return 0.0f;
	return _normalizeAngle((float)(sensor_direction[motor]) * (angle_prev[motor] * (float)pole_pairs) - zero_electric_angle[motor]);
}
/******************************************************************************/


