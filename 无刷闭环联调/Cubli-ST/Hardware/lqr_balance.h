/**
  ******************************************************************************
  * @file           : lqr_balance.h
  * @brief          : Cubli 反作用飞轮动量轮平衡控制算法 (基于 ETH Zurich LQR 模型)
  *                   支持:
  *                   1. 6轴 IMU 卡尔曼滤波与互补滤波姿态估计 (倾角与角速度解算)
  *                   2. 1D 边沿平衡 (Edge Balance) 4态 LQR 控制器
  *                   3. 3D 顶点平衡 (Corner Balance) 解耦坐标映射 LQR 控制器
  *                   4. 跌倒安全保护 (Fall Detection & Auto Safe Lock)
  *                   5. 在线参数调节与自动零位/偏置标定
  ******************************************************************************
  */

#ifndef __LQR_BALANCE_H__
#define __LQR_BALANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef _PI
#define _PI        3.141592653589793f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (_PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / _PI)
#endif

/* ========================================================================== */
/* 平衡状态机枚举                                                             */
/* ========================================================================== */
typedef enum {
    BAL_STATE_DISABLED     = 0,  // 平衡失能待机
    BAL_STATE_CALIBRATING  = 1,  // 传感器静态零偏校准中
    BAL_STATE_STANDBY      = 2,  // 就绪，等待进入平衡捕捉窗口
    BAL_STATE_BALANCING    = 3,  // LQR 闭环平衡控制中
    BAL_STATE_FALL_PROTECT = 4   // 倾角超限跌倒保护 (电机安全切断)
} BalanceState_e;

/* 平衡模式选择 */
typedef enum {
    BAL_MODE_EDGE_M1 = 0,  // 单边沿平衡: M1 (DRV8313 #1, 绕 X/Pitch 轴)
    BAL_MODE_EDGE_M2 = 1,  // 单边沿平衡: M2 (DRV8313 #2, 绕 Y/Roll 轴)
    BAL_MODE_EDGE_M3 = 2,  // 单边沿平衡: M3 (DRV8313 #3, 绕 Z 轴)
    BAL_MODE_CORNER  = 3   // 顶点平衡: M1/M2/M3 三轴解耦联合平衡
} BalanceMode_e;

/* ========================================================================== */
/* LQR 控制器增益参数结构体 (电压域相电压控制)                                 */
/* 倾角通道 (随 sensor_direction 翻转): u_ang = K_theta*(theta-theta_0) + K_dtheta*dtheta */
/* 动量轮退饱和通道 (传感坐标系, 固定负反馈): u = u_ang - K_w*w_wheel - K_i*∫w_wheel*dt    */
/* ========================================================================== */
typedef struct {
    float K_theta;     // 倾角比例刚度增益 (V/rad)
    float K_dtheta;    // 倾角角速度微分阻尼增益 (V/(rad/s))
    float K_w;         // 动量轮退饱和速度反馈增益 (V/(rad/s)), 作用于 -w_wheel
    float K_i;         // 动量轮稳态转速积分衰减增益 (V/(rad)), 作用于 -∫w_wheel*dt
    float theta_0;     // 机械平衡点倾角零位偏置 (rad)
    float max_u;       // 最大允许相电压输出限幅 (V)
} LQR_Gains_t;

/* ========================================================================== */
/* 姿态估计与滤波状态结构体                                                   */
/* ========================================================================== */
typedef struct {
    float pitch;       // 绕 X 轴倾角 (rad)
    float roll;        // 绕 Y 轴倾角 (rad)
    float yaw;         // 绕 Z 轴角 (rad, 积分相对量)
    float pitch_deg;   // 绕 X 轴倾角 (度)
    float roll_deg;    // 绕 Y 轴倾角 (度)
    
    float gyro_rad[3]; // 机体滤波后三轴角速度 (rad/s) [0]=X, [1]=Y, [2]=Z
    float accel_filt[3];// 机体滤波后三轴加速度 (g)
    
    float gyro_bias[3];// 陀螺仪零偏补偿 (rad/s)
    bool  is_calibrated;
} Attitude_t;

/* ========================================================================== */
/* Cubli 平衡系统主控制结构体                                                 */
/* ========================================================================== */
typedef struct {
    BalanceState_e state;
    BalanceMode_e  mode;
    
    LQR_Gains_t gains[3];       // 分别对应 M1, M2, M3 的 LQR 增益
    LQR_Gains_t corner_gains[2];// 顶点平衡: [0]=虚拟X轴, [1]=虚拟Y轴
    float K_yaw_damping;        // 顶点平衡偏航阻尼增益
    
    Attitude_t attitude;        // 当前姿态
    
    float wheel_int[3];         // 动量轮转速积分项 (rad)
    float control_u[3];         // 当前三电机力矩/相电压控制输出 (V)
    
    float max_angle_trip;       // 跌倒保护触发倾角阈值 (rad, 默认 ~15°)
    float capture_angle;        // 平衡捕获允许接入阈值 (rad, 默认 ~5°)
    
    uint32_t last_update_us;    // 控制周期微秒时间戳
    uint32_t calib_samples;     // 校准采样计数
    float gyro_calib_sum[3];    // 校准积分累加和
} Cubli_Balance_t;

/* ========================================================================== */
/* 导出全局变量与接口函数                                                     */
/* ========================================================================== */
extern Cubli_Balance_t cubli_bal;

/* 系统初始化 */
void LQR_Balance_Init(void);

/* IMU 姿态解算核心更新 (建议 200Hz ~ 1000Hz 周期调用) */
void LQR_Attitude_Update(float accel_g[3], float gyro_dps[3], float dt);

/* 平衡控制核心循环 (计算 LQR 状态反馈并直接驱动 FOC 相电压) */
void LQR_Balance_Loop(void);

/* 控制状态切换接口 */
void LQR_Balance_Enable(bool enable);
void LQR_Balance_SetMode(BalanceMode_e mode);
void LQR_Balance_StartCalib(void);

/* 参数在线调节接口 */
void LQR_SetGains(uint8_t motor_idx, float kp, float kd, float kw, float ki);
void LQR_SetZeroAngle(uint8_t motor_idx, float angle_deg);
void LQR_SetMaxVoltage(float max_v);

#ifdef __cplusplus
}
#endif

#endif /* __LQR_BALANCE_H__ */
