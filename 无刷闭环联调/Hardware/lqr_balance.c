/**
  ******************************************************************************
  * @file           : lqr_balance.c
  * @brief          : Cubli 反作用动量轮平衡控制算法实现 (ETH Zurich LQR 模型)
  ******************************************************************************
  */

#include "lqr_balance.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "timer.h"
#include "delay.h"
#include <math.h>
#include <stdio.h>

#ifndef _PI
#define _PI        3.141592653589793f
#endif

#ifndef _2PI
#define _2PI       6.283185307179586f
#endif

#define DEG_TO_RAD (_PI / 180.0f)
#define RAD_TO_DEG (180.0f / _PI)

/* 全局平衡控制单例对象 */
Cubli_Balance_t cubli_bal;

/* 内部一阶卡尔曼滤波器结构体 (用于单个倾角轴的高精度融合) */
typedef struct {
    float angle;      // 估计角度 (rad)
    float bias;       // 估计陀螺仪漂移 (rad/s)
    float P[2][2];    // 估计协方差矩阵
    float Q_angle;    // 过程噪声协方差 (角度)
    float Q_bias;     // 过程噪声协方差 (漂移)
    float R_measure;  // 测量噪声协方差 (加速度计)
} KalmanFilter_t;

static KalmanFilter_t kf_pitch;
static KalmanFilter_t kf_roll;

/* 初始化单个卡尔曼滤波器 */
static void Kalman_Init(KalmanFilter_t *kf)
{
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;
}

/* 卡尔曼滤波更新步: 输入陀螺仪角速度(rad/s)与加速度测量倾角(rad), 返回滤波后的倾角 */
static float Kalman_Update(KalmanFilter_t *kf, float new_angle, float new_rate, float dt)
{
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

    // 1. 状态预测: angle += (rate - bias) * dt
    float rate = new_rate - kf->bias;
    kf->angle += dt * rate;

    // 2. 协方差更新
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 3. 计算卡尔曼增益 K = P * H' / (H * P * H' + R)
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // 4. 测量更新: y = z - H * x
    float y = new_angle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // 5. 更新估计协方差: P = (I - K * H) * P
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/* ========================================================================== */
/* 平衡系统默认参数初始化                                                     */
/* ========================================================================== */
void LQR_Balance_Init(void)
{
    cubli_bal.state = BAL_STATE_DISABLED;
    cubli_bal.mode = BAL_MODE_EDGE_M1; // 默认采用 M1 (DRV8313 #1) 边沿平衡
    cubli_bal.max_angle_trip = 15.0f * DEG_TO_RAD; // 15度触发跌倒保护
    cubli_bal.capture_angle  = 5.0f * DEG_TO_RAD;  // 5度内捕获激活平衡
    cubli_bal.K_yaw_damping  = 0.30f;
    cubli_bal.last_update_us = 0;
    cubli_bal.calib_samples  = 0;

    int i;
    for (i = 0; i < 3; i++)
    {
        /* 1D 边沿平衡经典 LQR 推荐参数 (电压域 12V 供电体系) */
        // u = K_theta * theta + K_dtheta * dtheta + K_w * w_wheel + K_i * int_w
        cubli_bal.gains[i].K_theta  = 32.0f;  // 倾角比例增益 (克服重力失稳力矩)
        cubli_bal.gains[i].K_dtheta = 2.40f;  // 倾角角速度微分阻尼增益 (抑制剧烈振荡)
        cubli_bal.gains[i].K_w      = 0.080f; // 动量轮退饱和速度反馈项 (将飞轮拉回零转速)
        cubli_bal.gains[i].K_i      = 0.008f; // 动量轮速度稳态残差积分消除项
        cubli_bal.gains[i].theta_0  = 0.0f;   // 默认几何垂直为 0
        cubli_bal.gains[i].max_u    = 6.0f;   // 最大限制在 6.0V 相电压内
        
        cubli_bal.wheel_int[i] = 0.0f;
        cubli_bal.control_u[i] = 0.0f;
        cubli_bal.attitude.gyro_bias[i] = 0.0f;
    }

    /* 3D 顶点平衡虚拟轴 LQR 增益初始化 */
    for (i = 0; i < 2; i++)
    {
        cubli_bal.corner_gains[i].K_theta  = 28.0f;
        cubli_bal.corner_gains[i].K_dtheta = 2.20f;
        cubli_bal.corner_gains[i].K_w      = 0.070f;
        cubli_bal.corner_gains[i].K_i      = 0.006f;
        cubli_bal.corner_gains[i].theta_0  = 0.0f;
        cubli_bal.corner_gains[i].max_u    = 6.0f;
    }

    Kalman_Init(&kf_pitch);
    Kalman_Init(&kf_roll);

    cubli_bal.attitude.pitch = 0.0f;
    cubli_bal.attitude.roll = 0.0f;
    cubli_bal.attitude.yaw = 0.0f;
    cubli_bal.attitude.pitch_deg = 0.0f;
    cubli_bal.attitude.roll_deg = 0.0f;
    cubli_bal.attitude.is_calibrated = false;
}

/* ========================================================================== */
/* 触发传感器零偏校准流程                                                     */
/* ========================================================================== */
void LQR_Balance_StartCalib(void)
{
    cubli_bal.state = BAL_STATE_CALIBRATING;
    cubli_bal.calib_samples = 0;
    cubli_bal.gyro_calib_sum[0] = 0.0f;
    cubli_bal.gyro_calib_sum[1] = 0.0f;
    cubli_bal.gyro_calib_sum[2] = 0.0f;
    cubli_bal.attitude.is_calibrated = false;
    printf("[LQR] IMU Zero-bias Calibration started. Please keep Cubli stationary...\r\n");
}

/* ========================================================================== */
/* IMU 姿态解算更新 (加速度计 + 陀螺仪多传感器融合卡尔曼滤波)                   */
/* accel_g: 单位 g, gyro_dps: 单位 °/s                                        */
/* ========================================================================== */
void LQR_Attitude_Update(float accel_g[3], float gyro_dps[3], float dt)
{
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

    // 1. 静止零偏自校准处理 (采集 200 个样本取平均)
    if (cubli_bal.state == BAL_STATE_CALIBRATING)
    {
        cubli_bal.gyro_calib_sum[0] += gyro_dps[0] * DEG_TO_RAD;
        cubli_bal.gyro_calib_sum[1] += gyro_dps[1] * DEG_TO_RAD;
        cubli_bal.gyro_calib_sum[2] += gyro_dps[2] * DEG_TO_RAD;
        cubli_bal.calib_samples++;

        if (cubli_bal.calib_samples >= 200)
        {
            cubli_bal.attitude.gyro_bias[0] = cubli_bal.gyro_calib_sum[0] / 200.0f;
            cubli_bal.attitude.gyro_bias[1] = cubli_bal.gyro_calib_sum[1] / 200.0f;
            cubli_bal.attitude.gyro_bias[2] = cubli_bal.gyro_calib_sum[2] / 200.0f;
            cubli_bal.attitude.is_calibrated = true;
            cubli_bal.state = BAL_STATE_STANDBY;
            printf("[LQR] Calibration Complete! Gyro Bias (rad/s): X=%.4f, Y=%.4f, Z=%.4f\r\n",
                   cubli_bal.attitude.gyro_bias[0],
                   cubli_bal.attitude.gyro_bias[1],
                   cubli_bal.attitude.gyro_bias[2]);
        }
        return;
    }

    // 2. 扣除陀螺仪零偏并转为 rad/s
    cubli_bal.attitude.gyro_rad[0] = (gyro_dps[0] * DEG_TO_RAD) - cubli_bal.attitude.gyro_bias[0];
    cubli_bal.attitude.gyro_rad[1] = (gyro_dps[1] * DEG_TO_RAD) - cubli_bal.attitude.gyro_bias[1];
    cubli_bal.attitude.gyro_rad[2] = (gyro_dps[2] * DEG_TO_RAD) - cubli_bal.attitude.gyro_bias[2];

    // 3. 加速度计一阶低通滤波并计算静态重力倾角
    const float lpf_alpha = 0.2f;
    cubli_bal.attitude.accel_filt[0] += lpf_alpha * (accel_g[0] - cubli_bal.attitude.accel_filt[0]);
    cubli_bal.attitude.accel_filt[1] += lpf_alpha * (accel_g[1] - cubli_bal.attitude.accel_filt[1]);
    cubli_bal.attitude.accel_filt[2] += lpf_alpha * (accel_g[2] - cubli_bal.attitude.accel_filt[2]);

    float ax = cubli_bal.attitude.accel_filt[0];
    float ay = cubli_bal.attitude.accel_filt[1];
    float az = cubli_bal.attitude.accel_filt[2];

    // 计算加速度重力倾角 (在机体坐标系下的倾斜弧度)
    float measured_pitch = atan2f(ay, sqrtf(ax * ax + az * az));
    float measured_roll  = atan2f(-ax, az);

    // 4. 卡尔曼滤波更新估计各轴倾角与角速度
    cubli_bal.attitude.pitch = Kalman_Update(&kf_pitch, measured_pitch, cubli_bal.attitude.gyro_rad[0], dt);
    cubli_bal.attitude.roll  = Kalman_Update(&kf_roll, measured_roll, cubli_bal.attitude.gyro_rad[1], dt);
    cubli_bal.attitude.yaw   += cubli_bal.attitude.gyro_rad[2] * dt;

    cubli_bal.attitude.pitch_deg = cubli_bal.attitude.pitch * RAD_TO_DEG;
    cubli_bal.attitude.roll_deg  = cubli_bal.attitude.roll * RAD_TO_DEG;
}

/* ========================================================================== */
/* 平衡控制核心循环 (由主循环或定时器以 200Hz ~ 400Hz 周期调用)                 */
/* ========================================================================== */
void LQR_Balance_Loop(void)
{
    // 计算执行微秒周期 dt
    uint32_t now_us = micros();
    float dt;
    if (cubli_bal.last_update_us == 0) dt = 0.005f;
    else dt = (float)(now_us - cubli_bal.last_update_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;
    cubli_bal.last_update_us = now_us;

    // 若未使能或正在校准，维持输出为 0
    if (cubli_bal.state == BAL_STATE_DISABLED || cubli_bal.state == BAL_STATE_CALIBRATING)
    {
        cubli_bal.control_u[0] = 0.0f;
        cubli_bal.control_u[1] = 0.0f;
        cubli_bal.control_u[2] = 0.0f;
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* 1. 单边沿平衡模式 (1D Edge Balance: M1, M2 或 M3)                       */
    /* ---------------------------------------------------------------------- */
    if (cubli_bal.mode <= BAL_MODE_EDGE_M3)
    {
        uint8_t m = (uint8_t)cubli_bal.mode; // 对应电机索引 0, 1, 2
        float theta = 0.0f;
        float dtheta = 0.0f;

        // 依据被选定的边沿电机确定所跟踪的机体倾角与角速度
        // 注意: 同一LQR状态对必须是同一坐标的角度与角速度 (P项与D项作用量必须一致)
        if (m == 0) // M1: 绕 Pitch (X) 轴
        {
            theta  = cubli_bal.attitude.pitch;
            dtheta = cubli_bal.attitude.gyro_rad[0];
        }
        else if (m == 1) // M2: 绕 Roll (Y) 轴
        {
            theta  = cubli_bal.attitude.roll;
            dtheta = cubli_bal.attitude.gyro_rad[1];
        }
        else // M3: 绕 Z (Yaw) 轴, 使用积分得到的偏航角与 Z 轴角速度配对 (实验性: yaw 会缓慢漂移)
        {
            theta  = cubli_bal.attitude.yaw;
            dtheta = cubli_bal.attitude.gyro_rad[2];
        }

        // 倾角误差
        float angle_err = theta - cubli_bal.gains[m].theta_0;

        // 跌倒失稳安全检测 (Fall Detection)
        if (fabsf(angle_err) > cubli_bal.max_angle_trip)
        {
            cubli_bal.state = BAL_STATE_FALL_PROTECT;
            cubli_bal.wheel_int[m] = 0.0f;
            cubli_bal.control_u[m] = 0.0f;
            setPhaseVoltage(0.0f, 0.0f, electrical_angle[m], m);
            return;
        }

        // 自从跌倒保护后，如果倾角回到捕获窗口内，自动重新恢复平衡闭环
        if (cubli_bal.state == BAL_STATE_FALL_PROTECT || cubli_bal.state == BAL_STATE_STANDBY)
        {
            if (fabsf(angle_err) < cubli_bal.capture_angle)
            {
                cubli_bal.state = BAL_STATE_BALANCING;
                cubli_bal.wheel_int[m] = 0.0f;
            }
            else
            {
                cubli_bal.control_u[m] = 0.0f;
                setPhaseVoltage(0.0f, 0.0f, electrical_angle[m], m);
                return;
            }
        }

        // 读取对应动量轮的实时机械角速度 (来自 MT6701 磁编码器滤波速度)
        float w_wheel = shaft_velocity[m];

        // 动量轮稳态转速积分更新 (带有抗积分饱和限制)
        cubli_bal.wheel_int[m] += w_wheel * dt;
        if (cubli_bal.wheel_int[m] > 50.0f)  cubli_bal.wheel_int[m] = 50.0f;
        if (cubli_bal.wheel_int[m] < -50.0f) cubli_bal.wheel_int[m] = -50.0f;

        /* ------------------------------------------------------------------ */
        /* 4态LQR控制律 (相电压Uq域) - 符号与坐标系修正:                        */
        /* 1) 机体倾角通道 u_ang = K_theta*err + K_dtheta*dtheta:              */
        /*    该通道的符号建立在"倾角正方向-电压正方向"的机架安装映射上,        */
        /*    需随编码器旋向 (sensor_direction) 整体翻转;                      */
        /* 2) 动量轮退饱和通道: shaft_velocity[] 已是传感坐标系速度,            */
        /*    必须以负号构成负反馈 (w>0 -> 指令反向制动把飞轮拉回零速),          */
        /*    且绝不能再随 sensor_direction 二次翻转。                          */
        /*    原实现 +K_w*w + K_i*int 再整体翻转 = 对飞轮转速正反馈,            */
        /*    会导致飞轮单向飞车直至电压饱和、退饱和机制完全失效。               */
        /* ------------------------------------------------------------------ */
        float u_ang = cubli_bal.gains[m].K_theta  * angle_err
                    + cubli_bal.gains[m].K_dtheta * dtheta;
        if (sensor_direction[m] < 0) u_ang = -u_ang;

        float u = u_ang
                - cubli_bal.gains[m].K_w * w_wheel
                - cubli_bal.gains[m].K_i * cubli_bal.wheel_int[m];

        // 输出电压安全限幅
        float max_u = cubli_bal.gains[m].max_u;
        if (max_u > voltage_limit) max_u = voltage_limit;
        u = _constrain(u, -max_u, max_u);

        cubli_bal.control_u[m] = u;

        // FOC 闭环相电压输出：直接施加到指定电机
        loopFOC(m);
        voltage[m].q = u;
        voltage[m].d = 0.0f;
        setPhaseVoltage(voltage[m].q, voltage[m].d, electrical_angle[m], m);

        // 未参与平衡的其余电机输出保持 0
        int k;
        for (k = 0; k < 3; k++)
        {
            if (k != m)
            {
                cubli_bal.control_u[k] = 0.0f;
                setPhaseVoltage(0.0f, 0.0f, electrical_angle[k], k);
            }
        }
    }
    /* ---------------------------------------------------------------------- */
    /* 2. 3D 顶点平衡模式 (Corner Balance: M1 + M2 + M3 解耦映射)             */
    /* ---------------------------------------------------------------------- */
    else if (cubli_bal.mode == BAL_MODE_CORNER)
    {
        // 顶点平衡: 空间对角线朝上，计算两正交水平倾角误差
        float err_x = cubli_bal.attitude.pitch - cubli_bal.corner_gains[0].theta_0;
        float err_y = cubli_bal.attitude.roll  - cubli_bal.corner_gains[1].theta_0;

        float total_tilt = sqrtf(err_x * err_x + err_y * err_y);

        // 跌倒保护判定
        if (total_tilt > cubli_bal.max_angle_trip)
        {
            cubli_bal.state = BAL_STATE_FALL_PROTECT;
            cubli_bal.wheel_int[0] = 0.0f;
            cubli_bal.wheel_int[1] = 0.0f;
            cubli_bal.wheel_int[2] = 0.0f;
            setPhaseVoltage(0, 0, 0, 0);
            setPhaseVoltage(0, 0, 0, 1);
            setPhaseVoltage(0, 0, 0, 2);
            return;
        }

        if (cubli_bal.state == BAL_STATE_FALL_PROTECT || cubli_bal.state == BAL_STATE_STANDBY)
        {
            if (total_tilt < cubli_bal.capture_angle)
            {
                cubli_bal.state = BAL_STATE_BALANCING;
            }
            else
            {
                setPhaseVoltage(0, 0, 0, 0);
                setPhaseVoltage(0, 0, 0, 1);
                setPhaseVoltage(0, 0, 0, 2);
                return;
            }
        }

        // 3D 几何投影: 将三个电机的飞轮速度转换为虚拟倾角轴速度 (120° 正交分解)
        // w_vx = w2 - 0.5 * (w1 + w3)
        // w_vy = sqrt(3)/2 * (w3 - w1)
        // 并同步累加三轴飞轮转速积分 (供退饱和项使用, 原实现缺失导致顶点模式飞轮转速不受控)
        int k;
        for (k = 0; k < 3; k++)
        {
            cubli_bal.wheel_int[k] += shaft_velocity[k] * dt;
            if (cubli_bal.wheel_int[k] >  50.0f) cubli_bal.wheel_int[k] =  50.0f;
            if (cubli_bal.wheel_int[k] < -50.0f) cubli_bal.wheel_int[k] = -50.0f;
        }

        float w1 = shaft_velocity[0];
        float w2 = shaft_velocity[1];
        float w3 = shaft_velocity[2];

        float w_vx = w2 - 0.5f * (w1 + w3);
        float w_vy = 0.8660254f * (w3 - w1);

        float int_vx = cubli_bal.wheel_int[1] - 0.5f * (cubli_bal.wheel_int[0] + cubli_bal.wheel_int[2]);
        float int_vy = 0.8660254f * (cubli_bal.wheel_int[2] - cubli_bal.wheel_int[0]);

        // 两个虚拟轴分别执行 LQR 计算恢复力矩
        // 倾角通道为机架安装符号 (随硬件标定); 飞轮速度/积分通道取负号负反馈, 与1D边沿模式一致
        float Tx = cubli_bal.corner_gains[0].K_theta  * err_x
                 + cubli_bal.corner_gains[0].K_dtheta * cubli_bal.attitude.gyro_rad[0]
                 - cubli_bal.corner_gains[0].K_w      * w_vx
                 - cubli_bal.corner_gains[0].K_i      * int_vx;

        float Ty = cubli_bal.corner_gains[1].K_theta  * err_y
                 + cubli_bal.corner_gains[1].K_dtheta * cubli_bal.attitude.gyro_rad[1]
                 - cubli_bal.corner_gains[1].K_w      * w_vy
                 - cubli_bal.corner_gains[1].K_i      * int_vy;

        // 偏航轴阻尼力矩
        float Tz = -cubli_bal.K_yaw_damping * cubli_bal.attitude.gyro_rad[2];

        // 120° 逆变换将虚拟力矩映射为三个实际电机相电压
        // u1 = -Ty * cos(30°) - Tx * sin(30°) + 1/3 * Tz
        // u2 = Tx + 1/3 * Tz
        // u3 = -Tx * sin(30°) + Ty * cos(30°) + 1/3 * Tz
        float u1 = -Ty * 0.8660254f - Tx * 0.5f + 0.3333333f * Tz;
        float u2 =  Tx + 0.3333333f * Tz;
        float u3 = -Tx * 0.5f + Ty * 0.8660254f + 0.3333333f * Tz;

        cubli_bal.control_u[0] = _constrain(u1, -voltage_limit, voltage_limit);
        cubli_bal.control_u[1] = _constrain(u2, -voltage_limit, voltage_limit);
        cubli_bal.control_u[2] = _constrain(u3, -voltage_limit, voltage_limit);

        // 驱动三个电机的 FOC 力矩
        for (k = 0; k < 3; k++)
        {
            loopFOC(k);
            voltage[k].q = cubli_bal.control_u[k];
            voltage[k].d = 0.0f;
            setPhaseVoltage(voltage[k].q, voltage[k].d, electrical_angle[k], k);
        }
    }
}

/* ========================================================================== */
/* 平衡系统使能/失能开关                                                       */
/* ========================================================================== */
void LQR_Balance_Enable(bool enable)
{
    if (enable)
    {
        // 若未校准零偏，首先进入校准
        if (!cubli_bal.attitude.is_calibrated)
        {
            LQR_Balance_StartCalib();
        }
        else
        {
            /* 使能后先进入 STANDBY 捕获状态, 由控制循环在倾角进入捕获窗口
             * (capture_angle) 时才真正切入 BALANCING, 避免大倾角下直接输出
             * 满幅矫正电压造成捕获瞬间猛烈跳机 (原实现直接进入 BALANCING)。 */
            cubli_bal.state = BAL_STATE_STANDBY;
            printf("[LQR] Balance Enabled (Mode: %d), waiting capture window...\r\n", cubli_bal.mode);
        }
    }
    else
    {
        cubli_bal.state = BAL_STATE_DISABLED;
        cubli_bal.control_u[0] = 0.0f;
        cubli_bal.control_u[1] = 0.0f;
        cubli_bal.control_u[2] = 0.0f;
        setPhaseVoltage(0, 0, 0, 0);
        setPhaseVoltage(0, 0, 0, 1);
        setPhaseVoltage(0, 0, 0, 2);
        printf("[LQR] Balance Disabled.\r\n");
    }
}

/* 设置平衡模式 */
void LQR_Balance_SetMode(BalanceMode_e mode)
{
    cubli_bal.mode = mode;
    cubli_bal.wheel_int[0] = 0.0f;
    cubli_bal.wheel_int[1] = 0.0f;
    cubli_bal.wheel_int[2] = 0.0f;
    printf("[LQR] Mode set to: %s\r\n",
           mode == BAL_MODE_EDGE_M1 ? "EDGE_M1" :
           (mode == BAL_MODE_EDGE_M2 ? "EDGE_M2" :
           (mode == BAL_MODE_EDGE_M3 ? "EDGE_M3" : "CORNER_3D")));
}

/* 在线配置 LQR 增益 */
void LQR_SetGains(uint8_t motor_idx, float kp, float kd, float kw, float ki)
{
    if (motor_idx < 3)
    {
        cubli_bal.gains[motor_idx].K_theta  = kp;
        cubli_bal.gains[motor_idx].K_dtheta = kd;
        cubli_bal.gains[motor_idx].K_w      = kw;
        cubli_bal.gains[motor_idx].K_i      = ki;
        printf("[LQR] M%d Gains Updated: K_theta=%.2f, K_dtheta=%.2f, K_w=%.4f, K_i=%.4f\r\n",
               motor_idx + 1, kp, kd, kw, ki);
    }
}

/* 在线配置机械平衡点偏置角 (度) */
void LQR_SetZeroAngle(uint8_t motor_idx, float angle_deg)
{
    if (motor_idx < 3)
    {
        cubli_bal.gains[motor_idx].theta_0 = angle_deg * DEG_TO_RAD;
        printf("[LQR] M%d Balance Zero Angle set to: %.2f deg (%.4f rad)\r\n",
               motor_idx + 1, angle_deg, cubli_bal.gains[motor_idx].theta_0);
    }
}

/* 在线配置最大相电压输出 */
void LQR_SetMaxVoltage(float max_v)
{
    if (max_v < 0.5f) max_v = 0.5f;
    if (max_v > voltage_limit) max_v = voltage_limit;
    int i;
    for (i = 0; i < 3; i++)
    {
        cubli_bal.gains[i].max_u = max_v;
    }
    printf("[LQR] Max Balance Voltage set to: %.2f V\r\n", max_v);
}
