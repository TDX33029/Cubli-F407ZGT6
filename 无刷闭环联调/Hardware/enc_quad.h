/**
  ******************************************************************************
  * @file           : enc_quad.h
  * @brief          : MT6701 编码器读取双通道抽象层
  *                   方式A (原方法): 三路 I2C 逐次读取 MT6701 绝对角度
  *                   方式B (新增): F407 三个定时器硬件正交解码 (ABZ 增量输出)
  *
  * 通过宏 ENC_USE_HW_QUAD 在编译期选择 (上电后不再切换):
  *   ENC_USE_HW_QUAD 0 -> 方式A I2C 绝对式读取 (默认, 兼容原固件行为)
  *   ENC_USE_HW_QUAD 1 -> 方式B 定时器硬件正交解码
  *
  * 硬件正交解码引脚 (依据网表 Netlist_MB_Schematic_2026-10-07.net):
  *   M1 (ABZ-1): A=PA5  -> TIM2_CH1 (AF1),  B=PB3 -> TIM2_CH2 (AF1)
  *   M2 (ABZ-2): A=PA0  -> TIM5_CH1 (AF2),  B=PA1 -> TIM5_CH2 (AF2)
  *   M3 (ABZ-3): A=PC6  -> TIM8_CH1 (AF3),  B=PC7 -> TIM8_CH2 (AF3)
  *
  * 注意: 方式B占用 TIM5, 因此 micros() 微秒时基在 HW_QUAD 模式下
  *       自动迁移到 TIM7 (16位 1MHz + 溢出中断扩展, 见 timer.c)。
  ******************************************************************************
  */

#ifndef __ENC_QUAD_H__
#define __ENC_QUAD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========================================================================== */
/* 编码器读取方式选择 (编译期宏, 上电后不再切换)                                */
/* ========================================================================== */
#define ENC_USE_HW_QUAD        0   /* 0=三路I2C绝对式读取(原方法); 1=三定时器硬件正交解码(新方法) */

/* MT6701 ABZ 增量输出每圈正交计数值 (x4解码后)。
 * 若实际 MT6701 ABZ 配置不同 (例如 512/2048), 只需修改此处, 禁止运行时修改。 */
#define ENC_QUAD_COUNTS_PER_REV   1024

/* ========================================================================== */
/* 统一编码器访问接口 (所有上层代码只允许经由本接口读取编码器)                   */
/* idx: 传感器索引 0/1/2 (对应 I2C1/I2C2/I2C3 或 TIM2/TIM5/TIM8)               */
/* 返回: 0=成功, 1=失败(HW_QUAD 模式下恒为 0)                                   */
/*   raw: 原始计数值 (I2C: 0~16383, 定时器: 0~(COUNTS_PER_REV-1) 循环)          */
/*   deg: 单圈机械角度 [0, 360)                                                 */
/* ========================================================================== */
uint8_t ENC_GetAngle(uint8_t idx, int16_t *raw, float *deg);

/* 原方法: 三路 I2C MT6701 绝对角度读取 (始终保留可调用) */
uint8_t ENC_I2C_GetAngle(uint8_t idx, int16_t *raw, float *deg);

/* 新方法: 定时器硬件正交解码读取 (ENC_USE_HW_QUAD=1 时由 ENC_GetAngle 路由) */
void ENC_Quad_Init(void);
uint8_t ENC_Quad_GetAngle(uint8_t idx, int16_t *raw, float *deg);

#ifdef __cplusplus
}
#endif

#endif /* __ENC_QUAD_H__ */
