#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H

/* ============================================================================
 * BSP：二自由度舵机 PWM（TIM1 双通道，50 Hz）
 *
 * 通道与引脚（与 config/board_config.h 一致，改接线只需改那里）：
 *     左右旋转 yaw   ：TIM1_CH4 -> PA11，舵机行程 270 度
 *     上下俯仰 pitch ：TIM1_CH1 -> PA8，舵机行程 180 度
 *
 * 时基：72 MHz / 72 = 1 MHz 计数，ARR = 20000 -> 20 ms 周期，
 *      因此「比较寄存器值 = 脉宽微秒数」。
 *
 * ⚠️ TIM1 是高级定时器，Init 里除 TIM_Cmd 外还会调用
 *    TIM_CtrlPWMOutputs(TIM1, ENABLE) 打开 MOE；少了这一步引脚没有波形。
 *
 * 层次约定：本层是唯一操作 TIM1 的地方；application/modules 只传角度。
 * ========================================================================== */

#include <stdint.h>

/* 初始化两路舵机 PWM，并把输出置为各自行程中点（1500 us）。
 * 上电即给出有效脉冲，避免舵机因为「没有信号」而松掉。 */
void BSP_Servo_Init(void);

/**
 * @brief 设置左右旋转（yaw）舵机角度。
 * @param angle_deg 机械角度，单位度，有效范围 0 ~ 270（对应 500 ~ 2500 us）。
 *
 * @note 内部会把脉宽夹到 [500, 2500] us 作为最后一道保护；
 *       正常调用方（modules/gimbal_controller）已经先做过行程限幅。
 */
void BSP_Servo_SetYawDeg(float angle_deg);

/**
 * @brief 设置上下俯仰（pitch）舵机角度。
 * @param angle_deg 机械角度，单位度，有效范围 0 ~ 180（对应 500 ~ 2500 us）。
 */
void BSP_Servo_SetPitchDeg(float angle_deg);

#endif /* BSP_SERVO_PWM_H */
