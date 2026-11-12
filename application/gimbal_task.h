#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/* ============================================================================
 * Application：云台跟踪任务
 *
 * 这是本工程唯一的一体化「机器人应用逻辑」，负责把三个模块串起来：
 *
 *     bsp_servo_pwm  <-  gimbal_controller  <-  vision_protocol  <-  bsp_uart
 *
 * 本任务与需求文档《实验室学习项目》的对应关系：
 *   「云台人体跟踪：摄像头采集图像，识别人体，输出人体在画面中的坐标；
 *     STM32 根据坐标偏差调节舵机角度，使人保持在画面中心。」
 *
 * 范围说明（按任务要求裁剪）：
 *   本工程只负责舵机云台与 K230 视觉；模式切换按键、循迹、电机、超声测距
 *   属于另一块主控（Chassis_2026），本工程不包含，也不预留它们的代码。
 *
 * 层次约定：application 层只调用 modules/bsp 的公共接口，
 *          不出现任何寄存器或 HAL/StdPeriph 调用。
 * ========================================================================== */

#include <stdint.h>

/**
 * @brief 初始化云台任务：舵机 PWM、OLED、控制器状态、视觉串口。
 * @note 上电后云台立即摆到行程中点，避免通电瞬间乱甩。
 */
void GimbalTask_Init(void);

/**
 * @brief 云台任务单次执行，调用周期必须等于 GIMBAL_CONTROL_PERIOD_MS（10 ms）。
 *
 * 内部完成：收帧 -> 取最新帧 -> 控制计算 -> 输出 PWM，
 * 并按 100 ms / 500 ms 节流刷新 OLED 与串口回传。
 */
void GimbalTask_Loop(void);

#endif /* GIMBAL_TASK_H */
