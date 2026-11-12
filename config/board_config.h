#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ============================================================================
 * 云台板（Gimbal_2026）片上外设配置
 *
 * 主控：STM32F103C8T6（64 KB Flash / 20 KB RAM，Cortex-M3）
 *
 * ★ 引脚定义已全部移到 config/pin_config.h —— 改接线只改那个文件。
 *   本文件只负责「外设怎么配」：串口实例、波特率、定时器时基、舵机换算。
 *
 * 层次约定：只有 bsp/ 层的实现文件可以引用这些宏去操作寄存器；
 *          application/ 与 modules/ 不允许直接包含本文件里的外设句柄。
 * ========================================================================== */

#include "stm32f10x.h"
#include "pin_config.h"

/* ---------------------------------------------------------------- K230 视觉串口
 * 与 K230 侧 person_detection_gimbal.py 的 GimbalLink 一致。
 * 引脚（PA2/PA3）见 pin_config.h。
 * ------------------------------------------------------------------------- */
#define BOARD_VISION_UART                   USART2
#define BOARD_VISION_UART_CLK               RCC_APB1Periph_USART2
#define BOARD_VISION_UART_IRQn              USART2_IRQn
#define BOARD_VISION_UART_IRQHandler        USART2_IRQHandler
#define BOARD_VISION_BAUDRATE               115200U

/* 接收环形缓冲大小（2 的幂便于取模；实际可用 255 字节）。
 * K230 约 33 fps × 13 字节 ≈ 433 B/s，255 字节能容忍主循环卡顿约 0.5 s。 */
#define BOARD_VISION_RX_BUF_SIZE            256U

/* --------------------------------------------------------- 双板链路串口
 * 用途：把「云台 yaw 偏角」发给循迹板，供模式 2 转向用。
 * 本板只发不收，所以只初始化 TX（PA9）。
 * 协议见上级目录 docs/双板分工与协议.md（type = 0x02）。
 * ------------------------------------------------------------------------- */
#define BOARD_LINK_UART                     USART1
#define BOARD_LINK_UART_CLK                 RCC_APB2Periph_USART1
#define BOARD_LINK_BAUDRATE                 115200U

/* ------------------------------------------------------------------ 舵机 PWM
 * TIM1（挂在 APB2）：72 MHz / 72 = 1 MHz 计数，ARR = 20000 -> 20 ms 周期（50 Hz）
 * 1 个计数 = 1 us，所以比较寄存器值就是脉宽微秒数。
 *
 * ⚠️ 通道号必须与 pin_config.h 的引脚成对：
 *      CH1 = PA8（pitch），CH4 = PA11（yaw）
 * ⚠️ TIM1 是高级定时器，除 TIM_Cmd 外还必须在 BSP 里调用
 *      TIM_CtrlPWMOutputs(TIM1, ENABLE) 打开 MOE，否则引脚没有波形。
 * ------------------------------------------------------------------------- */
#define BOARD_SERVO_TIM                     TIM1
#define BOARD_SERVO_TIM_CLK                 RCC_APB2Periph_TIM1

/* 左右旋转（yaw）：TIM1_CH4 -> PA11，舵机行程 270 度 */
#define BOARD_SERVO_YAW_CHANNEL             4U
#define BOARD_SERVO_YAW_FULL_SCALE_DEG      270.0f

/* 上下俯仰（pitch）：TIM1_CH1 -> PA8，舵机行程 180 度 */
#define BOARD_SERVO_PITCH_CHANNEL           1U
#define BOARD_SERVO_PITCH_FULL_SCALE_DEG    180.0f

/* 脉宽与角度换算：0 度 -> 500 us，满量程 -> 2500 us
 * 脉宽(us) = BOARD_SERVO_PULSE_MIN_US + 角度 / 满量程 × BOARD_SERVO_PULSE_SPAN_US
 * 换算关系必须与 gimbal_config.h 里的行程限幅配套，改一个必须改另一个。 */
#define BOARD_SERVO_PULSE_MIN_US            500.0f
#define BOARD_SERVO_PULSE_SPAN_US           2000.0f
#define BOARD_SERVO_PULSE_MAX_US            2500.0f    /* = MIN + SPAN，仅用于断言/文档 */

/* 定时器时基（1 MHz 计数，20 ms 周期） */
#define BOARD_SERVO_TIM_PRESCALER           (72U - 1U)
#define BOARD_SERVO_TIM_PERIOD              (20000U - 1U)

#endif /* BOARD_CONFIG_H */
