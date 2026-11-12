#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/* ============================================================================
 * 视觉云台系统 PCB（STM32F103C8T6 最小系统板）引脚定义
 * —— **改引脚只改这个文件**
 *
 * 本板已占用引脚一览：
 *   PA2   USART2_TX   K230 串口（4 针排针，丝印 TX）
 *   PA3   USART2_RX   K230 串口（4 针排针，丝印 RX）
 *   PA8   TIM1_CH1    云台(上) 俯仰舵机 180°
 *   PA11  TIM1_CH4    云台(下) 旋转舵机 270°
 *   PA9   USART1_TX   板间通信（4 针排针，丝印 TX）
 *   PB4   GPIO        蜂鸣器（3V3/PB4/GND，目前无驱动代码）
 *   PB12  GPIO        OLED SCL（宏保留，板上无 OLED 座）
 *   PB13  GPIO        OLED SDA（宏保留，板上无 OLED 座）
 *
 * 为什么单独拆出这个文件：
 *   引脚是「板子怎么焊」的事，定时器/波特率/中断号是「程序怎么配」的事。
 *   两者混在一起时，改一根线要在一堆外设配置里翻半天。现在分开了：
 *       pin_config.h    只放「哪根线接哪个脚」—— 你平时要改的就是这个
 *       board_config.h  只放「外设怎么配」（USART/TIM 实例、波特率、预分频）
 *
 * 使用规则：
 *   1) 其它文件（bsp 与 modules 下的 .c）只允许通过这里的宏拿引脚，
 *      不允许出现第二处 GPIO_Pin_x 硬编码；
 *   2) 改完引脚后编译一次即可，board_config.h 会跟着生效；
 *   3) 改完记得同步 docs/引脚分配.md。
 *
 * ⚠️ 三条改引脚时容易踩的约束：
 *   1)【外设绑定】两路舵机必须落在 TIM1 的 CH1/CH4 上：
 *         TIM1_CH1 = PA8，TIM1_CH2 = PA9，TIM1_CH3 = PA10，TIM1_CH4 = PA11
 *      本板用的 PA8/PA11 正好是 CH1/CH4。换到别的组要同时改
 *      board_config.h 的 BOARD_SERVO_TIM 和通道号。
 *      TIM1 是高级定时器，输出前必须 TIM_CtrlPWMOutputs(TIM1, ENABLE) 打开 MOE，
 *      否则引脚没有波形（BSP 里已经做了）。
 *   2)【串口绑定】USART2 的 TX/RX 只能是 PA2/PA3（本板未做重映射）。
 *      换脚等于换串口实例，board_config.h 的 BOARD_VISION_UART 也要跟着改。
 *   3)【5V 容忍】PA0~PA7 是 TTa（不容忍 5V），PA8~PA15、PB2~PB15 是 FT。
 *      本板没有 5V 输入信号：K230 串口是 3.3V 电平，两路舵机是 MCU 输出。
 *
 * 引脚总表见 docs/引脚分配.md
 * ========================================================================== */

#include "stm32f10x.h"

/* ---------------------------------------------------------------- 舵机 PWM
 * 云台(上) 3 针排针：GND / 5V / PA8   -> 俯仰 pitch，180° 舵机（TIM1_CH1）
 * 云台(下) 3 针排针：GND / 5V / PA11  -> 旋转 yaw，270° 舵机（TIM1_CH4）
 *
 * 「上/下」与「俯仰/旋转」的对应是按下述推断写的：
 *   云台(上) 是俯仰轴、云台(下) 是旋转轴。
 * 若实机方向相反（上电后转错轴），只需要把下面两个 PIN 宏（以及
 * board_config.h 里对应的 CHANNEL 宏）对调，不要改控制参数。
 * ------------------------------------------------------------------------- */
#define BOARD_SERVO_GPIO                    GPIOA
#define BOARD_SERVO_GPIO_CLK                RCC_APB2Periph_GPIOA

#define BOARD_SERVO_YAW_PIN                 GPIO_Pin_11     /* PA11 = TIM1_CH4，左右旋转 270° */
#define BOARD_SERVO_PITCH_PIN               GPIO_Pin_8      /* PA8  = TIM1_CH1，上下俯仰 180° */

/* ------------------------------------------------------------ K230 视觉串口
 * 4 针排针：5V / GND / PA3 / PA2，丝印上方标 RX TX。
 * 按丝印顺序（PA3=RX、PA2=TX）：
 *   K230 UART2_TX(GPIO44) --> STM32 PA3（RX）
 *   K230 UART2_RX(GPIO45) <-- STM32 PA2（TX）  只收不发时可不接
 * 必须共地。
 * ------------------------------------------------------------------------- */
#define BOARD_VISION_GPIO                   GPIOA
#define BOARD_VISION_GPIO_CLK               RCC_APB2Periph_GPIOA
#define BOARD_VISION_TX_PIN                 GPIO_Pin_2      /* PA2 = USART2_TX */
#define BOARD_VISION_RX_PIN                 GPIO_Pin_3      /* PA3 = USART2_RX */

/* ------------------------------------------------------------ 双板链路串口
 * 4 针排针：PA9 / PA10 / GND / 3V3，丝印标 TX RX（PA9=TX、PA10=RX）。
 * 只发不收：把「云台 yaw 偏角」发给循迹板的 PA10。
 * 板上另有 PA10(RX)，本工程只发不收，不配置。
 * ------------------------------------------------------------------------- */
#define BOARD_LINK_GPIO                     GPIOA
#define BOARD_LINK_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define BOARD_LINK_TX_PIN                   GPIO_Pin_9      /* PA9 = USART1_TX */

/* ------------------------------------------------------------------ 蜂鸣器
 * 3 针排针：3V3 / PB4 / GND。
 * 蜂鸣器 3V3/PB4/GND，目前无驱动代码 —— 这里只登记引脚，不初始化、不驱动。
 * 注意 PB4 复位后是 JTAG 的 NJTRST，将来写驱动时需先
 * GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)（保留 SWD）。
 * ------------------------------------------------------------------------- */
#define BOARD_BUZZER_GPIO                   GPIOB
#define BOARD_BUZZER_PIN                    GPIO_Pin_4
#define BOARD_BUZZER_GPIO_CLK               RCC_APB2Periph_GPIOB

/* -------------------------------------------------------------- OLED 软件 I2C
 * 0.96 寸 SSD1306，GPIO 模拟 I2C。开漏输出，靠模块板载上拉电阻拉高。
 * 板上无 OLED 座，这两个脚在闲置排针上；要用 OLED 需飞线 3V3/GND/SCL/SDA。
 * 宏保留是为了不改动 modules/oled.c 与 bsp_soft_i2c.c 的接口。
 * ------------------------------------------------------------------------- */
#define BOARD_OLED_GPIO                     GPIOB
#define BOARD_OLED_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define BOARD_OLED_SCL_PIN                  GPIO_Pin_12     /* PB12 = SCL */
#define BOARD_OLED_SDA_PIN                  GPIO_Pin_13     /* PB13 = SDA */

/* ---------------------------------------------------------------- 空闲引脚
 * 空闲（闲置排针 A 组）：PA4 PA5 PA6 PA7 PB0 PB1 PB10 PB11 PB7 PB8
 * 空闲（闲置排针 B 组）：PA0 PA1 PB9 PB6 PB5 PA12 PB15 PB14 PB13 PB12
 *
 * 说明：
 *   - PB12/PB13 虽在 B 组排针上，但上面已保留给 OLED 宏；真正接 OLED 之前
 *     先确认没有别的功能同时占用这两个脚。
 *   - PA13/PA14 是 SWDIO/SWCLK 调试口，PB2 是 BOOT1，均不要占用。
 * ------------------------------------------------------------------------- */

#endif /* PIN_CONFIG_H */
