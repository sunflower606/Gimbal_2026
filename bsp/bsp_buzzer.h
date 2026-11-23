/**
 * @file    bsp_buzzer.h
 * @brief   有源蜂鸣器的 GPIO 控制（云台板 PB4）。
 *
 * 硬件：正点原子云台板 J3 = 3V3 / PB4 / GND，板上带三极管驱动，
 *       所以 PB4 只需给电平 —— 有源蜂鸣器内含振荡电路，不需要方波。
 *
 * 极性：BOARD_BUZZER_ACTIVE_HIGH == 1 时「高电平响」。
 *       若实测相反（上电一直响），把这个宏改成 0U 即可，不用动其它代码。
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

/** @brief 初始化 PB4 为推挽输出，并立即静音。 */
void BSP_Buzzer_Init(void);

/** @brief 鸣叫（按 BOARD_BUZZER_ACTIVE_HIGH 决定电平）。 */
void BSP_Buzzer_On(void);

/** @brief 静音。 */
void BSP_Buzzer_Off(void);

#endif /* BSP_BUZZER_H */
