/**
 * @file    buzzer.h
 * @brief   蜂鸣器节奏播放器（非阻塞）。
 *
 * 设计要点
 * --------
 * 1) **绝不阻塞**：整套节奏由 Buzzer_Step() 每 10 ms 推进一小步，
 *    调用者（GimbalTask_Loop）永远是立即返回的。
 *    如果在控制环里 Delay_ms(1000) 播长鸣，舵机会瞎转 1 秒 —— 这是绝对禁止的。
 *
 * 2) **事件边沿触发**：同一个事件只在上沿播放一次；重复播放由调用者判断，
 *    否则车一直丢线会响成一片。
 *
 * 3) 播放期间来了新事件 → **直接打断，播新的**。
 *    例如正在播"过十字"，紧接着到了停车线，应该马上改成到站长鸣。
 */
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/** 音型号 —— 与底盘 modules/vision_link.c 的 0x05 帧第 8 字节一一对应 */
typedef enum {
    BUZZER_PATTERN_START = 0,      /* 起跑：两声上行 嘀—嘀 */
    BUZZER_PATTERN_CROSS = 1,      /* 过十字：一声短 嘀 */
    BUZZER_PATTERN_LOST = 2,       /* 丢线/丢人：三声急促 嘀嘀嘀 */
    BUZZER_PATTERN_STOP_LINE = 3,  /* 到停车线：长鸣 1 s */
    BUZZER_PATTERN_LINK_LOST = 4,  /* 断链：长鸣 0.5 s */
    BUZZER_PATTERN_LOW_BAT = 5,    /* 低电压：一短声 */
    BUZZER_PATTERN_COUNT = 6
} BuzzerPattern_t;

/** @brief 初始化 BSP 层并静音。 */
void Buzzer_Init(void);

/** @brief 播放一个音型（会打断正在播放的）。 */
void Buzzer_Play(BuzzerPattern_t pattern);

/** @brief 每 10 ms 调一次（与云台控制周期一致），推进节奏。 */
void Buzzer_Step(void);

/** @brief 是否正在播放（诊断/串口用）。 */
uint8_t Buzzer_IsPlaying(void);

/** @brief 总开关：0 = 静音（演示时可关掉）。 */
void Buzzer_SetEnabled(uint8_t enabled);

/** @return 1 = 允许发声。 */
uint8_t Buzzer_IsEnabled(void);

#endif /* BUZZER_H */
