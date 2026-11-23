/**
 * @file    buzzer.c
 * @brief   蜂鸣器节奏播放器（非阻塞，10 ms 步进）。
 *
 * 节奏表用「段」描述：每段 = {时长 ms, 电平}。
 * 这样加新音型只要往 s_patterns[] 里加一行，不用改播放逻辑。
 */
#include "buzzer.h"
#include "bsp_buzzer.h"
#include "gimbal_config.h"

/* 与 GimbalTask_Loop 的调用周期一致（gimbal_config.h 里的 10 ms） */
#define BUZZER_STEP_MS            10U

typedef struct {
    uint16_t ms;   /* 这一段的时长 */
    uint8_t on;    /* 1 = 响，0 = 静音 */
} BuzzerSeg_t;

/* ---- 音型定义 --------------------------------------------------------- */

/* 起跑：80 ms 响 + 120 ms 静 + 80 ms 响 → "嘀—嘀" */
static const BuzzerSeg_t s_pat_start[] = {
    { 80U, 1U }, { 120U, 0U }, { 80U, 1U }
};

/* 过十字：一声短 "嘀" */
static const BuzzerSeg_t s_pat_cross[] = {
    { 80U, 1U }
};

/* 丢线/丢人：三声急促 "嘀嘀嘀" */
static const BuzzerSeg_t s_pat_lost[] = {
    { 60U, 1U }, { 60U, 0U }, { 60U, 1U }, { 60U, 0U }, { 60U, 1U }
};

/* 到停车线：长鸣 1 s */
static const BuzzerSeg_t s_pat_stop_line[] = {
    { 1000U, 1U }
};

/* 断链：长鸣 0.5 s（比到站短一半，听感上区分得出来） */
static const BuzzerSeg_t s_pat_link_lost[] = {
    { 500U, 1U }
};

/* 低电压：一短声（由底盘每 5 s 触发一次） */
static const BuzzerSeg_t s_pat_low_bat[] = {
    { 100U, 1U }
};

typedef struct {
    const BuzzerSeg_t *segs;
    uint8_t count;
} BuzzerDef_t;

static const BuzzerDef_t s_patterns[BUZZER_PATTERN_COUNT] = {
    { s_pat_start,     (uint8_t)(sizeof(s_pat_start)     / sizeof(s_pat_start[0]))     },
    { s_pat_cross,     (uint8_t)(sizeof(s_pat_cross)     / sizeof(s_pat_cross[0]))     },
    { s_pat_lost,      (uint8_t)(sizeof(s_pat_lost)      / sizeof(s_pat_lost[0]))      },
    { s_pat_stop_line, (uint8_t)(sizeof(s_pat_stop_line) / sizeof(s_pat_stop_line[0])) },
    { s_pat_link_lost, (uint8_t)(sizeof(s_pat_link_lost) / sizeof(s_pat_link_lost[0])) },
    { s_pat_low_bat,   (uint8_t)(sizeof(s_pat_low_bat)   / sizeof(s_pat_low_bat[0]))   }
};

/* ---- 播放状态 --------------------------------------------------------- */

static const BuzzerDef_t *s_cur = 0;   /* 当前音型（0 = 没在播） */
static uint8_t  s_seg = 0U;            /* 当前段下标 */
static uint16_t s_elapsed_ms = 0U;     /* 当前段已过的毫秒 */
static uint8_t  s_enabled = 1U;

/* 把某一段的电平写进 BSP */
static void Buzzer_Apply(uint8_t on)
{
    if (on != 0U) {
        BSP_Buzzer_On();
    } else {
        BSP_Buzzer_Off();
    }
}

void Buzzer_Init(void)
{
    BSP_Buzzer_Init();
    s_cur = 0;
    s_seg = 0U;
    s_elapsed_ms = 0U;
    /* 上电默认值来自配置宏；运行期可用蓝牙 bz on/off 覆盖，掉电后回到这里 */
    s_enabled = (GIMBAL_BUZZER_ENABLE != 0U) ? 1U : 0U;
}

void Buzzer_Play(BuzzerPattern_t pattern)
{
    const BuzzerDef_t *def;

    if (s_enabled == 0U) {
        return;
    }
    if ((uint32_t)pattern >= (uint32_t)BUZZER_PATTERN_COUNT) {
        return;
    }

    def = &s_patterns[pattern];
    if (def->count == 0U) {
        return;
    }

    /* 打断正在播的，直接从头播新的 */
    s_cur = def;
    s_seg = 0U;
    s_elapsed_ms = 0U;
    Buzzer_Apply(def->segs[0].on);
}

void Buzzer_Step(void)
{
    if (s_cur == 0) {
        return;
    }

    s_elapsed_ms += BUZZER_STEP_MS;

    /* 用 while 而不是 if：某段时长比 10 ms 还短时也能正确跨段 */
    while (s_elapsed_ms >= s_cur->segs[s_seg].ms) {
        s_elapsed_ms -= s_cur->segs[s_seg].ms;
        s_seg++;

        if (s_seg >= s_cur->count) {
            /* 整段播完，收尾静音 */
            s_cur = 0;
            Buzzer_Apply(0U);
            return;
        }
        Buzzer_Apply(s_cur->segs[s_seg].on);
    }
}

uint8_t Buzzer_IsPlaying(void)
{
    return (s_cur != 0) ? 1U : 0U;
}

void Buzzer_SetEnabled(uint8_t enabled)
{
    s_enabled = (enabled != 0U) ? 1U : 0U;
    if (s_enabled == 0U) {
        s_cur = 0;
        Buzzer_Apply(0U);
    }
}

uint8_t Buzzer_IsEnabled(void)
{
    return s_enabled;
}
