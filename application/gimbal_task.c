#include "gimbal_task.h"

#include <stddef.h>

#include "bsp_servo_pwm.h"
#include "Delay.h"
#include "gimbal_config.h"
#include "gimbal_controller.h"
#include "link_protocol.h"
#include "oled.h"
#include "vision_config.h"
#include "vision_protocol.h"

/* 控制器实例（本板只有一个云台，静态分配即可，无堆分配） */
static GimbalController_t s_gimbal;

/* 节流计时基准 */
static uint32_t s_last_display_ms = 0U;
static uint32_t s_last_debug_ms = 0U;
static uint32_t s_last_link_ms = 0U;    /* 双板链路发送基准 */

/* 最近一次显示的帧计数（用于 OLED 第 4 行） */
static uint32_t s_display_frame_count = 0U;

/* ------------------------------------------------------------ 内部：显示 -- */

/* 只画固定标签，上电调用一次 */
static void GimbalTask_DrawLabels(void)
{
    OLED_ShowString(1U, 1U, "Yaw:");
    OLED_ShowString(2U, 1U, "Pit:");
    OLED_ShowString(3U, 1U, "X:");
    OLED_ShowString(3U, 7U, "Y:");
    OLED_ShowString(4U, 1U, "St:");
}

/* 刷新动态内容。约 3~5 ms，必须节流。 */
static void GimbalTask_RefreshDisplay(const VisionFrame_t *frame)
{
    uint32_t yaw_deg;
    uint32_t pitch_deg;

    /* 角度显示为整数度；负值理论上不会出现（已被行程限幅到 >= 3 度） */
    yaw_deg = (uint32_t)GimbalController_GetYawDeg(&s_gimbal);
    pitch_deg = (uint32_t)GimbalController_GetPitchDeg(&s_gimbal);

    OLED_ShowNum(1U, 5U, yaw_deg, 3U);
    OLED_ShowNum(2U, 5U, pitch_deg, 3U);

    /* 第 3 行显示低通后的偏差（乘方向符号、过死区之前的原始量） */
    OLED_ShowSignedNum(3U, 3U, (int32_t)GimbalController_GetFilteredX(&s_gimbal), 2U);
    OLED_ShowSignedNum(3U, 9U, (int32_t)GimbalController_GetFilteredY(&s_gimbal), 2U);

    /* 第 4 行：跟踪状态 + 收到帧数 */
    if (GimbalController_IsTracking(&s_gimbal) != 0U) {
        OLED_ShowString(4U, 4U, "LOCK");
    } else {
        OLED_ShowString(4U, 4U, "LOST");
    }

    if (frame != NULL) {
        s_display_frame_count = frame->frame_count;
    }
    /* OLED 只有 4 位数字宽度，超过 9999 固定显示 9999 */
    OLED_ShowNum(4U, 9U, (s_display_frame_count > 9999U) ? 9999U : s_display_frame_count, 4U);
}

/* ------------------------------------------------------------ 对外接口 -- */

void GimbalTask_Init(void)
{
    uint32_t now_ms;

    /* 1) 舵机 PWM 必须先初始化，后面的控制器初始化会立刻写 CCR */
    BSP_Servo_Init();

    /* 2) OLED（内部会初始化软件 I2C 引脚） */
    OLED_Init();

    /* 3) 视觉串口 + 帧状态 */
    Vision_Init();

    /* 3.5) 双板链路串口（USART1，只发不收） */
    LinkProtocol_Init();

    /* 4) 控制器参数与状态，并摆到行程中点 */
    GimbalController_Init(&s_gimbal);

    /* 5) 画固定标签，并先刷一次完整画面 */
    OLED_Clear();
    GimbalTask_DrawLabels();

    now_ms = Delay_GetMs();
    s_last_display_ms = now_ms;
    s_last_debug_ms = now_ms;
    s_last_link_ms = now_ms;
    GimbalTask_RefreshDisplay(NULL);
}

void GimbalTask_Loop(void)
{
    VisionFrame_t frame;
    uint32_t now_ms;

    /* ---------- 1. 收帧：把缓冲里所有完整帧解析出来，只保留最新一帧 ---------- */
    Vision_Poll();
    frame = Vision_GetFrame();

    /* ---------- 2. 控制：算角度并输出 PWM（内部处理丢失保持） ---------- */
    GimbalController_Update(&s_gimbal, &frame);

    /* ---------- 3. 节流刷 OLED ---------- */
    now_ms = Delay_GetMs();
    if ((uint32_t)(now_ms - s_last_display_ms) >= GIMBAL_DISPLAY_PERIOD_MS) {
        s_last_display_ms = now_ms;
        GimbalTask_RefreshDisplay(&frame);
    }

    /* ---------- 4. 节流串口回传（调试用，可在量产后删除） ---------- */
    if ((uint32_t)(now_ms - s_last_debug_ms) >= GIMBAL_DEBUG_PERIOD_MS) {
        s_last_debug_ms = now_ms;
        Vision_PrintFrame();
    }

    /* ---------- 5. 节流发给循迹板（模式 2 转向用） ----------
     * 发的是「云台 yaw 相对正前方的偏角」，不是画面偏差：
     * 人一直被云台保持在画面中心，画面偏差恒接近 0，不携带方向信息；
     * 而云台转了多少度去对准人，正好编码了人相对车头的方位。
     *
     * 无论有没有人都要发（无人时 flags 不带 detected 位）：
     * 停发会让循迹板无法区分「没人」和「断线」。 */
    if ((uint32_t)(now_ms - s_last_link_ms) >= GIMBAL_LINK_PERIOD_MS) {
        LinkPayload_t payload;

        s_last_link_ms = now_ms;

        payload.yaw_dev_tenth_deg = GimbalController_GetYawDevTenthDeg(&s_gimbal);
        payload.offset_x = frame.x_norm;
        payload.distance_cm = 0U;   /* 预留：将来接测距后填真实距离 */
        payload.detected = GimbalController_IsTracking(&s_gimbal);

        LinkProtocol_Send(&payload);
    }
}
