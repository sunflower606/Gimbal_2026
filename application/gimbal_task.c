#include "gimbal_task.h"

#include <stddef.h>
#include <stdio.h>

#include "bsp_bt_uart.h"
#include "bt_command.h"
#include "bsp_servo_pwm.h"
#include "bsp_uart_link.h"
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

/* 蓝牙命令：s_bt_cmd 是当前要下发的命令码，s_bt_hold 是还剩几帧要带它 */
static BtCommand_t s_bt_cmd = BT_CMD_NONE;
static uint8_t s_bt_hold = 0U;

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

/* ------------------------------------------- 内部：把循迹板文本转发给蓝牙 --
 * 循迹板每 500 ms 发一行 ASCII 调试文本过来（约 110 字节）。这里原样搬进
 * 蓝牙的发送队列：不改写、不加前缀，电脑上看到的就是循迹板原始输出。
 *
 * 每拍最多搬 64 字节：一是给控制环设上限（即使缓冲里堆了几行也不会一次搬太久），
 * 二是入队本身是非阻塞的，真正的串口搬运由 TXE 中断完成，不会拖慢舵机。
 * ------------------------------------------------------------------------- */
static void GimbalTask_ForwardChassisText(void)
{
    uint16_t budget = 64U;

    while (budget > 0U && BSP_Link_GetRxCount() > 0U) {
        if (BSP_BT_SendByte(BSP_Link_ReadByte()) == 0U) {
            /* 蓝牙队列满（没连上、或电脑没在读）：丢掉这一个字节继续，
             * 绝不阻塞 —— 日志丢了无所谓，舵机不能卡。 */
        }
        budget--;
    }
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

    /* 3.5) 双板链路串口（USART1：PA9 发偏角、PA10 收循迹板调试文本） */
    LinkProtocol_Init();

    /* 3.6) 蓝牙透传串口（USART3：PB10 -> 模块 RXD、PB11 <- 模块 TXD） */
    BSP_BT_Init();
    BtCommand_Init();

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

/* ------------------------------------------------------ 内部：命令回执 ----
 * 手机发完命令后要能立刻看到结果，所以回一行 "CMD:mode2 OK"。
 * 这一行会插在循迹板日志流中间，属于正常现象（同一条蓝牙链路）。
 * ------------------------------------------------------------------------- */
static void GimbalTask_EchoCommand(BtCommand_t command)
{
    char text[24];
    int len;

    len = snprintf(text, sizeof(text), "CMD:%s OK\r\n", BtCommand_ToString(command));
    if (len > 0) {
        (void)BSP_BT_SendBuffer((const uint8_t *)text, (uint16_t)len);
    }
}

void GimbalTask_Loop(void)
{
    VisionFrame_t frame;
    uint32_t now_ms;

    /* ---------- 0. 蓝牙命令：手机发来的 mode1/mode2/stop ----------
     * 独立于视觉链路：哪怕人丢了、云台在保持，命令也要能收到。
     * 解析出来的命令码只改 s_bt_cmd/s_bt_hold，真正的下发在下面第 5 步。 */
    {
        BtCommand_t command = BtCommand_Poll();

        if (command != BT_CMD_NONE) {
            s_bt_cmd = command;
            s_bt_hold = (uint8_t)GIMBAL_BT_CMD_HOLD_FRAMES;
            GimbalTask_EchoCommand(command);
        }
    }

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

        /* 蓝牙命令只保持几帧，之后归 0；循迹板按边沿检测，不会重复执行 */
        payload.command = (s_bt_hold != 0U) ? (uint8_t)s_bt_cmd : 0U;
        if (s_bt_hold != 0U) {
            s_bt_hold--;
            if (s_bt_hold == 0U) {
                s_bt_cmd = BT_CMD_NONE;
            }
        }

        LinkProtocol_Send(&payload);
    }

    /* ---------- 6. 把循迹板发来的调试文本转发给蓝牙 ----------
     * 转发放在最后：它只做入队（几十纳秒），不影响上面任何一步的时序。 */
    GimbalTask_ForwardChassisText();
}
