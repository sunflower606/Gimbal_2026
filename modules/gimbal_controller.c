#include "gimbal_controller.h"

#include <stddef.h>

#include "bsp_servo_pwm.h"
#include "gimbal_config.h"
#include "vision_config.h"

/* 兜底触发必须比"判丢"更晚，否则一丢人就开始扫（太敏感） */
#if (GIMBAL_SWEEP_TRIGGER_MS <= GIMBAL_LOSE_TIMEOUT_MS)
#error "GIMBAL_SWEEP_TRIGGER_MS must be greater than GIMBAL_LOSE_TIMEOUT_MS"
#endif

/* ---------------------------------------------------------------- 内部工具 -- */

/* 把 v 夹到 [lo, hi] */
static float GimbalController_Clamp(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }

    return value;
}

/* 软死区：把 |error| 小于死区的那一段整体扣掉。
 *
 * 为什么不用「小于 d 就置 0」：那种写法在边界不连续，微分项
 * Kd*(e - e_last)/dt 会把跳变放大成一次冲击（实测单拍约 0.21 度，
 * 是软写法的 5 倍）。软写法输出从 0 平滑增长，死区内行为完全一样。 */
static float GimbalController_ApplyDeadZone(float error, float dead_zone)
{
    if (error > dead_zone) {
        return error - dead_zone;
    }
    if (error < -dead_zone) {
        return error + dead_zone;
    }

    return 0.0f;
}

/* 按角度输出一次 PWM。yaw 走 270 度舵机，pitch 走 180 度舵机。 */
static void GimbalController_ApplyOutput(const GimbalController_t *ctrl)
{
    BSP_Servo_SetYawDeg(ctrl->yaw_angle_deg);
    BSP_Servo_SetPitchDeg(ctrl->pitch_angle_deg);
}

/* ------------------------------------------------------------------ 接口 -- */

void GimbalController_Init(GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->kp = GIMBAL_PID_KP;
    ctrl->ki = GIMBAL_PID_KI;
    ctrl->kd = GIMBAL_PID_KD;
    ctrl->dt_s = GIMBAL_CONTROL_DT_S;
    ctrl->lpf_alpha = GIMBAL_LPF_ALPHA;
    ctrl->dead_zone = GIMBAL_DEAD_ZONE;
    ctrl->sign_x = GIMBAL_SIGN_X;
    ctrl->sign_y = GIMBAL_SIGN_Y;
    ctrl->deg_per_unit_x = GIMBAL_DEG_PER_UNIT_X;
    ctrl->deg_per_unit_y = GIMBAL_DEG_PER_UNIT_Y;
    ctrl->yaw_min_deg = GIMBAL_YAW_MIN_DEG;
    ctrl->yaw_max_deg = GIMBAL_YAW_MAX_DEG;
    ctrl->pitch_min_deg = GIMBAL_PITCH_MIN_DEG;
    ctrl->pitch_max_deg = GIMBAL_PITCH_MAX_DEG;

    ctrl->yaw_angle_deg = GIMBAL_YAW_HOME_DEG;
    ctrl->pitch_angle_deg = GIMBAL_PITCH_HOME_DEG;

    ctrl->filt_x = 0.0f;
    ctrl->filt_y = 0.0f;
    ctrl->integ_x = 0.0f;
    ctrl->integ_y = 0.0f;
    ctrl->last_error_x = 0.0f;
    ctrl->last_error_y = 0.0f;
    ctrl->pending_yaw = 0.0f;
    ctrl->pending_pitch = 0.0f;
    ctrl->tracking = 0U;

    /* 限位反向扫视状态（2026-11-19） */
    ctrl->sweep_state = 0U;
    ctrl->sweep_dir = 0;
    ctrl->sweep_rounds = 0U;
    ctrl->sweep_edge_ms = 0U;
    ctrl->sweep_lost_ms = 0U;
    ctrl->last_turn_dir = 0;
    ctrl->boot_ms = 0U;
    ctrl->vision_seen = 0U;
    ctrl->target_seen = 0U;
    ctrl->chassis_mode = GIMBAL_CHASSIS_MODE_STOP;   /* 未上报前按"停"处理 */
    ctrl->mode_seen = 0U;

    /* 角度先夹一次，保证 config 里写错行程也不会越界 */
    ctrl->yaw_angle_deg = GimbalController_Clamp(ctrl->yaw_angle_deg,
                                                 ctrl->yaw_min_deg, ctrl->yaw_max_deg);
    ctrl->pitch_angle_deg = GimbalController_Clamp(ctrl->pitch_angle_deg,
                                                   ctrl->pitch_min_deg, ctrl->pitch_max_deg);

    GimbalController_ApplyOutput(ctrl);
}

void GimbalController_SetChassisMode(GimbalController_t *ctrl, uint8_t mode)
{
    ctrl->chassis_mode = mode;
    ctrl->mode_seen = 1U;
}

void GimbalController_SetAngle(GimbalController_t *ctrl, float yaw_deg, float pitch_deg)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->yaw_angle_deg = GimbalController_Clamp(yaw_deg, ctrl->yaw_min_deg, ctrl->yaw_max_deg);
    ctrl->pitch_angle_deg = GimbalController_Clamp(pitch_deg, ctrl->pitch_min_deg,
                                                   ctrl->pitch_max_deg);

    /* 手动摆位后清掉历史，避免带着旧偏差继续跑 */
    ctrl->integ_x = 0.0f;
    ctrl->integ_y = 0.0f;
    ctrl->last_error_x = 0.0f;
    ctrl->last_error_y = 0.0f;
    ctrl->filt_x = 0.0f;
    ctrl->filt_y = 0.0f;

    GimbalController_ApplyOutput(ctrl);
}

void GimbalController_Output(const GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    GimbalController_ApplyOutput(ctrl);
}

/* ------------------------------------------------- 限位反向扫视（私有） -- */
/**
 * @brief 目标丢失时调用：顶到行程端点就往反方向扫，扫到另一端再折回。
 *
 * 只在「人已经看不到」时进入（调用方在 Update 的丢失分支里调用），
 * 所以不会和正常跟踪打架。
 *
 * 状态机：
 *   sweep_state == 0（待触发）
 *       若 yaw 贴在端点 ±GIMBAL_SWEEP_EDGE_DEG 内 -> 累加 sweep_edge_ms
 *       连续贴边达到 GIMBAL_SWEEP_CONFIRM_MS -> 进入扫视，方向朝"离开端点"
 *       否则把计时清零（车身一晃就贴边不该触发）
 *   sweep_state == 1（扫视中）
 *       每拍转 GIMBAL_SWEEP_SPEED_DEG_S × dt_s
 *       到达另一端 -> 单程数 +1 并换向
 *       单程数达到 GIMBAL_SWEEP_PASSES -> 回上电中点并退出
 */
static void GimbalController_SweepStep(GimbalController_t *ctrl)
{
    const float step = GIMBAL_SWEEP_SPEED_DEG_S * ctrl->dt_s;

    /* ---------------- 上电静默：两道闸门（2026-11-20） ----------------
     * K230 上电要 5~15 s 才出第一帧，这段"无人可跟"的时间不能当成"人丢了"。 */
    if (ctrl->boot_ms < (uint32_t)GIMBAL_BOOT_GRACE_MS) {
        return;                     /* 闸门 1：上电保护窗口内不扫 */
    }
    if ((GIMBAL_SWEEP_NEED_TARGET != 0U) && (ctrl->target_seen == 0U)) {
        return;                     /* 闸门 2：从没跟到过人，不存在"找人" */
    }
    /* 闸门 3（2026-11-23）：只在模式 1（循迹）下扫。
     * 模式 2 由车转身找人，云台扫视反而把偏角搅乱；STOP 时扫视纯属白耗。
     * 没收到过底盘上报也一律不扫（宁可不动，也不要乱扫）。 */
    if (GIMBAL_SWEEP_MODE_ONLY != 0U) {
        if ((ctrl->mode_seen == 0U) || (ctrl->chassis_mode != GIMBAL_CHASSIS_MODE_LINE)) {
            return;
        }
    }

    /* ---------------- 待触发：两种触发条件 ---------------- */
    if (ctrl->sweep_state == 0U) {
        uint8_t at_edge = 0U;

        /* 条件 A：贴在端点 ±EDGE 内（真被车身推到限位） */
        if (ctrl->yaw_angle_deg <= (ctrl->yaw_min_deg + GIMBAL_SWEEP_EDGE_DEG)) {
            ctrl->sweep_dir = 1;    /* 贴下限 -> 往角度增大方向找 */
            at_edge = 1U;
        } else if (ctrl->yaw_angle_deg >= (ctrl->yaw_max_deg - GIMBAL_SWEEP_EDGE_DEG)) {
            ctrl->sweep_dir = -1;   /* 贴上限 -> 往角度减小方向找 */
            at_edge = 1U;
        }

        if (at_edge != 0U) {
            ctrl->sweep_edge_ms = (uint16_t)(ctrl->sweep_edge_ms + GIMBAL_CONTROL_PERIOD_MS);
        } else {
            ctrl->sweep_edge_ms = 0U;   /* 没贴边，计时作废 */
            ctrl->sweep_dir = 0;
        }

        /* 条件 B：丢人累计时间（从"判定丢失"起算，加上判丢本身的
         * GIMBAL_LOSE_TIMEOUT_MS 即"从最后一帧算起"）。
         * 2026-11-20 新增：实测云台丢人后停在 239 度（离端点 28 度），
         * 条件 A 永远不成立，扫视一次都没触发过。 */
        if (ctrl->sweep_lost_ms > (uint16_t)(0xFFFFU - GIMBAL_CONTROL_PERIOD_MS)) {
            ctrl->sweep_lost_ms = 0xFFFFU;
        } else {
            ctrl->sweep_lost_ms =
                (uint16_t)(ctrl->sweep_lost_ms + GIMBAL_CONTROL_PERIOD_MS);
        }

        if ((ctrl->sweep_edge_ms >= GIMBAL_SWEEP_CONFIRM_MS) ||
            (((uint32_t)ctrl->sweep_lost_ms + GIMBAL_LOSE_TIMEOUT_MS) >=
             (uint32_t)GIMBAL_SWEEP_TRIGGER_MS)) {
            /* 方向：优先延续丢失前的转动方向（人往哪边走就从哪边找）；
             * 之前没在转就朝"离得远的那一端"扫，避免立刻折返。 */
            if (ctrl->sweep_dir == 0) {
                if (ctrl->last_turn_dir != 0) {
                    ctrl->sweep_dir = ctrl->last_turn_dir;
                } else {
                    ctrl->sweep_dir =
                        ((ctrl->yaw_angle_deg - ctrl->yaw_min_deg) <
                         (ctrl->yaw_max_deg - ctrl->yaw_angle_deg)) ? 1 : -1;
                }
            }
            /* 已经顶在某一端时只能往行程内侧走 */
            if (ctrl->yaw_angle_deg >= (ctrl->yaw_max_deg - GIMBAL_SWEEP_EDGE_DEG)) {
                ctrl->sweep_dir = -1;
            } else if (ctrl->yaw_angle_deg <= (ctrl->yaw_min_deg + GIMBAL_SWEEP_EDGE_DEG)) {
                ctrl->sweep_dir = 1;
            }

            ctrl->sweep_state = 1U;
            ctrl->sweep_rounds = 0U;
            ctrl->sweep_edge_ms = 0U;
            ctrl->sweep_lost_ms = 0U;
            return;
        }
        return;
    }

    /* ---------------------------- 扫视中 ---------------------------- */
    ctrl->yaw_angle_deg += ((float)ctrl->sweep_dir * step);
    ctrl->yaw_angle_deg = GimbalController_Clamp(ctrl->yaw_angle_deg,
                                                 ctrl->yaw_min_deg, ctrl->yaw_max_deg);

    /* 到达另一端（或已顶到端点）：算一个单程，折回 */
    if ((ctrl->yaw_angle_deg <= ctrl->yaw_min_deg) ||
        (ctrl->yaw_angle_deg >= ctrl->yaw_max_deg)) {
        ctrl->sweep_rounds++;
        ctrl->sweep_dir = (int8_t)(-ctrl->sweep_dir);

        if (ctrl->sweep_rounds >= GIMBAL_SWEEP_PASSES) {
            /* 扫够次数仍没看到人：回中点放弃，避免一直耗电/顶板 */
            ctrl->yaw_angle_deg = GimbalController_Clamp(GIMBAL_YAW_HOME_DEG,
                                                         ctrl->yaw_min_deg,
                                                         ctrl->yaw_max_deg);
            ctrl->sweep_state = 0U;
            ctrl->sweep_dir = 0;
            ctrl->sweep_rounds = 0U;
            ctrl->sweep_edge_ms = 0U;
        }
    }
}

void GimbalController_Update(GimbalController_t *ctrl, const VisionFrame_t *frame)
{
    float error_x;
    float error_y;
    float u_x;
    float u_y;

    if (ctrl == NULL) {
        return;
    }

    /* 上电计时（用于上电静默窗口）+ 记录摄像头是否已经在上发数据 */
    if (ctrl->boot_ms < 0xFFFFFFFFUL) {
        ctrl->boot_ms += (uint32_t)(ctrl->dt_s * 1000.0f + 0.5f);
    }
    if ((ctrl->vision_seen == 0U) && (Vision_HasReceivedFrame() != 0U)) {
        ctrl->vision_seen = 1U;
    }

    /* ---------- 1. 目标是否还在 ----------
     * 丢失时保持当前角度不动：不累加角度、不盲目乱转。
     * 这样人短暂遮挡后重新出现，云台还在原位，能立刻接上。
     *
     * 例外（2026-11-19）：如果云台正顶在行程端点（人已经转到看不到的方位），
     * 保持不动就永远等不到人 —— 这时改成"顶到限位就往反方向扫"。 */
    if (Vision_IsTargetFresh((uint16_t)GIMBAL_LOSE_TIMEOUT_MS) == 0U || frame == NULL) {
        ctrl->tracking = 0U;
        /* 丢人时清掉未发出的零头，避免重新捕获的瞬间把攒的旧量一次性放出去 */
        ctrl->pending_yaw = 0.0f;
        ctrl->pending_pitch = 0.0f;
        GimbalController_SweepStep(ctrl);
        GimbalController_ApplyOutput(ctrl);
        return;
    }
    ctrl->tracking = 1U;
    ctrl->target_seen = 1U;         /* 曾经跟踪到人 -> 以后才允许扫视 */
    ctrl->sweep_lost_ms = 0U;       /* 重新看到人：丢人计时清零 */

    /* 重新看到人：立刻退出扫视并清掉历史，避免带着扫视期间的状态进 PID */
    if (ctrl->sweep_state != 0U) {
        ctrl->sweep_state = 0U;
        ctrl->sweep_dir = 0;
        ctrl->sweep_rounds = 0U;
        ctrl->sweep_edge_ms = 0U;
        ctrl->integ_x = 0.0f;
        ctrl->integ_y = 0.0f;
        ctrl->last_error_x = 0.0f;
        ctrl->last_error_y = 0.0f;
    }

    /* ---------- 2. 低通滤波 ----------
     * K230 约 30 ms 一帧、控制 10 ms 一拍，同一个偏移值会被连续用 3 拍。
     * 不滤波的话云台每 3 拍跳一次（33 Hz 台阶感）；低通把这些台阶抹平。 */
    ctrl->filt_x += ((float)frame->x_norm - ctrl->filt_x) * ctrl->lpf_alpha;
    ctrl->filt_y += ((float)frame->y_norm - ctrl->filt_y) * ctrl->lpf_alpha;

    /* ---------- 3. 方向符号 ---------- */
    error_x = ctrl->sign_x * ctrl->filt_x;
    error_y = ctrl->sign_y * ctrl->filt_y;

    /* ---------- 4. 死区 ----------
     * 目标静止时检测框会抖 ±十几像素，死区把它压成 0，云台完全不动。 */
    error_x = GimbalController_ApplyDeadZone(error_x, ctrl->dead_zone);
    error_y = GimbalController_ApplyDeadZone(error_y, ctrl->dead_zone);

    /* ---------- 5. 位置式 PID ----------
     * 注意 error 是「扣掉死区之后」的偏差：死区内 error = 0，
     * 积分不再累加、微分项也为 0，PID 输出恒为 0 -> 云台不动。 */
    ctrl->integ_x += error_x;
    ctrl->integ_y += error_y;

    u_x = ctrl->kp * error_x
        + ctrl->ki * ctrl->integ_x
        + ctrl->kd * (error_x - ctrl->last_error_x) / ctrl->dt_s;

    u_y = ctrl->kp * error_y
        + ctrl->ki * ctrl->integ_y
        + ctrl->kd * (error_y - ctrl->last_error_y) / ctrl->dt_s;

    ctrl->last_error_x = error_x;
    ctrl->last_error_y = error_y;

    /* ---------- 6. 角度累加 ----------
     * 画面 x（水平）-> yaw（左右旋转，270 度）
     * 画面 y（垂直）-> pitch（上下俯仰，180 度） */
    /* 速率限幅：把每拍的角度增量夹住（2026-11-18 新增）。
     * 不加这一层时，K230 一换框，u_x 会给出十几度的增量，云台瞬间甩头，
     * 底盘收到的偏角也就跟着突跳，表现为小车「突然一个大转向」。 */
    {
        float dyaw = u_x * ctrl->deg_per_unit_x;
        float dpitch = u_y * ctrl->deg_per_unit_y;
        const float yaw_step = GIMBAL_YAW_RATE_MAX_DEG_S * ctrl->dt_s;
        const float pitch_step = GIMBAL_PITCH_RATE_MAX_DEG_S * ctrl->dt_s;

        /* 输出死区（累积式，2026-11-23 新增，见 config 里 GIMBAL_OUT_DEAD_ZONE_DEG）
         * 小于阈值的增量先攒着，攒够一次再发：
         *   抖动（正负交替）永远攒不够 -> 云台不动；
         *   真实偏差（同向）攒够就发 -> 不留稳态误差。 */
        if (GIMBAL_OUT_DEAD_ZONE_DEG > 0.0f) {
            ctrl->pending_yaw += dyaw;
            if ((ctrl->pending_yaw >= GIMBAL_OUT_DEAD_ZONE_DEG) ||
                (ctrl->pending_yaw <= -GIMBAL_OUT_DEAD_ZONE_DEG)) {
                dyaw = ctrl->pending_yaw;
                ctrl->pending_yaw = 0.0f;
            } else {
                dyaw = 0.0f;
            }

            ctrl->pending_pitch += dpitch;
            if ((ctrl->pending_pitch >= GIMBAL_OUT_DEAD_ZONE_DEG) ||
                (ctrl->pending_pitch <= -GIMBAL_OUT_DEAD_ZONE_DEG)) {
                dpitch = ctrl->pending_pitch;
                ctrl->pending_pitch = 0.0f;
            } else {
                dpitch = 0.0f;
            }
        }

        if (dyaw > yaw_step) {
            dyaw = yaw_step;
        } else if (dyaw < -yaw_step) {
            dyaw = -yaw_step;
        }
        if (dpitch > pitch_step) {
            dpitch = pitch_step;
        } else if (dpitch < -pitch_step) {
            dpitch = -pitch_step;
        }

        /* 记下"丢失前最后往哪边转"，扫视起手方向优先沿用它 */
        if (dyaw > 0.001f) {
            ctrl->last_turn_dir = 1;
        } else if (dyaw < -0.001f) {
            ctrl->last_turn_dir = -1;
        }

        ctrl->yaw_angle_deg += dyaw;
        ctrl->pitch_angle_deg += dpitch;
    }

    /* ---------- 7. 行程限幅 ----------
     * 顶到边界就停住。当前 Ki = 0，积分不会继续累加，因此不存在
     * 「顶住限位还攒劲、松回来猛弹」的问题；若以后启用 Ki，
     * 必须在此处配合「输出饱和时停止积分」。 */
    ctrl->yaw_angle_deg = GimbalController_Clamp(ctrl->yaw_angle_deg,
                                                 ctrl->yaw_min_deg, ctrl->yaw_max_deg);
    ctrl->pitch_angle_deg = GimbalController_Clamp(ctrl->pitch_angle_deg,
                                                   ctrl->pitch_min_deg, ctrl->pitch_max_deg);

    /* ---------- 8. 输出 ---------- */
    GimbalController_ApplyOutput(ctrl);
}

/* -------------------------------------------------------------- 查询接口 -- */

float GimbalController_GetYawDeg(const GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return 0.0f;
    }

    return ctrl->yaw_angle_deg;
}

float GimbalController_GetPitchDeg(const GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return 0.0f;
    }

    return ctrl->pitch_angle_deg;
}

float GimbalController_GetFilteredX(const GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return 0.0f;
    }

    return ctrl->filt_x;
}

float GimbalController_GetFilteredY(const GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return 0.0f;
    }

    return ctrl->filt_y;
}

uint8_t GimbalController_IsTracking(const GimbalController_t *ctrl)
{
    if (ctrl == NULL) {
        return 0U;
    }

    return ctrl->tracking;
}

int16_t GimbalController_GetYawDevTenthDeg(const GimbalController_t *ctrl)
{
    float dev_tenth;

    if (ctrl == NULL) {
        return 0;
    }

    /* 相对「摄像头正对车头」的偏角，放大 10 倍转成 0.1 度单位。
     * 注意零点用的是 GIMBAL_CHASSIS_ZERO_DEG 而不是上电中点：
     * 摄像头装歪时两者不等，用中点会导致底盘以为人一直偏，车绕着人打转。 */
    dev_tenth = (ctrl->yaw_angle_deg - GIMBAL_CHASSIS_ZERO_DEG) * 10.0f;

    /* 四舍五入（不用 lroundf，避免为一个转换把 math.h 拉进控制链） */
    if (dev_tenth >= 0.0f) {
        dev_tenth += 0.5f;
    } else {
        dev_tenth -= 0.5f;
    }

    /* 行程最大 ±132 度 = ±1320，int16 装得下，但仍做一次保护 */
    if (dev_tenth > 32767.0f) {
        return 32767;
    }
    if (dev_tenth < -32768.0f) {
        return -32768;
    }

    return (int16_t)dev_tenth;
}
