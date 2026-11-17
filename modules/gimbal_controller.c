#include "gimbal_controller.h"

#include <stddef.h>

#include "bsp_servo_pwm.h"
#include "gimbal_config.h"
#include "vision_config.h"

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
    ctrl->tracking = 0U;

    /* 角度先夹一次，保证 config 里写错行程也不会越界 */
    ctrl->yaw_angle_deg = GimbalController_Clamp(ctrl->yaw_angle_deg,
                                                 ctrl->yaw_min_deg, ctrl->yaw_max_deg);
    ctrl->pitch_angle_deg = GimbalController_Clamp(ctrl->pitch_angle_deg,
                                                   ctrl->pitch_min_deg, ctrl->pitch_max_deg);

    GimbalController_ApplyOutput(ctrl);
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

void GimbalController_Update(GimbalController_t *ctrl, const VisionFrame_t *frame)
{
    float error_x;
    float error_y;
    float u_x;
    float u_y;

    if (ctrl == NULL) {
        return;
    }

    /* ---------- 1. 目标是否还在 ----------
     * 丢失时保持当前角度不动：不累加角度、不盲目乱转。
     * 这样人短暂遮挡后重新出现，云台还在原位，能立刻接上。 */
    if (Vision_IsTargetFresh((uint16_t)GIMBAL_LOSE_TIMEOUT_MS) == 0U || frame == NULL) {
        ctrl->tracking = 0U;
        GimbalController_ApplyOutput(ctrl);
        return;
    }
    ctrl->tracking = 1U;

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
    ctrl->yaw_angle_deg += u_x * ctrl->deg_per_unit_x;
    ctrl->pitch_angle_deg += u_y * ctrl->deg_per_unit_y;

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
