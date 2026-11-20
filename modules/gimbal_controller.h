#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

/* ============================================================================
 * Module：云台控制律（位置式 PID + 低通 + 死区 + 行程限幅）
 *
 * 控制律来自已完成整定的 Gimbal_1.0 工程，**数学与参数均未改动**，
 * 只做了重命名与分层。原工程的文件名/变量名与物理轴的对应关系是错位的
 * （angle_on 其实控制左右旋转、angle_down 控制上下俯仰），迁移时统一改成
 * 按物理轴命名，彻底消除歧义：
 *
 *     yaw   = 左右旋转，270 度舵机，TIM1_CH4 -> PA11
 *     pitch = 上下俯仰，180 度舵机，TIM1_CH1 -> PA8
 *
 * 画面坐标与轴的对应：
 *     画面 x（水平偏差）-> yaw
 *     画面 y（垂直偏差）-> pitch
 *
 * 每拍的计算（单位均为「度」）：
 *     1) 一阶低通：filt += (raw - filt) × lpf_alpha
 *     2) 方向符号：error = sign × filt
 *     3) 软死区：|error| <= dead_zone 时置 0，否则扣掉死区那一段
 *     4) 积分累加：integ += error
 *     5) 位置式 PID：u = Kp*error + Ki*integ + Kd*(error - last_error)/dt
 *     6) 角度累加：angle += u × deg_per_unit
 *     7) 行程限幅：angle 夹到 [min, max]
 *
 * 目标丢失时：保持当前角度不变，不累加、不乱转。
 * ========================================================================== */

#include <stdbool.h>
#include <stdint.h>

#include "vision_protocol.h"

/** @brief 云台控制器状态与参数。 */
typedef struct {
    /* ---------------- 可调参数（Init 时由 config/ 宏填入） ---------------- */
    float kp;                   /* 比例增益，无量纲 */
    float ki;                   /* 积分增益，无量纲 */
    float kd;                   /* 微分增益，单位 s（公式里再除以 dt_s） */
    float dt_s;                 /* 控制周期，单位 s */
    float lpf_alpha;            /* 坐标低通系数，0~1 */
    float dead_zone;            /* 死区，单位：归一化偏差 */
    float sign_x;               /* 水平方向符号，+1 或 -1 */
    float sign_y;               /* 垂直方向符号，+1 或 -1 */
    float deg_per_unit_x;       /* 归一化偏差 -> 度（yaw），单位 度/单位 */
    float deg_per_unit_y;       /* 归一化偏差 -> 度（pitch），单位 度/单位 */
    float yaw_min_deg;          /* yaw 行程下限，单位 度 */
    float yaw_max_deg;          /* yaw 行程上限，单位 度 */
    float pitch_min_deg;        /* pitch 行程下限，单位 度 */
    float pitch_max_deg;        /* pitch 行程上限，单位 度 */

    /* ------------------------------- 运行状态 ------------------------------- */
    float yaw_angle_deg;        /* 当前 yaw 指令角，单位 度 */
    float pitch_angle_deg;      /* 当前 pitch 指令角，单位 度 */
    float filt_x;               /* 低通后的水平偏差，单位：归一化偏差 */
    float filt_y;               /* 低通后的垂直偏差，单位：归一化偏差 */
    float integ_x;              /* 水平积分累加，单位：归一化偏差 */
    float integ_y;              /* 垂直积分累加，单位：归一化偏差 */
    float last_error_x;         /* 上一拍水平偏差（已过死区） */
    float last_error_y;         /* 上一拍垂直偏差（已过死区） */
    uint8_t tracking;           /* 1 = 正在跟踪；0 = 目标丢失 */

    /* ------------------------- 限位反向扫视（2026-11-19，模式 1 兜底） ----
     * 目标丢失且云台顶在行程端点时，以固定角速度朝反方向扫，直到重新看到人。
     * 参数见 config/gimbal_config.h 的 GIMBAL_SWEEP_*。 */
    uint8_t sweep_state;        /* 0 = 未扫视；1 = 扫视中 */
    int8_t sweep_dir;           /* +1 = 朝 yaw 增大方向；-1 = 朝减小方向 */
    uint8_t sweep_rounds;       /* 已完成的单程数 */
    uint16_t sweep_edge_ms;     /* 连续贴限位的累计时间，单位 ms */
    uint16_t sweep_lost_ms;     /* 判丢后累计时间，单位 ms（兜底触发用） */
    int8_t last_turn_dir;       /* 丢失前最后一次转动方向：+1/-1/0 */
    uint32_t boot_ms;           /* 上电后累计运行时间，ms（上电静默用） */
    uint8_t vision_seen;        /* 1 = 收到过至少一帧（摄像头已上线） */
    uint8_t target_seen;        /* 1 = 曾经成功跟踪到人（扫视前置条件） */
} GimbalController_t;

/**
 * @brief 初始化控制器：填入 config/ 里的默认参数，清零状态，并把云台摆到行程中点。
 * @param ctrl 控制器实例；传 NULL 时直接返回。
 */
void GimbalController_Init(GimbalController_t *ctrl);

/**
 * @brief 用一帧视觉数据更新一次云台，并直接输出 PWM。
 * @param ctrl  控制器实例。
 * @param frame 视觉帧快照；传 NULL 时按「目标丢失」处理（保持角度）。
 *
 * @note 目标是否新鲜由本函数内部调用 Vision_IsTargetFresh 判断，
 *       调用方只需把最新帧传进来。
 */
void GimbalController_Update(GimbalController_t *ctrl, const VisionFrame_t *frame);

/**
 * @brief 把云台直接摆到指定角度（上电回中/手动摆位）。
 * @param yaw_deg   目标 yaw 角，单位 度，超出行程会被夹住。
 * @param pitch_deg 目标 pitch 角，单位 度，超出行程会被夹住。
 */
void GimbalController_SetAngle(GimbalController_t *ctrl, float yaw_deg, float pitch_deg);

/** @brief 按当前角度输出一次 PWM（不改变状态）。 */
void GimbalController_Output(const GimbalController_t *ctrl);

/* ------------------------------- 查询接口 ------------------------------- */
float GimbalController_GetYawDeg(const GimbalController_t *ctrl);
float GimbalController_GetPitchDeg(const GimbalController_t *ctrl);
float GimbalController_GetFilteredX(const GimbalController_t *ctrl);
float GimbalController_GetFilteredY(const GimbalController_t *ctrl);
uint8_t GimbalController_IsTracking(const GimbalController_t *ctrl);

/**
 * @brief 当前 yaw 相对上电中点（正前方）的偏角，单位 0.1 度。
 *
 * 这是发给循迹板做模式 2 转向的信号。
 * 为什么不用画面偏差：云台一直在跟踪人，人始终在画面中心，
 * 所以画面偏差恒接近 0，**不携带方向信息**；
 * 而「云台转了多大角度去对准人」正好编码了人相对车头的方位。
 *
 * @return 偏角，单位 0.1 度；例如 300 表示向右（或向左，取决于舵机装配）30.0 度。
 *         取值约 -1320 ~ +1320（行程 3~267 度，中点 135 度）。
 */
int16_t GimbalController_GetYawDevTenthDeg(const GimbalController_t *ctrl);

#endif /* GIMBAL_CONTROLLER_H */
