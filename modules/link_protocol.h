#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

/* ============================================================================
 * Module：云台板 -> 循迹板 数据帧（type = 0x02）
 *
 * 用途：把「云台 yaw 偏角」发给循迹板，供模式 2（纯人体跟随）转向使用。
 *
 * 为什么发 yaw 偏角而不是画面偏差：
 *   云台一直在跟踪人，人始终被保持在画面中心，画面偏差恒接近 0，
 *   不携带方向信息。而「云台转了多少度去对准人」正好编码了
 *   人相对车头的方位 —— 车头转向人的方向，云台就会转回去，
 *   形成天然收敛的负反馈。
 *
 * 帧格式（13 字节，大端序，与 K230 帧同结构，只有 type 不同）：
 *   0      head0                0xAA
 *   1      head1                0x55
 *   2      type                 0x02
 *   3      length               0x08（负载长度）
 *   4-5    yaw_dev_tenth_deg    int16，云台 yaw 相对中点偏角，单位 0.1 度
 *   6-7    offset_x             int16，画面偏差 -100~+100（诊断用）
 *   8-9    distance_cm          uint16，0 = 未知（预留，将来接测距）
 *   10     flags                uint8，bit0 = 检测到人
 *   11     reserved             uint8，0
 *   12     checksum             uint8，前 12 字节逐字节异或
 * ========================================================================== */

#include <stdint.h>

/** @brief 一帧要发送的负载内容。 */
typedef struct {
    int16_t yaw_dev_tenth_deg;  /* 云台 yaw 相对中点偏角，单位 0.1 度 */
    int16_t offset_x;           /* 画面水平偏差，-100 ~ +100 */
    uint16_t distance_cm;       /* 距离，单位 cm；0 = 未知 */
    uint8_t detected;           /* 1 = 检测到人 */
} LinkPayload_t;

/* 初始化链路的 BSP 串口 */
void LinkProtocol_Init(void);

/**
 * @brief 组帧并发送一帧。
 * @param payload 负载内容；传 NULL 时按「无人」发送（flags = 0）。
 *
 * @note 阻塞发送约 1.1 ms（13 字节 @115200），调用方必须节流。
 */
void LinkProtocol_Send(const LinkPayload_t *payload);

#endif /* LINK_PROTOCOL_H */
