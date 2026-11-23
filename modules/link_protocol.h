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
 *   11     command              uint8，0 = 无命令 / 1 = mode1 / 2 = mode2 / 3 = stop
 *                               （手机经蓝牙发给云台板的文本命令，转成命令码后随帧下发；
 *                                 原来保留为 0，现在给命令用，长度不变）
 *   12     checksum             uint8，前 12 字节逐字节异或
 * ========================================================================== */

#include <stdint.h>

/** @brief 一帧要发送的负载内容。 */
typedef struct {
    int16_t yaw_dev_tenth_deg;  /* 云台 yaw 相对中点偏角，单位 0.1 度 */
    int16_t offset_x;           /* 画面水平偏差，-100 ~ +100 */
    uint16_t distance_cm;       /* 距离，单位 cm；0 = 未知 */
    uint8_t detected;           /* 1 = 检测到人 */
    uint8_t command;            /* 0 = 无命令；1/2/3 = mode1/mode2/stop（见 bt_command.h） */
} LinkPayload_t;

/* 帧类型：状态帧（现有）与调参请求/应答（新增） */
#define LINK_TYPE_STATUS      0x02U
#define LINK_TYPE_PARAM_SET   0x03U
#define LINK_TYPE_PARAM_ACK   0x04U

/* 调参操作码：写在 0x03 帧的第 8 字节 */
#define LINK_PARAM_OP_SET     0U    /* 改值 */
#define LINK_PARAM_OP_GET     1U    /* 只读回当前值 */
#define LINK_PARAM_OP_RESET   2U    /* 恢复编译期默认值（底盘回 id=0xFFFF） */
#define LINK_PARAM_OP_TOGGLE  3U    /* 切换 A2（参数存 Flash）开关，id 固定 13 */
#define LINK_PARAM_OP_SAVE    4U    /* 参数写进 Flash，结果在应答值里 */

/* 初始化链路的 BSP 串口 */
void LinkProtocol_Init(void);

/**
 * @brief 组帧并发送一帧。
 * @param payload 负载内容；传 NULL 时按「无人」发送（flags = 0）。
 *
 * @note 阻塞发送约 1.1 ms（13 字节 @115200），调用方必须节流。
 */
void LinkProtocol_Send(const LinkPayload_t *payload);

/**
 * @brief 发一帧调参请求（type = 0x03）到循迹板。
 * @param param_id 参数编号（0~63）
 * @param value    新值（OP_SET 时有效；OP_GET 时忽略）
 * @param op       LINK_PARAM_OP_SET / LINK_PARAM_OP_GET
 *
 * @note 非阻塞（走 TX 环缓），每帧 13 字节 @115200 约 1.1 ms。
 */
void LinkProtocol_SendParam(uint16_t param_id, int16_t value, uint8_t op);


#endif /* LINK_PROTOCOL_H */
