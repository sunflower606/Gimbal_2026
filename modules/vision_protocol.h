#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

/* ============================================================================
 * Module：K230 视觉帧解析（13 字节定长帧 -> 归一化偏差）
 *
 * 数据流：K230 --UART--> bsp_uart 环形缓冲 --> 本模块解析成一帧帧数据
 *
 * 解析策略（移植自已验证的 Gimbal_1.0/Vision.c）：
 *   不依赖「一帧刚好一次收完」。每次 Poll 都在环形缓冲里滑窗扫描 4 字节帧头，
 *   凑够 13 字节就校验；校验不过只前进 1 字节重新找帧头（滑窗重同步），
 *   这样即使中间丢字节、多字节也能自动恢复。
 *
 * 解析出的像素坐标会顺手换算成 ±100 的归一化偏差，
 * 控制层（modules/gimbal_controller）直接使用归一化值，不必关心分辨率。
 *
 * 层次约定：本模块只依赖 bsp_uart 与 System/Delay，不碰任何寄存器。
 * ========================================================================== */

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 一帧视觉数据（已换算成控制层可用的形式）。
 *
 * 单位说明：
 *   x_px / y_px            原始像素坐标，单位 pixel
 *   offset_x_px / offset_y_px  相对画面中心的像素偏差，单位 pixel，右/下为正
 *   x_norm / y_norm        归一化偏差，无量纲，取值 -100 ~ +100（画面半边 = 100）
 */
typedef struct {
    int16_t x_px;
    int16_t y_px;
    int16_t offset_x_px;
    int16_t offset_y_px;
    int16_t x_norm;
    int16_t y_norm;
    uint8_t detected;           /* 1 = 本帧检测到人；0 = 本帧为丢失帧 */
    uint32_t frame_count;       /* 收到校验通过的帧数（丢失帧也计入） */
    uint32_t lost_frame_count;  /* 其中「未检测到人」的帧数 */
    uint32_t checksum_error_count;
    uint32_t last_ms;           /* 最近一次收到合法帧的时刻，ms */
} VisionFrame_t;

/* 初始化：清状态 + 初始化串口 BSP */
void Vision_Init(void);

/* 主循环每周期调用：把缓冲里所有完整帧解析出来，只保留最新一帧 */
void Vision_Poll(void);

/** @brief 取最近一帧数据的快照（值拷贝，改返回值不影响内部状态）。 */
VisionFrame_t Vision_GetFrame(void);

/**
 * @brief 判断最近一帧是否「新鲜且确实检测到人」。
 * @param timeout_ms 允许的最大数据龄，单位 ms。
 * @return 1 = 可用；0 = 丢失（未检测到人 / 从未收到 / 超时）。
 */
uint8_t Vision_IsTargetFresh(uint16_t timeout_ms);

/**
 * @brief 是否收到过至少一帧校验通过的帧（不管有没有人）。
 * @return 1 = 摄像头已经在上发数据；0 = 从未收到（摄像头没上电/没启动完）。
 * @note 用于区分「摄像头还没起来」和「摄像头在工作但没看到人」。
 */
uint8_t Vision_HasReceivedFrame(void);

/**
 * @brief 通过调试串口打印当前帧与统计信息（阻塞发送，约 5 ms，不要在控制高频路径调用）。
 */
void Vision_PrintFrame(void);

#endif /* VISION_PROTOCOL_H */
