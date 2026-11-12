#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include "gimbal_config.h"  /* VISION_TIMEOUT_MS 复用 GIMBAL_LOSE_TIMEOUT_MS，保证两侧一致 */

/* ============================================================================
 * K230 视觉串口协议配置
 *
 * 协议：13 字节定长帧，大端序，与 K230 侧 person_detection_gimbal.py 的
 *       GimbalLink.send() 严格对应（该脚本随工程归档在 k230/ 目录）。
 *
 *   字节 0-3    帧头        0xAA 0x55 0x01 0x08
 *   字节 4-5    x           int16 大端，人体框中心像素 X；负值表示丢失
 *   字节 6-7    y           int16 大端，人体框中心像素 Y；负值表示丢失
 *   字节 8-9    reserved    uint16，固定 0
 *   字节 10-11  reserved    uint16，固定 0
 *   字节 12     checksum    前 12 字节逐字节异或
 *
 * 坐标系：检测画面像素坐标，原点左上角，x 向右增、y 向下增。
 * ========================================================================== */

/* ---------------------------------------------------------------- 帧结构 */
#define VISION_FRAME_LEN                    13U
#define VISION_HEAD_LEN                     4U
#define VISION_CHECKSUM_INDEX               12U

#define VISION_HEAD_0                       0xAAU
#define VISION_HEAD_1                       0x55U
#define VISION_HEAD_2                       0x01U
#define VISION_HEAD_3                       0x08U

#define VISION_X_INDEX                      4U
#define VISION_Y_INDEX                      6U

/* 协议里用负值表示「这一帧没检测到人」 */
#define VISION_LOST_PX                      (-1)

/* ------------------------------------------------------------ 检测画面尺寸
 * 必须与 K230 脚本里的 rgb888p_size = [1280, 960] 一致。
 * 若 K230 侧改了分辨率，这里必须同步改，否则偏差会整体偏移。
 * ------------------------------------------------------------------------- */
#define VISION_IMAGE_WIDTH_PX               1280
#define VISION_IMAGE_HEIGHT_PX              960

/* 归一化偏差标度：画面半边 = 100（即偏差取值 -100 ~ +100） */
#define VISION_NORM_SCALE                   100

/* ---------------------------------------------------------------- 超时
 * 与 GIMBAL_LOSE_TIMEOUT_MS 保持一致（丢失判定用同一个超时）。
 * ------------------------------------------------------------------------- */
#define VISION_TIMEOUT_MS                   GIMBAL_LOSE_TIMEOUT_MS

#endif /* VISION_CONFIG_H */
