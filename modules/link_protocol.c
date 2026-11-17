#include "link_protocol.h"

#include <stddef.h>

#include "bsp_uart_link.h"

/* 帧结构与常量：必须与循迹板 modules/vision_link.c 以及
 * docs/双板分工与协议.md 三处保持一致。 */
#define LINK_FRAME_LEN          13U
#define LINK_HEAD_0             0xAAU
#define LINK_HEAD_1             0x55U
#define LINK_TYPE               0x02U
#define LINK_LENGTH             0x08U
#define LINK_CHECKSUM_INDEX     12U

#define LINK_FLAG_DETECTED      0x01U

/* 大端写入：int16 / uint16 */
static void LinkProtocol_WriteInt16(uint8_t *buf, uint16_t offset, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    buf[offset] = (uint8_t)(raw >> 8);
    buf[offset + 1U] = (uint8_t)(raw & 0xFFU);
}

static void LinkProtocol_WriteUint16(uint8_t *buf, uint16_t offset, uint16_t value)
{
    buf[offset] = (uint8_t)(value >> 8);
    buf[offset + 1U] = (uint8_t)(value & 0xFFU);
}

void LinkProtocol_Init(void)
{
    BSP_Link_Init();
}

void LinkProtocol_Send(const LinkPayload_t *payload)
{
    uint8_t frame[LINK_FRAME_LEN];
    uint8_t checksum = 0U;
    uint8_t i;

    frame[0] = LINK_HEAD_0;
    frame[1] = LINK_HEAD_1;
    frame[2] = LINK_TYPE;
    frame[3] = LINK_LENGTH;

    if (payload != NULL) {
        LinkProtocol_WriteInt16(frame, 4U, payload->yaw_dev_tenth_deg);
        LinkProtocol_WriteInt16(frame, 6U, payload->offset_x);
        LinkProtocol_WriteUint16(frame, 8U, payload->distance_cm);
        frame[10] = (payload->detected != 0U) ? LINK_FLAG_DETECTED : 0U;
        /* 蓝牙命令码：原样透传，循迹板负责边沿检测（只在 0->非0 时动作） */
        frame[11] = payload->command;
    } else {
        /* 没有数据可发：偏角与偏差写 0，flags 不带 detected 位。
         * 注意仍然要发帧 —— 停发会让对端无法区分「没人」和「断线」。 */
        LinkProtocol_WriteInt16(frame, 4U, 0);
        LinkProtocol_WriteInt16(frame, 6U, 0);
        LinkProtocol_WriteUint16(frame, 8U, 0U);
        frame[10] = 0U;
        frame[11] = 0U;
    }

    for (i = 0U; i < LINK_CHECKSUM_INDEX; i++) {
        checksum ^= frame[i];
    }
    frame[LINK_CHECKSUM_INDEX] = checksum;

    BSP_Link_SendBuffer(frame, LINK_FRAME_LEN);
}
