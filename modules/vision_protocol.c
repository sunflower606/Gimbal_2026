#include "vision_protocol.h"

#include <stdio.h>

#include "bsp_uart.h"
#include "Delay.h"
#include "vision_config.h"

/* 内部状态：当前帧 + 解析统计 */
static VisionFrame_t s_frame;

/* 大端 int16：K230 侧用 struct.pack(">hh", ...) 打包 */
static int16_t Vision_ReadInt16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* 像素偏差 -> 归一化 ±100。
 * 整数除法向零截断（-37.5 得 -37），带来不到 1 个单位的量化误差，对控制无影响。 */
static int16_t Vision_PxToNorm(int32_t offset_px, int32_t half_size)
{
    int32_t value = (offset_px * VISION_NORM_SCALE) / half_size;

    if (value > VISION_NORM_SCALE) {
        value = VISION_NORM_SCALE;
    }
    if (value < -VISION_NORM_SCALE) {
        value = -VISION_NORM_SCALE;
    }

    return (int16_t)value;
}

/* 从 offset 处开始是不是完整的 4 字节帧头 */
static uint8_t Vision_IsHeaderAt(uint16_t offset)
{
    uint8_t byte = 0U;

    if (BSP_UART_PeekAt(offset, &byte) == 0U || byte != VISION_HEAD_0) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 1U), &byte) == 0U || byte != VISION_HEAD_1) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 2U), &byte) == 0U || byte != VISION_HEAD_2) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 3U), &byte) == 0U || byte != VISION_HEAD_3) {
        return 0U;
    }

    return 1U;
}

void Vision_Init(void)
{
    s_frame.x_px = 0;
    s_frame.y_px = 0;
    s_frame.offset_x_px = 0;
    s_frame.offset_y_px = 0;
    s_frame.x_norm = 0;
    s_frame.y_norm = 0;
    s_frame.detected = 0U;
    s_frame.frame_count = 0U;
    s_frame.lost_frame_count = 0U;
    s_frame.checksum_error_count = 0U;
    s_frame.last_ms = 0U;

    BSP_UART_Init();
}

void Vision_Poll(void)
{
    uint16_t available;
    uint16_t position;
    uint8_t raw[VISION_FRAME_LEN];

    for (;;) {
        available = BSP_UART_GetRxCount();

        /* 凑不齐一整帧，等下次 */
        if (available < VISION_FRAME_LEN) {
            return;
        }

        /* 找帧头：滑窗扫描，失配只前进 1 字节 */
        position = 0U;
        while ((uint16_t)(position + VISION_FRAME_LEN) <= available) {
            if (Vision_IsHeaderAt(position) != 0U) {
                break;
            }
            position++;
        }

        /* 没找到帧头：丢掉前面的垃圾，但保留末尾 3 字节，
           防止把恰好跨在本次缓冲末尾的半个帧头丢掉 */
        if ((uint16_t)(position + VISION_FRAME_LEN) > available) {
            uint16_t drop = available;

            if (drop > (VISION_HEAD_LEN - 1U)) {
                drop = (uint16_t)(drop - (VISION_HEAD_LEN - 1U));
            }

            BSP_UART_Skip(drop);
            return;
        }

        /* 丢弃帧头之前的字节 */
        if (position > 0U) {
            BSP_UART_Skip(position);
        }

        /* 取出整帧（此时队首就是帧头） */
        {
            uint16_t i;

            for (i = 0U; i < VISION_FRAME_LEN; i++) {
                if (BSP_UART_PeekAt(i, &raw[i]) == 0U) {
                    return;             /* 数据不够，等下次 */
                }
            }
        }

        /* 异或校验：前 12 字节逐字节异或 */
        {
            uint8_t checksum = 0U;
            uint8_t i;

            for (i = 0U; i < VISION_CHECKSUM_INDEX; i++) {
                checksum ^= raw[i];
            }

            if (checksum != raw[VISION_CHECKSUM_INDEX]) {
                /* 校验失败：只丢 1 字节重新同步（可能是半个残帧） */
                s_frame.checksum_error_count++;
                BSP_UART_Skip(1U);
                continue;
            }
        }

        /* 校验通过，整帧出队 */
        BSP_UART_Skip(VISION_FRAME_LEN);

        {
            int16_t x_px = Vision_ReadInt16Be(&raw[VISION_X_INDEX]);
            int16_t y_px = Vision_ReadInt16Be(&raw[VISION_Y_INDEX]);

            /* 协议用负值表示「这一帧没检测到人」 */
            if (x_px < 0 || y_px < 0) {
                s_frame.detected = 0U;
                s_frame.x_norm = 0;
                s_frame.y_norm = 0;
                s_frame.lost_frame_count++;
            } else {
                int32_t offset_x = (int32_t)x_px - (VISION_IMAGE_WIDTH_PX / 2);
                int32_t offset_y = (int32_t)y_px - (VISION_IMAGE_HEIGHT_PX / 2);

                s_frame.x_px = x_px;
                s_frame.y_px = y_px;
                s_frame.offset_x_px = (int16_t)offset_x;
                s_frame.offset_y_px = (int16_t)offset_y;
                s_frame.x_norm = Vision_PxToNorm(offset_x, VISION_IMAGE_WIDTH_PX / 2);
                s_frame.y_norm = Vision_PxToNorm(offset_y, VISION_IMAGE_HEIGHT_PX / 2);
                s_frame.detected = 1U;
            }

            /* 注意：frame_count / last_ms 对丢失帧也更新 —— 它们表示「链路是否有数据」，
               而不是「有没有人」。判断有没有人必须看 detected。 */
            s_frame.frame_count++;
            s_frame.last_ms = Delay_GetMs();
        }
    }
}

VisionFrame_t Vision_GetFrame(void)
{
    return s_frame;
}

uint8_t Vision_IsTargetFresh(uint16_t timeout_ms)
{
    /* 先看有没有人：K230 未检测到人时会主动发丢失帧，这一条立即生效 */
    if (s_frame.detected == 0U) {
        return 0U;
    }

    /* 一帧都没收到过 */
    if (s_frame.frame_count == 0U) {
        return 0U;
    }

    /* 再看数据龄：覆盖「K230 断线/死机，一个字节都没有」的情况 */
    if ((uint32_t)(Delay_GetMs() - s_frame.last_ms) > (uint32_t)timeout_ms) {
        return 0U;
    }

    return 1U;
}

void Vision_PrintFrame(void)
{
    char buffer[128];

    (void)sprintf(buffer,
                  "V: x=%d y=%d off=%d/%d det=%d n=%lu err=%lu lost=%lu\r\n",
                  (int)s_frame.x_px,
                  (int)s_frame.y_px,
                  (int)s_frame.offset_x_px,
                  (int)s_frame.offset_y_px,
                  (int)s_frame.detected,
                  (unsigned long)s_frame.frame_count,
                  (unsigned long)s_frame.checksum_error_count,
                  (unsigned long)s_frame.lost_frame_count);

    BSP_UART_SendString(buffer);
}
