#ifndef BSP_UART_LINK_H
#define BSP_UART_LINK_H

/* ============================================================================
 * BSP：双板链路串口（USART1，PA9 发 / PA10 收，双向）
 *
 * 发送方向（PA9 -> 循迹板 PA10）：13 字节二进制帧，type = 0x02，
 *   内容 = 云台 yaw 偏差 + 是否检测到人，供模式 2 转向使用。
 * 接收方向（PA10 <- 循迹板 PA9）：循迹板每 500 ms 一行的 ASCII 调试文本，
 *   例如 "C: mode=1 ... st=9 sw=0 dage=55 oled=1"。
 *   本板不做解析，只做「原样转发给蓝牙」，见 bsp_bt_uart.h。
 *
 * 为什么接收必须用中断：115200 下一个字节只占 87 us，而主循环里有 OLED 刷屏
 * （几毫秒）和 Delay_ms，轮询一定丢字节。
 *
 * 层次约定：本文件是云台板上唯一允许操作 USART1 的地方。
 * ========================================================================== */

#include <stdint.h>

/* 初始化 USART1：PA9(TX) 复用推挽 + PA10(RX) 浮空输入，115200-8-N-1，开 RXNE 中断 */
void BSP_Link_Init(void);

/**
 * @brief 阻塞发送一段字节（用于 13 字节帧，约 1.1 ms）。
 * @param data 数据指针；为 NULL 或 len 为 0 时直接返回。
 * @param len  字节数。
 *
 * @note 阻塞式（每字节等 TXE）。调用方必须节流，不能放进 10 ms 控制环的每一拍。
 */
void BSP_Link_SendBuffer(const uint8_t *data, uint16_t len);

/**
 * @brief 接收缓冲里还有多少字节未读。
 * @return 字节数（0 = 空）。
 */
uint16_t BSP_Link_GetRxCount(void);

/**
 * @brief 读一个字节并出队。
 * @return 数据；缓冲为空时返回 0xFF（调用方先看 GetRxCount）。
 */
uint8_t BSP_Link_ReadByte(void);

#endif /* BSP_UART_LINK_H */
