#ifndef BSP_UART_H
#define BSP_UART_H

/* ============================================================================
 * BSP：K230 视觉串口（USART2，中断接收 + 环形缓冲）
 *
 * 层次约定：本文件是唯一允许操作 USART2 的地方。
 *          modules/vision_protocol.c 只通过下面这组接口取字节，
 *          不知道底层用的是哪个 USART、也没有任何寄存器访问。
 *
 * 为什么必须中断接收：主循环里有 Delay_ms 和 OLED 刷屏（共几毫秒），
 * 115200 下一个字节只要 87 us，轮询会大面积丢字节。
 * ========================================================================== */

#include <stdint.h>

/* 初始化 USART2：PA2 = TX，PA3 = RX，115200-8-N-1，开启 RXNE 中断 */
void BSP_UART_Init(void);

/* 发送：阻塞式，等 TXE 标志。只用于调试回传，不要在控制高频路径里大量调用 */
void BSP_UART_SendByte(uint8_t byte);
void BSP_UART_SendString(const char *text);

/**
 * @brief 当前接收缓冲里还有多少字节未读。
 * @return 字节数（0 表示空）。
 */
uint16_t BSP_UART_GetRxCount(void);

/**
 * @brief 查看队列里第 offset 个字节，但不出队。
 *
 * @param offset 0 = 队首。
 * @param byte   取到数据时写入；可传 0 只做存在性判断。
 * @return 1 = 取到；0 = 缓冲里没有这么多字节。
 *
 * @note 用于协议解析的「滑窗重同步」：先 Peek 判断帧头，确认后再 Skip 出队。
 */
uint8_t BSP_UART_PeekAt(uint16_t offset, uint8_t *byte);

/**
 * @brief 从队首丢弃 count 个字节。
 * @param count 要丢弃的数量；超过现有字节数时按实际数量丢。
 */
void BSP_UART_Skip(uint16_t count);

/**
 * @brief 读一个字节并出队。
 * @return 数据；缓冲为空时返回 0xFF（调用方应先用 GetRxCount 判断）。
 */
uint8_t BSP_UART_ReadByte(void);

#endif /* BSP_UART_H */
