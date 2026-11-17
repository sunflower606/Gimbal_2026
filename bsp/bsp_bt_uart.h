#ifndef BSP_BT_UART_H
#define BSP_BT_UART_H

/* ============================================================================
 * BSP：蓝牙透传串口（USART3，PB10 = TX / PB11 = RX，115200-8-N-1）
 *
 * 硬件：JDY-31（经典蓝牙 SPP 透传模块）。PB10 -> 模块 RXD，PB11 <- 模块 TXD，
 *       5V 供电、GND 共地。模块波特率必须先用 AT+BAUD8 改成 115200 再断电重启。
 *
 * 数据流：循迹板 PA9 --(USART1 RX)--> 云台板 --(本文件 USART3 TX)--> 蓝牙 --> 电脑
 *         本板只是「原样转发」，不解析、不改写、不加时间戳，电脑看到的就是
 *         循迹板原始的那行 "C: ..."。
 *
 * 为什么发送走中断而不是阻塞：一行约 110 字节，@115200 阻塞发送要 9.5 ms，
 * 而云台控制环周期只有 10 ms —— 阻塞会让舵机晚一拍、肉眼可见抖动。
 * 这里主循环只负责入队（几十纳秒），搬运交给 TXE 中断。
 * ========================================================================== */

#include <stdint.h>

/* 初始化 USART3：PB10 复用推挽 + PB11 浮空输入，开 RXNE / TXE 中断 */
void BSP_BT_Init(void);

/**
 * @brief 把一段字节放进发送队列（非阻塞）。
 * @param data 数据指针；NULL 或 len 为 0 时直接返回。
 * @param len  字节数。
 * @return 实际入队的字节数；小于 len 说明队列满、多出来的被丢弃。
 *
 * @note 丢弃而不是阻塞：蓝牙没连上时（没人收）宁可丢日志，也不能卡住控制环。
 */
uint16_t BSP_BT_SendBuffer(const uint8_t *data, uint16_t len);

/* 放一个字节进发送队列；返回 1 = 成功，0 = 队列满。 */
uint8_t BSP_BT_SendByte(uint8_t byte);

/* 发送队列里还有多少字节没发出去（诊断用：一直涨说明蓝牙没在收）。 */
uint16_t BSP_BT_GetTxPending(void);

/* 接收队列里还有多少字节（手机发来的命令文本）。 */
uint16_t BSP_BT_GetRxCount(void);

/**
 * @brief 从接收队列取一个字节。
 * @param byte 输出；成功时写入取到的字节。
 * @return 1 = 取到；0 = 队列空。
 *
 * @note 由 modules/bt_command.c 逐字节取出拼行，再把 mode1/mode2/stop
 *       转成链路帧里的命令位发给循迹板。
 */
uint8_t BSP_BT_ReadByte(uint8_t *byte);

#endif /* BSP_BT_UART_H */
