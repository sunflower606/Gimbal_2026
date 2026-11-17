#include "bsp_uart_link.h"

#include <stddef.h>

#include "board_config.h"

/* 接收环形缓冲：永远留一个空位区分「空」和「满」，
 * 实际可用 BOARD_LINK_RX_BUF_SIZE - 1 字节。满了丢弃新字节，保住未转发的数据。 */
static uint8_t s_rx_buf[BOARD_LINK_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;    /* 中断写 */
static volatile uint16_t s_rx_tail = 0U;    /* 主循环写 */

void BSP_Link_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB2PeriphClockCmd(BOARD_LINK_UART_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_LINK_GPIO_CLK, ENABLE);

    /* TX：复用推挽。写成普通推挽的话引脚由 GPIO 驱动，发不出波形。 */
    gpio_init.GPIO_Pin = BOARD_LINK_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_LINK_GPIO, &gpio_init);

    /* RX：浮空输入（对端是 STM32 的推挽 TX，会主动驱动电平） */
    gpio_init.GPIO_Pin = BOARD_LINK_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BOARD_LINK_GPIO, &gpio_init);

    usart_init.USART_BaudRate = BOARD_LINK_BAUDRATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  /* 双向：发偏角、收调试文本 */
    USART_Init(BOARD_LINK_UART, &usart_init);

    USART_ITConfig(BOARD_LINK_UART, USART_IT_RXNE, ENABLE);
    USART_Cmd(BOARD_LINK_UART, ENABLE);

    /* 优先级低于 K230 串口（那是 30 ms 一帧的实时数据），
     * 调试文本晚几十微秒转发没有任何影响。 */
    nvic_init.NVIC_IRQChannel = BOARD_LINK_UART_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic_init.NVIC_IRQChannelSubPriority = 2U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

void BSP_Link_SendBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == NULL || len == 0U) {
        return;
    }

    for (i = 0U; i < len; i++) {
        USART_SendData(BOARD_LINK_UART, (uint16_t)data[i]);
        while (USART_GetFlagStatus(BOARD_LINK_UART, USART_FLAG_TXE) == RESET) {
            /* 等待发送数据寄存器空 */
        }
    }
}

uint16_t BSP_Link_GetRxCount(void)
{
    /* 先取快照，避免两次读之间被中断改动 */
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BOARD_LINK_RX_BUF_SIZE - tail + head);
}

uint8_t BSP_Link_ReadByte(void)
{
    uint8_t byte;

    if (s_rx_head == s_rx_tail) {
        return 0xFFU;
    }

    byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BOARD_LINK_RX_BUF_SIZE);

    return byte;
}

/* ------------------------------------------------------------------ ISR ----
 * 只做「读寄存器 + 写缓冲 + 清标志」，不得超过几微秒：
 * 115200 下字节间隔 87 us，中断里做任何耗时操作都会丢字节。
 * ------------------------------------------------------------------------- */
void BOARD_LINK_UART_IRQHandler(void)
{
    if (USART_GetITStatus(BOARD_LINK_UART, USART_IT_RXNE) != RESET) {
        uint8_t byte = (uint8_t)USART_ReceiveData(BOARD_LINK_UART);
        uint16_t next = (uint16_t)((s_rx_head + 1U) % BOARD_LINK_RX_BUF_SIZE);

        /* 缓冲未满才写；满了丢弃新字节，保住尚未转发的数据 */
        if (next != s_rx_tail) {
            s_rx_buf[s_rx_head] = byte;
            s_rx_head = next;
        }

        USART_ClearITPendingBit(BOARD_LINK_UART, USART_IT_RXNE);
    }
}
