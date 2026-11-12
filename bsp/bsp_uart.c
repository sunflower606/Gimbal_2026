#include "bsp_uart.h"

#include <stddef.h>

#include "board_config.h"

/* 接收环形缓冲。
 * 约定：永远留一个空位区分「空」和「满」，所以实际可用
 * BOARD_VISION_RX_BUF_SIZE - 1 字节。缓冲区满时丢弃新字节（不覆盖未读数据）。 */
static uint8_t s_rx_buf[BOARD_VISION_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;    /* 中断写 */
static volatile uint16_t s_rx_tail = 0U;    /* 主循环写 */

/* --------------------------------------------------------------- 初始化 ---- */
void BSP_UART_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB1PeriphClockCmd(BOARD_VISION_UART_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_VISION_GPIO_CLK, ENABLE);

    /* TX：复用推挽（写成普通推挽的话引脚由 GPIO 而不是 USART 驱动，发不出数据） */
    gpio_init.GPIO_Pin = BOARD_VISION_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_VISION_GPIO, &gpio_init);

    /* RX：浮空输入（K230 侧是推挽输出，会主动驱动电平） */
    gpio_init.GPIO_Pin = BOARD_VISION_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BOARD_VISION_GPIO, &gpio_init);

    usart_init.USART_BaudRate = BOARD_VISION_BAUDRATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(BOARD_VISION_UART, &usart_init);

    USART_ITConfig(BOARD_VISION_UART, USART_IT_RXNE, ENABLE);
    USART_Cmd(BOARD_VISION_UART, ENABLE);

    nvic_init.NVIC_IRQChannel = BOARD_VISION_UART_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic_init.NVIC_IRQChannelSubPriority = 1U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

/* --------------------------------------------------------------- 发送 ---- */
void BSP_UART_SendByte(uint8_t byte)
{
    USART_SendData(BOARD_VISION_UART, (uint16_t)byte);
    while (USART_GetFlagStatus(BOARD_VISION_UART, USART_FLAG_TXE) == RESET) {
        /* 等待发送数据寄存器空 */
    }
}

void BSP_UART_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        BSP_UART_SendByte((uint8_t)(*text));
        text++;
    }
}

/* --------------------------------------------------------------- 接收 ---- */

/* 内部：把「距队尾 offset 个」换算成环形缓冲的物理下标 */
static uint16_t BSP_UART_IndexAt(uint16_t offset)
{
    uint16_t index = (uint16_t)(s_rx_tail + offset);

    if (index >= BOARD_VISION_RX_BUF_SIZE) {
        index = (uint16_t)(index - BOARD_VISION_RX_BUF_SIZE);
    }

    return index;
}

uint16_t BSP_UART_GetRxCount(void)
{
    /* 先取快照，避免两次读之间被中断改动 */
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BOARD_VISION_RX_BUF_SIZE - tail + head);
}

uint8_t BSP_UART_PeekAt(uint16_t offset, uint8_t *byte)
{
    if (offset >= BSP_UART_GetRxCount()) {
        return 0U;
    }

    if (byte != NULL) {
        *byte = s_rx_buf[BSP_UART_IndexAt(offset)];
    }

    return 1U;
}

void BSP_UART_Skip(uint16_t count)
{
    uint16_t available = BSP_UART_GetRxCount();

    if (count > available) {
        count = available;
    }

    s_rx_tail = BSP_UART_IndexAt(count);
}

uint8_t BSP_UART_ReadByte(void)
{
    uint8_t byte;

    if (s_rx_head == s_rx_tail) {
        return 0xFFU;
    }

    byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BOARD_VISION_RX_BUF_SIZE);

    return byte;
}

/* ------------------------------------------------------------------ ISR ----
 * 只做「读寄存器 + 写缓冲 + 清标志」，不得超过几微秒：
 * 115200 下字节间隔 87 us，中断里做任何耗时操作都会丢字节。
 * ------------------------------------------------------------------------- */
void BOARD_VISION_UART_IRQHandler(void)
{
    if (USART_GetITStatus(BOARD_VISION_UART, USART_IT_RXNE) != RESET) {
        uint8_t byte = (uint8_t)USART_ReceiveData(BOARD_VISION_UART);
        uint16_t next = (uint16_t)((s_rx_head + 1U) % BOARD_VISION_RX_BUF_SIZE);

        /* 缓冲未满才写；满了丢弃新字节，保住尚未解析的数据 */
        if (next != s_rx_tail) {
            s_rx_buf[s_rx_head] = byte;
            s_rx_head = next;
        }

        USART_ClearITPendingBit(BOARD_VISION_UART, USART_IT_RXNE);
    }
}
