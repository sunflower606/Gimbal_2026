#include "bsp_bt_uart.h"

#include <stddef.h>

#include "board_config.h"

/* ---------------------------------------------------------------- 环形缓冲 --
 * 两边都留一个空位区分「空」和「满」，实际可用 SIZE-1 字节。
 * TX 满：丢新字节（宁可丢日志，不能卡控制环）。
 * RX 满：丢新字节（当前没有命令协议，丢了也无所谓）。
 * ------------------------------------------------------------------------- */
static uint8_t s_tx_buf[BOARD_BT_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0U;    /* 主循环写 */
static volatile uint16_t s_tx_tail = 0U;    /* 中断读 */

static uint8_t s_rx_buf[BOARD_BT_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;    /* 中断写 */
static volatile uint16_t s_rx_tail = 0U;    /* 主循环读（当前不读） */

void BSP_BT_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB1PeriphClockCmd(BOARD_BT_UART_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_BT_GPIO_CLK, ENABLE);

    /* TX：复用推挽 */
    gpio_init.GPIO_Pin = BOARD_BT_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_BT_GPIO, &gpio_init);

    /* RX：浮空输入（模块 TXD 是推挽输出） */
    gpio_init.GPIO_Pin = BOARD_BT_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BOARD_BT_GPIO, &gpio_init);

    usart_init.USART_BaudRate = BOARD_BT_BAUDRATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(BOARD_BT_UART, &usart_init);

    /* 只开 RXNE；TXE 等到真有数据入队时再开，避免空转中断 */
    USART_ITConfig(BOARD_BT_UART, USART_IT_RXNE, ENABLE);
    USART_Cmd(BOARD_BT_UART, ENABLE);

    nvic_init.NVIC_IRQChannel = BOARD_BT_UART_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 3U;
    nvic_init.NVIC_IRQChannelSubPriority = 3U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

/* --------------------------------------------------------------- 入队 ---- */

/* 调用方必须在临界区里（关中断），否则 head/tail 会被 ISR 并发改动 */
static uint8_t BSP_BT_EnqueueLocked(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_tx_head + 1U) % BOARD_BT_TX_BUF_SIZE);

    if (next == s_tx_tail) {
        return 0U;  /* 队列满：丢弃新字节 */
    }

    s_tx_buf[s_tx_head] = byte;
    s_tx_head = next;

    return 1U;
}

uint8_t BSP_BT_SendByte(uint8_t byte)
{
    uint8_t ok;

    __disable_irq();
    ok = BSP_BT_EnqueueLocked(byte);
    if (ok != 0U) {
        /* 有数据了才打开 TXE 中断，让 ISR 去搬 */
        USART_ITConfig(BOARD_BT_UART, USART_IT_TXE, ENABLE);
    }
    __enable_irq();

    return ok;
}

uint16_t BSP_BT_SendBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t sent = 0U;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    __disable_irq();
    while (sent < len) {
        if (BSP_BT_EnqueueLocked(data[sent]) == 0U) {
            break;  /* 队列满，剩下的丢弃 */
        }
        sent++;
    }
    if (sent > 0U) {
        USART_ITConfig(BOARD_BT_UART, USART_IT_TXE, ENABLE);
    }
    __enable_irq();

    return sent;
}

uint16_t BSP_BT_GetTxPending(void)
{
    uint16_t head = s_tx_head;
    uint16_t tail = s_tx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BOARD_BT_TX_BUF_SIZE - tail + head);
}

uint16_t BSP_BT_GetRxCount(void)
{
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BOARD_BT_RX_BUF_SIZE - tail + head);
}

uint8_t BSP_BT_ReadByte(uint8_t *byte)
{
    if (byte == 0) {
        return 0U;
    }
    if (s_rx_tail == s_rx_head) {
        return 0U;              /* 空 */
    }

    *byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BOARD_BT_RX_BUF_SIZE);

    return 1U;
}

/* ------------------------------------------------------------------ ISR ----
 * 同样只做寄存器级操作，< 几微秒。
 * ------------------------------------------------------------------------- */
void BOARD_BT_UART_IRQHandler(void)
{
    /* 收：塞进 RX 环（当前没人读，但写满也只是丢字节，不影响别的） */
    if (USART_GetITStatus(BOARD_BT_UART, USART_IT_RXNE) != RESET) {
        uint8_t byte = (uint8_t)USART_ReceiveData(BOARD_BT_UART);
        uint16_t next = (uint16_t)((s_rx_head + 1U) % BOARD_BT_RX_BUF_SIZE);

        if (next != s_rx_tail) {
            s_rx_buf[s_rx_head] = byte;
            s_rx_head = next;
        }

        USART_ClearITPendingBit(BOARD_BT_UART, USART_IT_RXNE);
    }

    /* 发：搬一个字节；队列空了就把 TXE 中断关掉，别空转 */
    if (USART_GetITStatus(BOARD_BT_UART, USART_IT_TXE) != RESET) {
        if (s_tx_head != s_tx_tail) {
            USART_SendData(BOARD_BT_UART, (uint16_t)s_tx_buf[s_tx_tail]);
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) % BOARD_BT_TX_BUF_SIZE);
        } else {
            USART_ITConfig(BOARD_BT_UART, USART_IT_TXE, DISABLE);
        }
    }
}
