#include "bsp_uart_link.h"

#include <stddef.h>

#include "board_config.h"

void BSP_Link_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;

    RCC_APB2PeriphClockCmd(BOARD_LINK_UART_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_LINK_GPIO_CLK, ENABLE);

    /* TX：复用推挽。写成普通推挽的话引脚由 GPIO 驱动，发不出波形。 */
    gpio_init.GPIO_Pin = BOARD_LINK_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_LINK_GPIO, &gpio_init);

    usart_init.USART_BaudRate = BOARD_LINK_BAUDRATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Tx;      /* 只发不收 */
    USART_Init(BOARD_LINK_UART, &usart_init);

    USART_Cmd(BOARD_LINK_UART, ENABLE);
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
