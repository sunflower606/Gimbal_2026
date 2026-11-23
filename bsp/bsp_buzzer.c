/**
 * @file    bsp_buzzer.c
 * @brief   有源蜂鸣器 GPIO 控制（云台板 PB4）。
 *
 * 只做一件事：把某个电平写到 PB4。节奏/时长全部交给
 * modules/buzzer.c 处理，这样 BSP 层不掺业务逻辑。
 */
#include "bsp_buzzer.h"
#include "stm32f10x.h"
#include "pin_config.h"

/* 极性开关：1 = 高电平响（NPN 驱动最常见），0 = 低电平响 */
#define BOARD_BUZZER_ACTIVE_HIGH            0U

void BSP_Buzzer_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(BOARD_BUZZER_GPIO_CLK, ENABLE);

    /* 关键：PB4 复位后是 JTAG 的 NJTRST 脚，不解除 JTAG 就永远是输入，
     * 写 BSRR 完全无效（蜂鸣器一声都不响）。关掉 JTAG、保留 SWD 即可。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    gpio_init.GPIO_Pin = BOARD_BUZZER_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;   /* 推挽：三极管基极需要电流 */
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;   /* 蜂鸣器是慢信号，2 MHz 足够且干扰小 */
    GPIO_Init(BOARD_BUZZER_GPIO, &gpio_init);

    /* 上电先静音：避免初始化到第一次 Buzzer_Step 之间响一下 */
    BSP_Buzzer_Off();
}

void BSP_Buzzer_On(void)
{
#if (BOARD_BUZZER_ACTIVE_HIGH != 0U)
    GPIO_SetBits(BOARD_BUZZER_GPIO, BOARD_BUZZER_PIN);
#else
    GPIO_ResetBits(BOARD_BUZZER_GPIO, BOARD_BUZZER_PIN);
#endif
}

void BSP_Buzzer_Off(void)
{
#if (BOARD_BUZZER_ACTIVE_HIGH != 0U)
    GPIO_ResetBits(BOARD_BUZZER_GPIO, BOARD_BUZZER_PIN);
#else
    GPIO_SetBits(BOARD_BUZZER_GPIO, BOARD_BUZZER_PIN);
#endif
}
