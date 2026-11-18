#include "stm32f10x.h"
#include "Delay.h"

/* ============================================================================
 * 毫秒时基（2026-11-18 重写：改成真正的 1 ms 中断）
 *
 * 【以前错在哪】
 *
 * 老做法是让 Delay_us() 每次自己重装 SysTick：
 *
 *      SysTick->LOAD = 72 * xus;
 *      SysTick->VAL  = 0x00;        <-- 清零
 *      ...
 *      SysTick->CTRL = 0x00000004;  <-- 关掉
 *
 * 于是「毫秒数」只能靠 Delay_us 自己记账（把每次延掉的微秒累加）。
 * 结果：主循环里除 Delay 之外的时间（串口转发、舵机计算、OLED）全部不漏算，
 * 时钟整体偏慢 —— 实测偏低 6%~10%。
 *
 * 本工程受影响的地方：GIMBAL_LOSE_TIMEOUT_MS(300 ms) 实际变成 320~330 ms，
 * 以及链路 30 ms 发送周期会随主循环开销一起漂。
 *
 * 【现在的做法】
 *
 *   SysTick 改成真正的 1 ms 周期中断（72000 个 HCLK），只负责 s_tick_ms++；
 *   Delay_us() 不再重装 SysTick，而是读 SysTick->VAL 的差值做微秒等待。
 *   这样 CPU 时间与真实时间一一对应。
 *
 * 【为什么不用 TIM6】
 *
 *   STM32F103C8 是中等容量型号，只有 TIM1~TIM4，没有 TIM6/TIM7
 *   （启动文件的向量表里到 USART3 / EXTI15_10 / RTCAlarm / USBWakeUp 就结束了）。
 *   本工程 TIM1 已用于两路舵机 PWM，所以只能用 SysTick。
 *
 * 【精度】
 *
 *   微秒等待按 SysTick->VAL 计数，1 个计数 = 1/72 us ≈ 13.9 ns。
 * ========================================================================== */

/* SysTick 重装值：72 MHz / 1000 = 1 ms */
#define DELAY_SYSTICK_RELOAD   (72000UL - 1UL)

/* 上电以来的毫秒数（SysTick 中断里自增） */
static volatile uint32_t s_tick_ms = 0U;

/* 1 = Delay_Init() 已经跑过 */
static uint8_t s_ready = 0U;

/**
  * @brief  SysTick 中断：1 ms 一次，只做计数
  */
void SysTick_Handler(void)
{
    s_tick_ms++;
}

/**
  * @brief  启动毫秒时基（可重复调用，只有第一次生效）
  * @note   没显式调用也会被 Delay_us()/Delay_GetMs() 自动补上。
  */
void Delay_Init(void)
{
    if (s_ready != 0U) {
        return;
    }

    SysTick->LOAD = DELAY_SYSTICK_RELOAD;
    SysTick->VAL  = 0U;
    /* bit2 CLKSOURCE=1(HCLK) | bit1 TICKINT=1 | bit0 ENABLE=1 */
    SysTick->CTRL = 0x00000007UL;

    s_ready = 1U;
}

/**
  * @brief  等若干个 SysTick 计数（72 个计数 = 1 us）
  * @note   SysTick 自由运行、会自动重装，这里用「差值累加」，
  *         跨过重装点也不会算错；单次等待长度不受 1 ms 限制。
  */
static void Delay_WaitTicks(uint32_t ticks)
{
    const uint32_t span = DELAY_SYSTICK_RELOAD + 1UL;   /* 一个完整周期的计数 */
    uint32_t last = SysTick->VAL;
    uint32_t elapsed = 0U;

    while (elapsed < ticks) {
        uint32_t now = SysTick->VAL;

        if (now <= last) {
            elapsed += (last - now);                    /* 本周期内正常倒数 */
        } else {
            elapsed += (last + (span - now));           /* 跨过 0，回绕到重装值 */
        }

        last = now;
    }
}

/**
  * @brief  微秒级延时
  * @param  xus 延时时长（us）
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
    if (s_ready == 0U) {
        Delay_Init();
    }

    Delay_WaitTicks(xus * 72UL);
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长（ms）
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
    while (xms--) {
        Delay_us(1000U);
    }
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长（s）
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
    while (xs--) {
        Delay_ms(1000U);
    }
}

/**
  * @brief  取上电以来经过的真实毫秒数
  * @retval 毫秒数（uint32_t，约 49.7 天回绕）
  */
uint32_t Delay_GetMs(void)
{
    if (s_ready == 0U) {
        Delay_Init();
    }

    return s_tick_ms;
}
