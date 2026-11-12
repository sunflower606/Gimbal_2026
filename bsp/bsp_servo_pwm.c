#include "bsp_servo_pwm.h"

#include "board_config.h"

/* 内部：角度 -> 脉宽微秒数，并夹到舵机电气安全区。
 *
 * 这里再做一次限幅是「防御性冗余」：
 * 正常路径上 modules/gimbal_controller 已经按行程夹过角度，
 * 但 BSP 作为最后一层，必须保证绝不输出超出 [500, 2500] us 的脉宽 ——
 * 超范围的脉宽会让舵机顶到机械死点并持续堵转（发热、崩齿、烧电机）。 */
static uint16_t BSP_Servo_DegToPulse(float angle_deg, float full_scale_deg)
{
    float pulse_us = BOARD_SERVO_PULSE_MIN_US +
                     (angle_deg / full_scale_deg) * BOARD_SERVO_PULSE_SPAN_US;

    if (pulse_us < BOARD_SERVO_PULSE_MIN_US) {
        pulse_us = BOARD_SERVO_PULSE_MIN_US;
    }
    if (pulse_us > BOARD_SERVO_PULSE_MAX_US) {
        pulse_us = BOARD_SERVO_PULSE_MAX_US;
    }

    /* 四舍五入比直接截断更接近真实脉宽，避免系统性偏小半个计数 */
    return (uint16_t)(pulse_us + 0.5f);
}

void BSP_Servo_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef oc_init;

    /* TIM1 在 APB2，不要写成 RCC_APB1PeriphClockCmd */
    RCC_APB2PeriphClockCmd(BOARD_SERVO_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_SERVO_GPIO_CLK, ENABLE);

    /* 两个引脚都复用推挽：必须 AF_PP，配成 Out_PP 的话引脚由 GPIO 驱动，没有波形 */
    gpio_init.GPIO_Pin = BOARD_SERVO_YAW_PIN | BOARD_SERVO_PITCH_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_SERVO_GPIO, &gpio_init);

    TIM_InternalClockConfig(BOARD_SERVO_TIM);

    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Prescaler = BOARD_SERVO_TIM_PRESCALER;
    time_base.TIM_Period = BOARD_SERVO_TIM_PERIOD;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(BOARD_SERVO_TIM, &time_base);

    /* PWM 模式 1 + 高电平有效：CNT < CCR 输出高，高电平时间 = CCR 微秒 */
    TIM_OCStructInit(&oc_init);
    oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
    oc_init.TIM_OutputState = TIM_OutputState_Enable;
    oc_init.TIM_Pulse = 0U;

    /* CH1 -> PA8（pitch） */
    TIM_OC1Init(BOARD_SERVO_TIM, &oc_init);
    /* CH4 -> PA11（yaw） */
    TIM_OC4Init(BOARD_SERVO_TIM, &oc_init);

    TIM_ARRPreloadConfig(BOARD_SERVO_TIM, ENABLE);

    TIM_Cmd(BOARD_SERVO_TIM, ENABLE);

    /* TIM1 是高级定时器：必须打开主输出使能（MOE），
     * 否则 CH1/CH4 引脚永远是无效电平，示波器上看不到任何波形。 */
    TIM_CtrlPWMOutputs(BOARD_SERVO_TIM, ENABLE);

    /* 上电先给行程中点，避免「TIM_Pulse = 0 无信号」让舵机松掉乱摆 */
    BSP_Servo_SetYawDeg(BOARD_SERVO_YAW_FULL_SCALE_DEG * 0.5f);
    BSP_Servo_SetPitchDeg(BOARD_SERVO_PITCH_FULL_SCALE_DEG * 0.5f);
}

void BSP_Servo_SetYawDeg(float angle_deg)
{
    uint16_t pulse_us = BSP_Servo_DegToPulse(angle_deg, BOARD_SERVO_YAW_FULL_SCALE_DEG);

    /* yaw 走 TIM1_CH4 -> PA11 */
    TIM_SetCompare4(BOARD_SERVO_TIM, pulse_us);
}

void BSP_Servo_SetPitchDeg(float angle_deg)
{
    uint16_t pulse_us = BSP_Servo_DegToPulse(angle_deg, BOARD_SERVO_PITCH_FULL_SCALE_DEG);

    /* pitch 走 TIM1_CH1 -> PA8 */
    TIM_SetCompare1(BOARD_SERVO_TIM, pulse_us);
}
