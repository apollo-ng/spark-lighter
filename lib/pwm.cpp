/*
https://gist.github.com/technobly/8313449 Thanks @ (Technobly/BDub)
*/

#include "pwm.h"

void                    setPWM          (uint8_t pin, uint8_t value)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

    if (pin >= TOTAL_PINS || PIN_MAP[pin].timer_peripheral == NULL)
    {
        return;
    }

    // SPI safety check
    if (SPI.isEnabled() == true && (pin == SCK || pin == MOSI || pin == MISO))
    {
        return;
    }

    // I2C safety check
    if (Wire.isEnabled() == true && (pin == SCL || pin == SDA))
    {
        return;
    }

    // Serial1 safety check
    if (Serial1.isEnabled() == true && (pin == RX || pin == TX))
    {
        return;
    }

    if (PIN_MAP[pin].pin_mode != OUTPUT &&
        PIN_MAP[pin].pin_mode != AF_OUTPUT_PUSHPULL)
    {
        return;
    }

    // Don't re-init PWM and cause a glitch if already setup,
    // just update duty cycle and return.
    if (PIN_MAP[pin].pin_mode == AF_OUTPUT_PUSHPULL)
    {
        TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM_ARR + 1) / 255);

        if (PIN_MAP[pin].timer_ch == TIM_Channel_1)
        {
            PIN_MAP[pin].timer_peripheral-> CCR1 = TIM_OCInitStructure.TIM_Pulse;
        }
        else if (PIN_MAP[pin].timer_ch == TIM_Channel_2)
        {
            PIN_MAP[pin].timer_peripheral-> CCR2 = TIM_OCInitStructure.TIM_Pulse;
        }
        else if (PIN_MAP[pin].timer_ch == TIM_Channel_3)
        {
            PIN_MAP[pin].timer_peripheral-> CCR3 = TIM_OCInitStructure.TIM_Pulse;
        }
        else if (PIN_MAP[pin].timer_ch == TIM_Channel_4)
        {
            PIN_MAP[pin].timer_peripheral-> CCR4 = TIM_OCInitStructure.TIM_Pulse;
        }
        return;
    }

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // PWM Frequency : PWM_FREQ (Hz)
    // TIM Counter clock = 24MHz
    uint16_t TIM_Prescaler = (uint16_t)(SystemCoreClock / 24000000) - 1;

    // TIM Channel Duty Cycle(%) = (TIM_CCR / TIM_ARR + 1) * 100
    uint16_t TIM_CCR = (uint16_t)(value * (TIM_ARR + 1) / 255);

    // AFIO clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    pinMode(pin, AF_OUTPUT_PUSHPULL);

    // TIM clock enable
    if (PIN_MAP[pin].timer_peripheral == TIM2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }
    else if (PIN_MAP[pin].timer_peripheral == TIM3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }
    else if (PIN_MAP[pin].timer_peripheral == TIM4)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(PIN_MAP[pin].timer_peripheral, & TIM_TimeBaseStructure);

    // PWM1 Mode configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = TIM_CCR;

    if (PIN_MAP[pin].timer_ch == TIM_Channel_1)
    {
        // PWM1 Mode configuration: Channel1
        TIM_OC1Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC1PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_2)
    {
        // PWM1 Mode configuration: Channel2
        TIM_OC2Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC2PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_3)
    {
        // PWM1 Mode configuration: Channel3
        TIM_OC3Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC3PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_4)
    {
        // PWM1 Mode configuration: Channel4
        TIM_OC4Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
        TIM_OC4PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(PIN_MAP[pin].timer_peripheral, ENABLE);

    // TIM enable counter
    TIM_Cmd(PIN_MAP[pin].timer_peripheral, ENABLE);
}
