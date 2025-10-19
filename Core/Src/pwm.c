/*
 * pwm.c
 *
 *  Created on: Jul 16, 2025
 *      Author: Chard Ethern
 */
#include "pwm.h"

// Экспортируемые таймеры
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;

// Частота PWM 50 Гц (для сервоприводов)
//#define PWM_FREQUENCY_HZ 50
//#define TIMER_CLOCK_HZ   84000000 // 84 MHz

void PWM_Init(void)
{
    // Запуск PWM каналов
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // PB7
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // PB8
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // PB9
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // PB6

    // Start 100Hz PWM channels (TIM3)
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // e.g., PB4- 100Hz
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // e.g., PB5- 100Hz
}

void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    // Clamp pulse to servo-safe range
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

