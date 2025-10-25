/*
 * pwm.h
 *
 *  Created on: Jul 16, 2025
 *      Author: Chard Ethern
 */

#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"

// Назначение каналов PWM
#define PWM_TIM4_CH2 TIM_CHANNEL_2  // PB7
#define PWM_TIM4_CH1 TIM_CHANNEL_1  // PB8
//#define PWM_TIM3_CH1 TIM_CHANNEL_1  // PB9
//#define PWM_TIM3_CH2 TIM_CHANNEL_2  // PB6

void PWM_Init(void);
void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us);

#endif // __PWM_H__
