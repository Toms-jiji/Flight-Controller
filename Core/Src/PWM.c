/*
 * ESP01.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Vivek
 */
#include "main.h"
#include "DEBUG_UART.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart6;

//#define QUAD_DEBUG
#define PWM_LOWER_LIMIT 1000
#define PWM_UPPER_LIMIT 2000
#define PWM_SCALING_FACTOR (PWM_UPPER_LIMIT-PWM_LOWER_LIMIT)/1000
#define PWM_OFFSET PWM_LOWER_LIMIT

#ifdef QUAD_DEBUG
	uint8_t message[100] = {'\0'};
#endif

void PWM_Start(){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void SET_PWM_Value(uint16_t PWM_RT, uint16_t PWM_RB, uint16_t PWM_LT, uint16_t PWM_LB){
	PWM_RT = (PWM_RT-1000)*PWM_SCALING_FACTOR + PWM_OFFSET;
	PWM_RB = (PWM_RB-1000)*PWM_SCALING_FACTOR + PWM_OFFSET;
	PWM_LT = (PWM_LT-1000)*PWM_SCALING_FACTOR + PWM_OFFSET;
	PWM_LB = (PWM_LB-1000)*PWM_SCALING_FACTOR + PWM_OFFSET;
#ifdef QUAD_DEBUG
	sprintf(message, "LB: %u \t LT: %u\t RT: %u \t RB: %u\n",PWM_LB, PWM_LT, PWM_RT, PWM_RB);
	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

	TIM1->CCR1 = PWM_RT;
	TIM1->CCR2 = PWM_RB;
    TIM2->CCR2 = PWM_LB;
    TIM3->CCR2 = PWM_LT;
}
