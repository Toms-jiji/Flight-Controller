/*
 * DEBUG_UART.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Vivek
 */

#include "main.h"

extern UART_HandleTypeDef huart6;

void TX_debug(char* tx_data){
	HAL_UART_Transmit(&huart6, tx_data, sizeof(tx_data), 100);
//	HAL_UART_Transmit (&huart6, 'T', 1, 10);
}
