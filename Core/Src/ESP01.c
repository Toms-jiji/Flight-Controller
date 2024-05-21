/*
 * ESP01.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Vivek
 */
#include "main.h"

extern UART_HandleTypeDef huart4;

_Bool verify_reply(char* received_data, char* desired_data);
void TX_esp_data(char* tx_data);

void ESP_Init ()
{
	uint8_t data[100]={0};  //  creating a buffer of 10 bytes
//
//	TX_RX_esp_data("AT+RST\r\n", data);
//	HAL_Delay(3000);
//
//	TX_RX_verify_esp_data("AT\r\n", "AT\r\r\n\r\nOK\r\n"); //checking if ESP is responding or not
//
////	TX_RX_esp_data("AT+CWMODE?", data);
//
//	TX_RX_esp_data("AT+C", data);		//to set the ESP into the station mode
//	HAL_UART_Receive (&huart4, data, 20, 100);
////	TX_RX_esp_data("AT+CWMODE?", data);
//
//	TX_RX_esp_data("AT+CWMODE=1\r\n", data);		//to set the ESP into the station mode
//
////	TX_RX_esp_data("AT+CWMODE?", data);
//
//	TX_RX_esp_data("AT+CWMODE=1\r\n", data);		//to set the ESP into the station mode
//
//	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", "Esp", "20180338");
//
//	TX_RX_verify_esp_data(data, "WIFI GOT IP\r\n\r\nOK\r\n");

	  char buffer[50];
	  HAL_UART_Transmit(&huart4, (uint8_t*)"AT+RST\r\n", strlen("AT+RST\r\n"), 1000);
	  HAL_Delay(2000);
	  HAL_UART_Transmit(&huart4, (uint8_t*)"AT+CWMODE=1\r\n", strlen("AT+CWMODE=1\r\n"), 1000);
	  HAL_Delay(2000);
	  TX_RX_esp_data("AT+CWMODE?", data);
	  sprintf(buffer, "AT+CWJAP=\"Esp\",\"20180338\"\r\n");
	  HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
	  HAL_Delay(2000);

}

void TX_esp_data(char* tx_data){
	HAL_UART_Transmit (&huart4, tx_data, sizeof (tx_data), 10);
}


void TX_RX_esp_data(char* tx_data, char* rx_data){
	uint8_t Rx_data[500];
	HAL_UART_Transmit (&huart4, tx_data, sizeof (tx_data), 10);
	HAL_UART_Receive (&huart4, rx_data, 40, 100);
}


void TX_RX_verify_esp_data(char* tx_data, char* rx_data){
	uint8_t Rx_data[500];

	while(verify_reply(Rx_data, rx_data)){
		HAL_UART_Transmit (&huart4, tx_data, sizeof (tx_data), 10);
		HAL_UART_Receive (&huart4, Rx_data, 20, 100);
	}

}

_Bool verify_reply(char* received_data, char* desired_data){
	while(*desired_data!='\0' && received_data!='\0'){
		if(*desired_data != *received_data)
			return 1;
		else{
			*desired_data++;
			*received_data++;
		}
	}
	return 0;
}


