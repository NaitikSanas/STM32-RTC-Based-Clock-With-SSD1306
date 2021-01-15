/*
 * print.cpp
 *
 *  Created on: Nov 24, 2020
 *      Author: Naitik
 */

#include "print.h"
extern UART_HandleTypeDef huart2;

void printMsg (char *msg, ...){
	char buff[80];
		va_list args;
		va_start(args,msg);
		vsprintf(buff,msg,args);
		HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}



