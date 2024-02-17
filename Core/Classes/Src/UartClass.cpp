/*
 * UartClass.cpp
 *
 *  Created on: Feb 14, 2024
 *      Author: Admin
 */

#include "../Inc/UartClass.h"


UartClass::UartClass(UART_HandleTypeDef * uart) : mUart{uart} { }

UartClass::~UartClass() {
	// TODO Auto-generated destructor stub
}

void UartClass::sendUart(const char* data)
{
	HAL_UART_Transmit(mUart, (uint8_t*)data, strlen(data),100);
}
