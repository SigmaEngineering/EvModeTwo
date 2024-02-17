/*
 * UartClass.h
 *
 *  Created on: Feb 14, 2024
 *      Author: Admin
 */

#ifndef CLASSES_UARTCLASS_H_
#define CLASSES_UARTCLASS_H_

#include "main.h"
#include <cstring>

extern UART_HandleTypeDef huart2;

class UartClass {
public:
	UartClass(UART_HandleTypeDef * uart);
	virtual ~UartClass();
	void sendUart(const char*);
private:
	UART_HandleTypeDef *mUart;
};

#endif /* CLASSES_UARTCLASS_H_ */
