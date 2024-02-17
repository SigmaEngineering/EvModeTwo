#ifndef STARTPOINT_H_
#define STARTPOINT_H_

#ifdef __cplusplus

#include "../Classes/Inc/UartClass.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#include "../Classes/Inc/MgmtLink.h"

extern "C" {
#endif

void start();

#ifdef __cplusplus

}


#endif

#endif // SRC_EVTASKS_MAINTASK_H_

