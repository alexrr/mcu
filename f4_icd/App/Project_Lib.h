/*
 * Project_Lib.h
 *
 *  Created on: 2 θών. 2017 γ.
 *      Author: alexr
 */

#ifndef PROJECT_LIB_H_
#define PROJECT_LIB_H_
#include "FreeRTOS.H"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"

extern QueueHandle_t Queue_user_input;
extern QueueHandle_t Queue_ir_cap;


void SetLogDev();
void InitLogDev(UART_HandleTypeDef* new_log_h);
uint8_t* LogOut(uint8_t* _str, ...);
void LogOutCh(uint8_t ch);
void LogOutFix(uint8_t* pstr,uint16_t len);

#endif /* PROJECT_LIB_H_ */
