/*
 * Project_Includes.h
 *
 *  Created on: 26 мая 2017 г.
 *      Author: alexey.radchuk
 */

#ifndef _PROJECT_DRIVERS_H_
#define _PROJECT_DRIVERS_H_
#include "FreeRTOS.H"
#include "task.h"
#include "semphr.h"
#include "lcd.h"
#include "motor_cmd_exec.h"
#include "ir_decode.h"
#include "DateTime.h"
#include <string.h>
#include <stdio.h>

typedef struct {
	uint32_t ICvalue;
	uint8_t Channel;
} TIM_CapValue_TypeDef;

#endif /* PROJECT_DRIVERS_H_ */
