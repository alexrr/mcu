/*
 * NEC_Decode.h
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#ifndef INC_NEC_DECODE_H_
#define INC_NEC_DECODE_H_

#include <stdint.h>
#include "Project_HAL.h"
#include "ir_common.h"

typedef enum {
    NEC_NOT_EXTENDED, NEC_EXTENDED
} NEC_TYPE;

typedef enum {
    NEC_INIT, NEC_AGC_OK, NEC_AGC_FAIL, NEC_FAIL, NEC_OK
} NEC_STATE;

typedef struct {
	NEC_STATE state;
	NEC_TYPE  type;
} NEC_ProtoData_TypeDef;

void NEC_Init(IR_handle_type_def* handle);

void NEC_DeInit(IR_handle_type_def* handle);

void NEC_TIM_IC_CaptureCallback(IR_handle_type_def* handle);

void NEC_Read(IR_handle_type_def* handle);

#endif /* INC_NEC_DECODE_H_ */
