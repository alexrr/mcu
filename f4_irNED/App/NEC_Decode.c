/*
 * NEC_Decode.c
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#include "Project_Lib.h".h"
#include "NEC_Decode.h"
#include "ir_common.h"

void NEC_TIM_IC_CaptureCallback(IR_handle_type_def* handle) {
	uint8_t old_iISR=InsideISR;
	InsideISR =1;
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
    if (((NEC_ProtoData_TypeDef*)handle->ProtData)->state == NEC_INIT) {

        HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel1);

        if (handle->rawTimerData[1] < handle->timingAgcBoundary) {
        	((NEC_ProtoData_TypeDef*)handle->ProtData)->state = NEC_OK;
        	SendRept(handle);
        } else {
        	((NEC_ProtoData_TypeDef*)handle->ProtData)->state = NEC_AGC_OK;
            HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel1,
                    (uint32_t*) handle->rawTimerData, 32);
        }

    } else if (((NEC_ProtoData_TypeDef*)handle->ProtData)->state == NEC_AGC_OK) {

    	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
        HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel1);

        for (int pos = 0; pos < 32; pos++) {
            int time = handle->rawTimerData[pos];
            if (time > handle->timingBitBoundary) {
                handle->recieved[pos / 8] |= 1 << (pos % 8);
            } else {
                handle->recieved[pos / 8] &= ~(1 << (pos % 8));
            }
        }

        uint8_t valid = 1;

        uint8_t naddr = ~handle->recieved[0];
        uint8_t ncmd = ~handle->recieved[2];

        if (((NEC_ProtoData_TypeDef*)handle->ProtData) == NEC_NOT_EXTENDED && handle->recieved[1] != naddr){
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
        	valid = 0;
        }
        if (handle->recieved[3] != ncmd){
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
            valid = 0;
        }

        ((NEC_ProtoData_TypeDef*)handle->ProtData)->state = NEC_OK;

        if (valid)
        	SendData(handle);
        else
        	SendErr(handle);
    }
    InsideISR=old_iISR;
}

void SendData(IR_handle_type_def* handle){
	if(InsideISR){
		BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(Queue_ir_data, handle->recieved, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else{
		handle->IR_DecodedCallback(handle->recieved,4);
	}
}

void SendRept(IR_handle_type_def* handle){
	handle->recieved[0]=0xff;
	handle->recieved[1]=0xff;
	handle->recieved[2]=0xff;
	handle->recieved[3]=0xff;
	SendData(handle);
}

void SendErr(IR_handle_type_def* handle){
	handle->recieved[0]=0x00;
	handle->recieved[1]=0x00;
	handle->recieved[2]=0x00;
	handle->recieved[3]=0x00;
	SendData(handle);
}


void NEC_Read(IR_handle_type_def* handle) {
	((NEC_ProtoData_TypeDef*)handle->ProtData)->state = NEC_INIT;
    HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel1,
            (uint32_t*) handle->rawTimerData, 2);
}

static NEC_ProtoData_TypeDef NEC_data;
void NEC_Init(IR_handle_type_def* handle){
	handle->ProtData = &NEC_data;

	handle->timerChannel1 = TIM_CHANNEL_1;
	handle->timerChannelActive1 = HAL_TIM_ACTIVE_CHANNEL_1;

	handle->timingBitBoundary = 1680;
	handle->timingAgcBoundary = 12500;
	((NEC_ProtoData_TypeDef*)handle->ProtData)->type = NEC_EXTENDED;

//	__HAL_TIM_ENABLE_IT(handle->timerHandle,TIM_IT_CC1);
//	__HAL_TIM_ENABLE(handle->timerHandle);
//	  if (HAL_TIM_IC_Start_IT(handle->timerHandle, handle->timerChannel1) != HAL_OK)
//	  {
//	    /* Starting Error */
//	    Error_Handler();
//	    while (1)
//	    {}
//	  }

	  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
}
