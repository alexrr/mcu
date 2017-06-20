/*
 * NEC_Decode.c
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#include "NEC_Decode.h"

uint8_t IR_waiting=0;

void NEC_TIM_IC_CaptureCallback(NEC* handle) {

    if (handle->state == NEC_INIT) {
        HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);
    	IR_waiting=0;
        handle->Busy = 0;

        if (handle->rawTimerData[1] < handle->timingAgcBoundary) {
            handle->state = NEC_OK;
            handle->Busy = 0;
            IR_waiting=0;
            handle->NEC_DecodedCallback(handle->decoded[0]<<8|handle->decoded[1], handle->decoded[2]<<8|handle->decoded[3]);
//            handle->NEC_RepeatCallback();
        } else {
            handle->state = NEC_AGC_OK;
            IR_waiting=1;
            handle->Busy = 1;
            HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
                    (uint32_t*) handle->rawTimerData, 32);
        }

    } else if (handle->state == NEC_AGC_OK) {
        HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);

        for (int pos = 0; pos < 32; pos++) {
            int time = handle->rawTimerData[pos];
            if (time > handle->timingBitBoundary) {
                handle->decoded[pos / 8] |= 1 << (pos % 8);
            } else {
                handle->decoded[pos / 8] &= ~(1 << (pos % 8));
            }
        }

        uint8_t valid = 1;

        uint8_t naddr = ~handle->decoded[0];
        uint8_t ncmd = ~handle->decoded[2];

//        if (handle->type == NEC_NOT_EXTENDED && handle->decoded[1] != naddr)
//            valid = 0;
//        if (handle->decoded[3] != ncmd)
//            valid = 0;

        handle->state = NEC_OK;

    	IR_waiting=0;
        handle->Busy = 0;
        if (valid){
            handle->NEC_DecodedCallback(handle->decoded[0]<<8|handle->decoded[1], handle->decoded[2]<<8|handle->decoded[3]);
        }
        else{
//        	handle->decoded[1]=0xff;
//        	handle->decoded[3]=0xff;
//        	handle->NEC_DecodedCallback(0xff,0xff);
        	    handle->decoded[0]=0;
        	    handle->decoded[1]=0;
        	    handle->decoded[2]=0;
        	    handle->decoded[3]=0;
            handle->NEC_ErrorCallback();
        }
        handle->state = NEC_INIT;
        NEC_Read(handle);
    }
}

void NEC_Read(NEC* handle) {
	if(handle->Busy!= 1){
	    handle->state = NEC_INIT;
	//    handle->decoded[0]=0;
	//    handle->decoded[1]=0;
	//    handle->decoded[2]=0;
	//    handle->decoded[3]=0;
	    handle->Busy = 1;
	    IR_waiting=1;
	    HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
	            (uint32_t*) handle->rawTimerData, 2);
	}

}

void NEC_TimeOut(NEC* handle){
    handle->Busy = 0;
    IR_waiting=0;
    HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);
}
