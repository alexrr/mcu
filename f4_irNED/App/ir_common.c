/**
 ******************************************************************************
 * @file    ir_common.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    08-March-2016
 * @brief   This file provides the shared sirc/rc5 firmware functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ir_common.h"

#include "Project_Lib.h"
#include "Project_App.h"

/** @addtogroup IR_REMOTE
 * @{
 */

/** @addtogroup IR_COMMON
 * @brief Shared infra-red remote module
 * @{
 */

/**
 * @addtogroup IR_Common_Private_Defines
 * @{
 */
#define  RC5HIGHSTATE     ((uint8_t )0x02)   /* RC5 high level definition*/
#define  RC5LOWSTATE      ((uint8_t )0x01)   /* RC5 low level definition*/

/**
 * @}
 */
TIM_HandleTypeDef *TimHandleHF;
TIM_HandleTypeDef *TimHandleLF;
TIM_HandleTypeDef *TimHandleLED;

/**
 * @addtogroup IR_Common_Private_Variables
 * @{
 */
uint32_t ICValue1 = 0;
uint32_t ICValue2 = 0;

/**
 * @}
 */

/**
 * @addtogroup IR_Common_Public_Variables
 * @{
 */
uint8_t BitsSentCounter = 0;
uint8_t AddressIndex = 0;
uint8_t InstructionIndex = 0;
__IO ProtoSelector_t IR_ProtoSelect;

/**
 * @}
 */
static IR_eventTypeDef event;
/**
 * @addtogroup IR_Common_Exported_Functions
 * @{
 */
IR_handle_type_def IR_handle;
IR_handle_type_def* Get_IR_handle(){
  return &IR_handle;
}

QueueHandle_t Queue_ir_data;

void Init_IR(TIM_HandleTypeDef* htim,ProtoSelector_t ps){
	IR_handle.timerHandle = htim;
	IR_handle.ProtSelector = ps;
	Queue_ir_data = xQueueCreate(10, sizeof(uint8_t[4]));
	if(IR_handle.ProtSelector==NEC_DEC){
		NEC_Init(&IR_handle);
	}

}

void StartLoopIR(void const * argument){
	BaseType_t xStatus;
	uint8_t data[4];
	for (;;) {
		if (Queue_ir_data != NULL) {
			while ((xStatus = xQueueReceive(Queue_ir_data, data,
					portMAX_DELAY)) == pdPASS) {
				IR_handle.IR_DecodedCallback(data,4);
				osDelay(2);
			}
			if(IR_handle.ProtSelector==NEC_DEC){
				NEC_Read(&IR_handle);
			}
		}
		osDelay(1);
	}
}


/**
 * @brief  Capture callback in non blocking mode - level change
 * @param  htim: TIM handle
 * @retval None
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	InsideISR = 1;

	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	if(IR_handle.ProtSelector == NEC_DEC){
	    if (htim == IR_handle.timerHandle) {
	    	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	        NEC_TIM_IC_CaptureCallback(&IR_handle);
	    }
	}
	if (IR_ProtoSelect == RC5_DEC) {
		/* - Timer Falling Edge Event:
		 The Timer interrupt is used to measure the period between two
		 successive falling edges (The whole pulse duration).

		 - Timer Rising Edge Event:
		 It is also used to measure the duration between falling and rising
		 edges (The low pulse duration).
		 The two durations are useful to determine the bit value. Each bit is
		 determined according to the last bit.

		 Update event:InfraRed decoders time out event.
		 ---------------------------------------------
		 It resets the InfraRed decoders packet.
		 - The Timer Overflow is set to 3.6 ms .*/

		/* IC2 Interrupt*/
		if (htim->Channel == IR_TIM_DEC_CH_ACTIV_A) {
			ICValue2 = HAL_TIM_ReadCapturedValue(IR_handle.timerHandle,
			IR_TIM_DEC_CHANNEL_A);
			/* RC5 */
			if(ICValue2>ICValue1){
				RC5_DataSampling(ICValue2 - ICValue1, 0);
			}
			else{
				RC5_DataSampling(ICValue1 - ICValue2, 0);
			}

		} /* IC1 Interrupt */
		else if (htim->Channel == IR_TIM_DEC_CH_ACTIV_B) {
			ICValue1 = HAL_TIM_ReadCapturedValue(IR_handle.timerHandle,
			IR_TIM_DEC_CHANNEL_B);
			RC5_DataSampling(ICValue1, 1);
		}
	} else if (IR_ProtoSelect == SIRC_DEC) {
		/*The Timer interrupt is used to measure the different period between
		 two successive falling edges in order to identify the frame bits.

		 We measure the low pulse duration:
		 - If the period measured is equal to T = 1200 micros and the low pulse
		 duration is equal to T/2 = 600 micros => the bit is logic '0'.
		 - If the period measured is equal to 3T/2 = 1800 micros and the low pulse
		 duration is equal to T = 1200micros => the bit is logic '1'.
		 - If the whole period measured is equal to 3000 micros and the low pulse
		 duration is equal to 2400 micros => the bit is ‘start bit’.

		 Update event:InfraRed decoders time out event
		 ----------------------------------------------
		 It resets the InfraRed decoders packet.
		 - The Timer Overflow is set to 4 ms.  */

		/* IC2 Interrupt */
		if (htim->Channel == IR_TIM_DEC_CH_ACTIV_A) {
			/* Get the Input Capture value */
			ICValue2 = HAL_TIM_ReadCapturedValue(IR_handle.timerHandle,
					IR_TIM_DEC_CHANNEL_A);
			SIRC_DataSampling(ICValue1, ICValue2);
		} /* IC1 Interrupt*/
		else if (htim->Channel == IR_TIM_DEC_CH_ACTIV_B) {
			/* Get the Input Capture value */
			ICValue1 = HAL_TIM_ReadCapturedValue(IR_handle.timerHandle,
					IR_TIM_DEC_CHANNEL_B);
		}
	}
	InsideISR = 0;
}

/**
 * @brief  Period elapsed callback in non blocking mode - timeout
 * @param  htim: TIM handle
 * @retval None
 */
uint32_t cnt = 0;
void ir_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* Depending */
	InsideISR = 1;
	if (htim == &TimHandleLF) {
//    if (RFDemoStatus == (uint8_t)SIRC_ENC)
//    {
//      SIRC_Encode_SignalGenerate();
//    }
//    else if (RFDemoStatus == (uint8_t)RC5_ENC)
//    {
//      RC5_Encode_SignalGenerate();
//    }
	}
	if (htim == IR_handle.timerHandle) {
		++cnt;
		//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		if (Queue_ir_event != NULL) {
			event.Protocol = IR_ProtoSelect;
			event.Source = PeriodElapsedCallbackSource;
			event.Val1 = cnt;
			event.Val2 = 0;
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			xQueueSendFromISR(Queue_ir_event, &event,
					&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		if (IR_ProtoSelect == (uint8_t) SIRC_DEC) {
			SIRC_ResetPacket();
		} else if (IR_ProtoSelect == (uint8_t) RC5_DEC) {
			RC5_ResetPacket();
		}
	}
	InsideISR = 0;
}

/**
 * @brief  Identify TIM clock
 * @param  None
 * @retval Timer clock
 */

uint32_t TIM_GetCounterCLKValue(void) {
	uint32_t apbprescaler = 0, apbfrequency = 0;
	uint32_t timprescaler = 0;

	/* Get the clock prescaler of APB1 */
	apbprescaler = ((RCC->CFGR >> 8) & 0x7);
	apbfrequency = HAL_RCC_GetPCLK1Freq();
	timprescaler = TIM_PRESCALER;

	LogOut("\n\r apbprescaler=%lu, apbfrequency=%lu, timprescaler=%u ",
			apbprescaler, apbfrequency, timprescaler);
	/* If APBx clock div >= 4 */
	if (apbprescaler >= 4) {
		return ((apbfrequency * 2) / (timprescaler + 1));
	} else {
		return (apbfrequency / (timprescaler + 1));
	}
}

/**
 * @brief  Force new configuration to the output channel
 * @param  action: new configuration
 * @retval None
 */
void TIM_ForcedOC1Config(uint32_t action) {
	uint32_t temporary = TimHandleLF->Instance->CCMR1;

	temporary &= ~TIM_CCMR1_OC1M;
	temporary |= action;
	TimHandleLF->Instance->CCMR1 = temporary;
}


/**
 * @}
 */
uint8_t InsideISR = 0;
QueueHandle_t Queue_ir_event = NULL;
const uint8_t* eventSource_str[20] = { (uint8_t*) "UnfilledSource", /*  0 */
(uint8_t*) "ResultReady",	 /*  1 */
(uint8_t*) "PeriodElapsedCB", /* 2 */
(uint8_t*) "CaptureCB", /*  3 */
(uint8_t*) "DecodeFun", /*  4 */
(uint8_t*) "ResetFun", /*  5 */
(uint8_t*) "DataSamplingFun", /*  6 */
(uint8_t*) "modifyLastBitF", /*  7 */
(uint8_t*) "WriteBitFun", /*  6 */

};

uint8_t LoggingStart = 0;

void PrepareLogging() {

	Queue_ir_event = xQueueCreate(50, sizeof(IR_eventTypeDef));
	LoggingStart = 1;
}
void StartEventLog(void const * argument) {
	BaseType_t xStatus;
	IR_eventTypeDef mevent;
	for (;;) {
		if (Queue_ir_event != NULL) {
			while ((xStatus = xQueueReceive(Queue_ir_event, &mevent,
					portMAX_DELAY)) == pdPASS) {
				LogOut("\n\rIR_Log: prot=%u source=%s ch_bit=%u p1=%lu p2=%lu",
						mevent.Protocol, eventSource_str[mevent.Source],
						mevent.ch_bit, mevent.Val1, mevent.Val2);
				osDelay(2);
			}
		}
		osDelay(1);
	}
}

/**
 * @}
 */

/**
 * @}
 */
