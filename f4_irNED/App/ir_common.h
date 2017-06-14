/**
  ******************************************************************************
  * @file    ir_decode.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    08-March-2016
  * @brief   This file contains all the functions prototypes for the IR Decode
  *          firmware library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IR_COMMON_H
#define __IR_COMMON_H

/* Includes ------------------------------------------------------------------*/

#include "Project_HAL.h"
#include "Project_LIB.h"
#include <stdio.h>

//#include "rc5_decode.h"
//#include "sirc_decode.h"
//#include "sirc_encode.h"
//#include "NEC_Decode.h"

/** @addtogroup IR_REMOTE
  * @brief Infra-red remote control
  * @{
  */

/** @addtogroup IR_COMMON
  * @brief Shared IR modules
  * @{
  */

/** @defgroup IR_Common_Public_Types
  * @{
  */
typedef enum {
  NO = 0,
  YES = !NO
} StatusYesOrNo_t;

typedef enum
{
  NONE = 0,
  RC5_DEC,
  SIRC_DEC,
  RC5_ENC,
  SIRC_ENC,
  NEC_DEC
} ProtoSelector_t;

typedef struct {
	uint16_t rawTimerData[32];
    __IO uint8_t recieved[4];
    __IO uint8_t bitCount;

    __IO ProtoSelector_t ProtSelector;

    GPIO_TypeDef *IR_GPIO_PORT;     //   GPIOA                    /*!< Port which IR input is connected */
    uint16_t 	IR_GPIO_PIN;           // GPIO_PIN_0               /*!< Pin which IR is connected */
    TIM_TypeDef *IR_TIM;    /*!< Timer used for IR decoding */
    IRQn_Type IR_TIM_IRQn; //            TIM2_IRQn             /*!< IR TIM IRQ */

    TIM_HandleTypeDef *timerHandle;

    uint32_t timerChannel1;
    HAL_TIM_ActiveChannel timerChannelActive1;

    __IO uint16_t timingBitBoundary;
    __IO uint16_t timingAgcBoundary;

    void *ProtData;
    void (*IR_DecodedCallback)(uint16_t, uint8_t);
    void (*IR_ErrorCallback)();
    void (*IR_RepeatCallback)();
} IR_handle_type_def;

extern IR_handle_type_def IR_handle;
/**
  * @}
  */
IR_handle_type_def* Get_IR_handle();
void Init_IR(TIM_HandleTypeDef* htim,ProtoSelector_t ps);

/** @defgroup IR_Common_Public_Constants
  * @{
  */
extern TIM_HandleTypeDef    *TimHandleHF;
extern TIM_HandleTypeDef    *TimHandleLF;
extern TIM_HandleTypeDef    *TimHandleLED;

#define TIM_PRESCALER          ((uint32_t)89)           /*!< TIM prescaler */

#define IR_TIM_CLK             __HAL_RCC_TIM2_CLK_ENABLE      /*!< Clock of the used timer */
#define IR_GPIO_PORT_CLK       __HAL_RCC_GPIOA_CLK_ENABLE      /*!< IR pin GPIO Clock Port */

#define IR_TIM_DEC_CHANNEL_A   TIM_CHANNEL_1            /*!< IR TIM Channel */
#define IR_TIM_DEC_CHANNEL_B   TIM_CHANNEL_2            /*!< IR TIM Complementary channel */
#define IR_TIM_DEC_CH_ACTIV_A  HAL_TIM_ACTIVE_CHANNEL_1
#define IR_TIM_DEC_CH_ACTIV_B  HAL_TIM_ACTIVE_CHANNEL_2
#define IR_TIM_DEC_TRIGGER     TIM_TS_TI1FP1

#define IR_ENC_HPERIOD_RC5      ((uint32_t)1333)        /*!< RC5 Encoder modulation frequency base period */
#define IR_ENC_LPERIOD_RC5      ((uint32_t)46630)       /*!< RC5 Encoder pulse base period */
#define IR_ENC_HPERIOD_SIRC     ((uint32_t)1200)        /*!< SIRC Encoder modulation frequency base period */
#define IR_ENC_LPERIOD_SIRC     ((uint32_t)28799)       /*!< SIRC Encoder pulse base period */

//#define IR_GPIO_AF_TR          GPIO_AF0_IR
//#define IR_GPIO_AF_REC         GPIO_AF0_TIM3



#define MESSAGE1  " LEFT | RIGHT| DOWN  | SEL  "
#define MESSAGE2  " PREV | NEXT | SWITCH| SEND "

#define TIM_FORCED_ACTIVE      ((uint16_t)0x0050)
#define TIM_FORCED_INACTIVE    ((uint16_t)0x0040)

/**
  * @}
  */

/** @defgroup IR_Common_Public_Variables
  * @{
  */
extern uint32_t RC5_FrameManchestarFormat;
extern __IO uint32_t aSIRCFramePWForm[];
extern uint8_t BitsSentCounter;
extern uint8_t AddressIndex;
extern uint8_t InstructionIndex;
extern __IO ProtoSelector_t IR_ProtoSelect;

/**
  * @}
  */

/** @defgroup IR_Common_Exported_Functions
  * @{
  */
void SIRC_Encode_SignalGenerate(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint32_t TIM_GetCounterCLKValue(void);
void TIM_ForcedOC1Config(uint32_t action);
void ir_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
  * @}
  */
extern uint8_t InsideISR;
void StartEventLog(void const * argument);
void PrepareLogging();

typedef enum
{
    UnfilledSource = 0,
	PeriodElapsedCallbackSource, /*1*/
	CaptureCallbackSource,		/*2*/
	DecodeFunSource,			/*3*/
	ResetFunSource,				/*4*/
	DataSamplingFunSource,		/*5*/
	modifyLastBitFunSource,		/*6*/
	WriteBitFunSource			/*7*/
} IR_eventSource_enum;

typedef struct
{
  uint8_t Protocol;   /*!< Field bit */
  IR_eventSource_enum Source;
  uint8_t  ch_bit;
  uint8_t  bit2;
  uint32_t Val1;
  uint32_t Val2;
}
IR_eventTypeDef;

extern QueueHandle_t Queue_ir_event;
extern uint8_t LoggingStart;


/**
  * @}
  */

/**
  * @}
  */
#endif /* __IR_COMMON_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
