/*
 * ir_decode.c
 *
 *  Created on: 3 θών. 2017 γ.
 *      Author: alexr
 */
#include "ir_decode.h"
#include "Project_Lib.h"

TIM_HandleTypeDef* ir_htim;
__IO uint8_t irq_src=0;

/* Logic table for rising edge: every line has values corresponding to previous bit.
 In columns are actual bit values for given bit time. */
const tRC5_lastBitType RC5_logicTableRisingEdge[2][2] = { { RC5_ZER, RC5_INV }, /* lastbit = ZERO */
{ RC5_NAN, RC5_ZER }, /* lastbit = ONE  */
};

/* Logic table for falling edge: every line has values corresponding to previous bit.
 In columns are actual bit values for given bit time. */
const tRC5_lastBitType RC5_logicTableFallingEdge[2][2] = { { RC5_NAN, RC5_ONE }, /* lastbit = ZERO */
{ RC5_ONE, RC5_INV }, /* lastbit = ONE  */
};

volatile StatusYesOrNo RC5FrameReceived = NO; /*!< RC5 Frame state */
volatile tRC5_packet RC5TmpPacket; /*!< First empty packet */

/* RC5  bits time definitions */
uint16_t RC5MinT = 0;
uint16_t RC5MaxT = 0;
uint16_t RC5Min2T = 0;
uint16_t RC5Max2T = 0;
volatile uint32_t TIMCLKValueKHz; /*!< Timer clock */
uint16_t RC5TimeOut = 0;

/** @defgroup RC5_Private_FunctionPrototypes
 * @{
 */
static void IR_RC5_DataSampling(uint16_t rawPulseLength, uint8_t edge);
static uint8_t IR_RC5_GetPulseLength(uint16_t pulseLength);
static void IR_RC5_modifyLastBit(tRC5_lastBitType bit);
static void IR_RC5_WriteBit(uint8_t bitVal);
static uint32_t TIM_GetCounterCLKValue(void);

void IR_RC5_DataSampling(uint16_t rawPulseLength, uint8_t edge) {
	uint8_t pulse;
	tRC5_lastBitType tmpLastBit;

	/* Decode the pulse length in protocol units */
	pulse = IR_RC5_GetPulseLength(rawPulseLength);

	/* On Rising Edge */
	if (edge == 1) {
		if (pulse <= RC5_2T_TIME) {
			/* Bit determination by the rising edge */
			tmpLastBit = RC5_logicTableRisingEdge[RC5TmpPacket.lastBit][pulse];
			IR_RC5_modifyLastBit(tmpLastBit);
		} else {
			IR_RC5_ResetPacket();
		}
	} else /* On Falling Edge */
	{
		/* If this is the first falling edge - don't compute anything */
		if (RC5TmpPacket.status & RC5_PACKET_STATUS_EMPTY) {
			RC5TmpPacket.status &= (uint8_t) ~RC5_PACKET_STATUS_EMPTY;
		} else {
			if (pulse <= RC5_2T_TIME) {
				/* Bit determination by the falling edge */
				tmpLastBit =
						RC5_logicTableFallingEdge[RC5TmpPacket.lastBit][pulse];
				IR_RC5_modifyLastBit(tmpLastBit);
			} else {
				IR_RC5_ResetPacket();
			}
		}
	}
}

/**
 * @brief  Convert raw pulse length expressed in timer ticks to protocol bit times.
 * @param  pulseLength:pulse duration
 * @retval bit time value
 */
static uint8_t IR_RC5_GetPulseLength(uint16_t pulseLength) {
	/* Valid bit time */
	if ((pulseLength > RC5MinT) && (pulseLength < RC5MaxT)) {
		/* We've found the length */
		return (RC5_1T_TIME); /* Return the correct value */
	} else if ((pulseLength > RC5Min2T) && (pulseLength < RC5Max2T)) {
		/* We've found the length */
		return (RC5_2T_TIME);/* Return the correct value */
	}
	return RC5_WRONG_TIME;/* Error */
}

/**
 * @brief  perform checks if the last bit was not incorrect.
 * @param  bit: where bit can be  RC5_NAN or RC5_INV or RC5_ZER or RC5_ONE
 * @retval None
 */
static void IR_RC5_modifyLastBit(tRC5_lastBitType bit) {
	if (bit != RC5_NAN) {
		if (RC5TmpPacket.lastBit != RC5_INV) {
			/* Restore the last bit */
			RC5TmpPacket.lastBit = bit;

			/* Insert one bit into the RC5 Packet */
			IR_RC5_WriteBit(RC5TmpPacket.lastBit);
		} else {
			IR_RC5_ResetPacket();
		}
	}
}

/**
 * @brief  Insert one bit into the final data word.
 * @param  bitVal: bit value 'RC5_ONE' or 'RC5_ZER'
 * @retval None
 */
static void IR_RC5_WriteBit(uint8_t bitVal) {
	/* First convert RC5 symbols to ones and zeros */
	if (bitVal == RC5_ONE) {
		bitVal = 1;
	} else if (bitVal == RC5_ZER) {
		bitVal = 0;
	} else {
		IR_RC5_ResetPacket();
		return;
	}

	/* Write this particular bit to data field */
	RC5TmpPacket.data |= bitVal;

	/* Test the bit number determined */
	if (RC5TmpPacket.bitCount != 0) /* If this is not the last bit */
	{
		/* Shift the data field */
		RC5TmpPacket.data = RC5TmpPacket.data << 1;
		/* And decrement the bitCount */
		RC5TmpPacket.bitCount--;
	} else {
		RC5FrameReceived = YES;
		LogOut("\n\rRecived IR:%x",RC5TmpPacket.data);
	}
}

/**
 * @brief  Identify TIM clock
 * @param  None
 * @retval Timer clock
 */
static uint32_t TIM_GetCounterCLKValue(void) {
	uint32_t apbprescaler = 0, apbfrequency = 0;
	uint32_t timprescaler = ir_htim->Init.Prescaler;

	uint32_t HCLK_Frequency = HAL_RCC_GetHCLKFreq();
	uint32_t iFlashLatency = 0;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &iFlashLatency);

	if ((ir_htim->Instance == TIM1) || (ir_htim->Instance == TIM8)
#ifdef TIM15
			|| (ir_htim->Instance == TIM15)
#endif
#ifdef TIM16
			|| (ir_htim->Instance == TIM16)
#endif
#ifdef TIM17
			|| (ir_htim->Instance == TIM17)
#endif
			) {
		/* Get the clock prescaler of APB2 */
		//apbfrequency = HCLK_Frequency / RCC_ClkInitStruct.APB2CLKDivider;
		apbfrequency = HAL_RCC_GetPCLK2Freq();
		apbprescaler = RCC_ClkInitStruct.APB2CLKDivider;
	} else if ((ir_htim->Instance == TIM2) || (ir_htim->Instance == TIM3)
			|| (ir_htim->Instance == TIM4) || (ir_htim->Instance == TIM5)
			|| (ir_htim->Instance == TIM6) || (ir_htim->Instance == TIM7)) {
		/* Get the clock prescaler of APB1 */
		//apbfrequency = HCLK_Frequency / RCC_ClkInitStruct.APB1CLKDivider;
		apbfrequency = HAL_RCC_GetPCLK1Freq();
		apbprescaler = RCC_ClkInitStruct.APB1CLKDivider;
	}

	/* If APBx clock div >= 4   need apbfrequency * 2 */
	LogOut("\n\rapbprescaler=%lu,apbfrequency=%lu,timprescaler=%lu",apbprescaler,apbfrequency, timprescaler);
	if (apbprescaler >= 2) {
		return ((apbfrequency * 2) / (timprescaler + 1));
	} else {
		return (apbfrequency / (timprescaler + 1));
	}
}

uint8_t IR_RC5_Decode(IR_Frame_TypeDef *rc5_frame) {
	/* If frame received */
	if (RC5FrameReceived != NO) {
		/* RC5 frame field decoding */
		rc5_frame->FieldBit = (RC5TmpPacket.data >> 12) & 0x1;
		rc5_frame->ToggleBit = (RC5TmpPacket.data >> 11) & 0x1;
		rc5_frame->Address = (RC5TmpPacket.data >> 6) & 0x1F;
		rc5_frame->Command = (RC5TmpPacket.data) & 0x3F;

		/* Default state */
		RC5FrameReceived = NO;
		IR_RC5_ResetPacket();
		return 1;
#ifdef USE_LCD
#endif
	}
	return 0;
}

/**
 * @brief  Set the incoming packet structure to default state.
 * @param  None
 * @retval None
 */
void IR_RC5_ResetPacket(void) {
	RC5TmpPacket.data = 0;
	RC5TmpPacket.bitCount = RC5_PACKET_BIT_COUNT - 1;
	RC5TmpPacket.lastBit = RC5_ONE;
	RC5TmpPacket.status = RC5_PACKET_STATUS_EMPTY;
}

/**
 * @brief  This function handles TIM interrupt Handler.
 *         RC5 Infrared decoder Implementation
 *         ===================================
 *         Capture Compare interrupt:
 *         --------------------------
 *        - Timer Falling Edge Event:
 *         The Timer interrupt is used to measure the period between two
 *         successive falling edges (The whole pulse duration).
 *
 *         - Timer Rising Edge Event:
 *         It is also used to measure the duration between falling and rising
 *         edges (The low pulse duration).
 *         The two durations are useful to determine the bit value. Each bit is
 *         determined according to the last bit.
 *
 *         Update event:InfraRed decoders time out event.
 *         ---------------------------------------------
 *         It resets the InfraRed decoders packet.
 *         - The Timer Overflow is set to 3.6 ms .
 * @param  None
 * @retval None
 */
static uint32_t ICValue1;
static uint32_t ICValue2;

TIM_CapValue_TypeDef IC_val;

void TIMx_IRQHandler_irdecode(TIM_HandleTypeDef *htim) {
	if (htim->Instance != ir_htim->Instance) {
		return;
	}
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	IC_val.Channel = 0;
	IC_val.CR1 = irq_src;
	IC_val.CR2 = 0;
	IC_val.ICvalue = 0;

	if (__HAL_TIM_GET_FLAG(htim,TIM_FLAG_CC1) != RESET) {
		ICValue2 = HAL_TIM_ReadCapturedValue(ir_htim, TIM_CHANNEL_1);
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
		IC_val.Channel = TIM_FLAG_CC1;
		IC_val.ICvalue = ICValue2 - ICValue1;
		IC_val.CR2 = ir_htim->Instance->CR2;
		IR_RC5_DataSampling(ICValue2 - ICValue1, 0);
		xQueueSendFromISR(Queue_ir_cap, &IC_val, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	} else if (__HAL_TIM_GET_FLAG(htim,TIM_FLAG_CC2) != RESET) {
		ICValue1 = HAL_TIM_ReadCapturedValue(ir_htim, TIM_CHANNEL_2);
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
		IC_val.Channel = TIM_FLAG_CC2;
		IC_val.ICvalue = ICValue1;
		IC_val.CR2 = ir_htim->Instance->CR2;
		IR_RC5_DataSampling(ICValue2 - ICValue1, 0);
		IR_RC5_DataSampling(ICValue1, 1);
		xQueueSendFromISR(Queue_ir_cap, &IC_val, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	} else {
		__HAL_TIM_CLEAR_FLAG(ir_htim, TIM_FLAG_UPDATE);
		if(RC5TmpPacket.status!=RC5_PACKET_STATUS_EMPTY){
			IR_RC5_ResetPacket();
			IC_val.Channel = 0xff;
			xQueueSendFromISR(Queue_ir_cap, &IC_val, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}

//	if (Queue_ir_cap != NULL) {
//		BaseType_t xHigherPriorityTaskWoken;
//		xHigherPriorityTaskWoken = pdFALSE;
//		IC_val.Channel = htim->Channel;
//		switch (htim->Channel) {
//		case HAL_TIM_ACTIVE_CHANNEL_1:
//			IC_val.ICvalue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			break;
//		case HAL_TIM_ACTIVE_CHANNEL_2:
//			IC_val.ICvalue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//			break;
//		default:
//			IC_val.ICvalue = 0;
//		}
//
//		xQueueSendFromISR(Queue_ir_cap, &IC_val, &xHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}

	/* IC1 Interrupt*/

//  if((TIM_GetFlagStatus(ir_htim->Instance, TIM_FLAG_CC1) != RESET))
//  {
//    TIM_ClearFlag(ir_htim->Instance, TIM_FLAG_CC1);
//    /* Get the Input Capture value */
////    ICValue2 = TIM_GetCapture1(ir_htim->Instance);
//    ICValue2 = HAL_TIM_ReadCapturedValue(ir_htim->Instance,TIM_CHANNEL_1);
//    /* RC5 */
//    IR_RC5_DataSampling(ICValue2 - ICValue1, 0);
//  }  /* IC2 Interrupt */
//  else  if((TIM_GetFlagStatus(ir_htim->Instance, TIM_FLAG_CC2) != RESET))
//  {
//    TIM_ClearFlag(ir_htim->Instance, TIM_FLAG_CC2);
//    /* Get the Input Capture value */
////    ICValue1 = TIM_GetCapture2(ir_htim->Instance);
//    ICValue1 = HAL_TIM_ReadCapturedValue(ir_htim->Instance,TIM_CHANNEL_2);
//    IR_RC5_DataSampling(ICValue1 , 1);
//  }
	/* Checks whether the IR_TIM flag is set or not.*/
//  else if ((TIM_GetFlagStatus(ir_htim->Instance, TIM_FLAG_Update) != RESET))
//  {
//    /* Clears the IR_TIM's pending flags*/
//    TIM_ClearFlag(ir_htim->Instance, TIM_FLAG_Update);
//
//    IR_RC5_ResetPacket();
//  }
}

/**
 * @brief  Initialize the RC5 decoder module ( Time range)
 * @param  None
 * @retval None
 */
void IR_RC5_Init(TIM_HandleTypeDef* _ir_htim) {
//  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  TIM_ICInitTypeDef TIM_ICInitStructure;
//
//  /*  Clock Configuration for TIMER */
//  RCC_APB1PeriphClockCmd(IR_TIM_CLK , ENABLE);
//
//  /* Enable Button GPIO clock */
//  RCC_APB2PeriphClockCmd(IR_GPIO_PORT_CLK | RCC_APB2Periph_AFIO, ENABLE);
//
//  /* Pin configuration: input floating */
//  GPIO_InitStructure.GPIO_Pin = IR_GPIO_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(IR_GPIO_PORT, &GPIO_InitStructure);
//
//  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
//
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//
//  /* Enable the TIM global Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = IR_TIM_IRQn ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//  /* TIMER frequency input */
//  TIM_PrescalerConfig(IR_TIM, TIM_PRESCALER, TIM_PSCReloadMode_Immediate);
//
//  /* TIM configuration */
//  TIM_ICInitStructure.TIM_Channel = IR_TIM_Channel;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_PWMIConfig(IR_TIM, &TIM_ICInitStructure);
//
//  /* Timer Clock */
//
//  /* Select the TIM3 Input Trigger: TI1FP1 */
//  TIM_SelectInputTrigger(IR_TIM, TIM_TS_TI1FP1);
//
//  /* Select the slave Mode: Reset Mode */
//  TIM_SelectSlaveMode(IR_TIM, TIM_SlaveMode_Reset);
//
//  /* Enable the Master/Slave Mode */
//  TIM_SelectMasterSlaveMode(IR_TIM, TIM_MasterSlaveMode_Enable);
//
//  /* Configures the TIM Update Request Interrupt source: counter overflow */
//  TIM_UpdateRequestConfig(IR_TIM,  TIM_UpdateSource_Regular);
//
//
//  /* Set the TIM auto-reload register for each IR protocol */
//
//  /* Clear update flag */
//  TIM_ClearFlag(IR_TIM, TIM_FLAG_Update);
//
//  /* Enable TIM3 Update Event Interrupt Request */
//  TIM_ITConfig(IR_TIM, TIM_IT_Update, ENABLE);
//
//  /* Enable the CC2/CC1 Interrupt Request */
//  TIM_ITConfig(IR_TIM, TIM_IT_CC2, ENABLE);
//  TIM_ITConfig(IR_TIM, TIM_IT_CC1, ENABLE);
//
//  /* Enable the timer */
//  TIM_Cmd(IR_TIM, ENABLE);

	HAL_GPIO_TogglePin(PCAP1_GPIO_Port, PCAP1_Pin);
	uint8_t* pstr=LogOut("\n\r#RC5 init");

	ir_htim = _ir_htim;

	//TIMCLKValueKHz = HAL_RCC_GetHCLKFreq() / 1000;
    TIMCLKValueKHz = TIM_GetCounterCLKValue()/1000;
    LogOut("\n\r TIMCLKValueKHz=%lu",TIMCLKValueKHz);
    RC5TimeOut = TIMCLKValueKHz * RC5_TIME_OUT_US / 1000;
    LogOut("\n\r RC5TimeOut=%lu", RC5TimeOut);

	ir_htim->Instance->ARR = RC5TimeOut;
	/* Bit time range */
	RC5MinT = (RC5_T_US - RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
	RC5MaxT = (RC5_T_US + RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
	RC5Min2T = (2 * RC5_T_US - RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
	RC5Max2T = (2 * RC5_T_US + RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;

    LogOut("\n\r RC5MinT=%u RC5MaxT=%u\n\r RC5Min2T=%u  RC5Max2T=%u",RC5MinT,RC5MaxT,RC5Min2T,RC5Max2T);

	__HAL_TIM_CLEAR_FLAG(ir_htim, TIM_FLAG_UPDATE);

    if (HAL_TIM_IC_Start_IT(ir_htim, TIM_CHANNEL_2) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

    /*##-5- Start the Input Capture in interrupt mode ##########################*/
	if (HAL_TIM_IC_Start_IT(ir_htim, TIM_CHANNEL_1) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

	if (HAL_TIM_Base_Start_IT(ir_htim) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

	/* Default state */

    IR_RC5_ResetPacket();
	osDelay(100);

    LogOut("\n\rRC5 init complite");

    HAL_GPIO_TogglePin(PCAP1_GPIO_Port, PCAP1_Pin);
}
