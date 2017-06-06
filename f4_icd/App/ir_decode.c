/*
 * ir_decode.c
 *
 *  Created on: 3 θών. 2017 γ.
 *      Author: alexr
 */
#include "ir_decode.h"
TIM_HandleTypeDef* ir_htim;

/* Logic table for rising edge: every line has values corresponding to previous bit.
   In columns are actual bit values for given bit time. */
const tRC5_lastBitType RC5_logicTableRisingEdge[2][2] =
{
  {RC5_ZER ,RC5_INV}, /* lastbit = ZERO */
  {RC5_NAN ,RC5_ZER}, /* lastbit = ONE  */
};

/* Logic table for falling edge: every line has values corresponding to previous bit.
   In columns are actual bit values for given bit time. */
const tRC5_lastBitType RC5_logicTableFallingEdge[2][2] =
{
  {RC5_NAN ,RC5_ONE},  /* lastbit = ZERO */
  {RC5_ONE ,RC5_INV},  /* lastbit = ONE  */
};

volatile StatusYesOrNo RC5FrameReceived = NO; /*!< RC5 Frame state */
volatile tRC5_packet   RC5TmpPacket;          /*!< First empty packet */

/* RC5  bits time definitions */
uint16_t  RC5MinT = 0;
uint16_t  RC5MaxT = 0;
uint16_t  RC5Min2T = 0;
uint16_t  RC5Max2T = 0;
volatile uint32_t TIMCLKValueKHz; /*!< Timer clock */
uint16_t RC5TimeOut = 0;

/** @defgroup RC5_Private_FunctionPrototypes
  * @{
  */
static void IR_RC5_DataSampling(uint16_t rawPulseLength, uint8_t edge);
static uint8_t IR_RC5_GetPulseLength (uint16_t pulseLength);
static void IR_RC5_modifyLastBit(tRC5_lastBitType bit);
static void IR_RC5_WriteBit(uint8_t bitVal);
static uint32_t TIM_GetCounterCLKValue(void);

void IR_RC5_DataSampling(uint16_t rawPulseLength, uint8_t edge)
{
  uint8_t pulse;
  tRC5_lastBitType tmpLastBit;

  /* Decode the pulse length in protocol units */
  pulse = IR_RC5_GetPulseLength(rawPulseLength);

  /* On Rising Edge */
  if (edge == 1)
  {
    if (pulse <= RC5_2T_TIME)
    {
      /* Bit determination by the rising edge */
      tmpLastBit = RC5_logicTableRisingEdge[RC5TmpPacket.lastBit][pulse];
      IR_RC5_modifyLastBit (tmpLastBit);
    }
    else
    {
      IR_RC5_ResetPacket();
    }
  }
  else     /* On Falling Edge */
  {
    /* If this is the first falling edge - don't compute anything */
    if (RC5TmpPacket.status & RC5_PACKET_STATUS_EMPTY)
    {
      RC5TmpPacket.status &= (uint8_t)~RC5_PACKET_STATUS_EMPTY;
    }
    else
    {
      if (pulse <= RC5_2T_TIME)
      {
        /* Bit determination by the falling edge */
        tmpLastBit = RC5_logicTableFallingEdge[RC5TmpPacket.lastBit][pulse];
        IR_RC5_modifyLastBit (tmpLastBit);
      }
      else
      {
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
static uint8_t IR_RC5_GetPulseLength (uint16_t pulseLength)
{
  /* Valid bit time */
  if ((pulseLength > RC5MinT) && (pulseLength < RC5MaxT))
  {
    /* We've found the length */
    return (RC5_1T_TIME);	/* Return the correct value */
  }
  else if ((pulseLength > RC5Min2T) && (pulseLength < RC5Max2T))
  {
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
static void IR_RC5_modifyLastBit(tRC5_lastBitType bit)
{
  if (bit != RC5_NAN)
  {
    if (RC5TmpPacket.lastBit != RC5_INV)
    {
      /* Restore the last bit */
      RC5TmpPacket.lastBit = bit;

      /* Insert one bit into the RC5 Packet */
      IR_RC5_WriteBit(RC5TmpPacket.lastBit);
    }
    else
    {
      IR_RC5_ResetPacket();
    }
  }
}

/**
  * @brief  Insert one bit into the final data word.
  * @param  bitVal: bit value 'RC5_ONE' or 'RC5_ZER'
  * @retval None
  */
static void IR_RC5_WriteBit(uint8_t bitVal)
{
  /* First convert RC5 symbols to ones and zeros */
  if (bitVal == RC5_ONE)
  {
    bitVal = 1;
  }
  else if (bitVal == RC5_ZER)
  {
    bitVal = 0;
  }
  else
  {
    IR_RC5_ResetPacket();
    return;
  }

  /* Write this particular bit to data field */
  RC5TmpPacket.data |=  bitVal;

  /* Test the bit number determined */
  if (RC5TmpPacket.bitCount != 0)  /* If this is not the last bit */
  {
    /* Shift the data field */
    RC5TmpPacket.data = RC5TmpPacket.data << 1;
    /* And decrement the bitCount */
    RC5TmpPacket.bitCount--;
  }
  else
  {
   RC5FrameReceived = YES;
  }
}

/**
  * @brief  Identify TIM clock
  * @param  None
  * @retval Timer clock
  */
static uint32_t TIM_GetCounterCLKValue(void)
{
  uint32_t apbprescaler = 0, apbfrequency = 0;
  uint32_t timprescaler = ir_htim->Init.Prescaler;

  uint32_t HCLK_Frequency = HAL_RCC_GetHCLKFreq();
  uint32_t iFlashLatency = 0;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct,&iFlashLatency);

  if((ir_htim->Instance == TIM1)
     || (ir_htim->Instance == TIM8)
#ifdef TIM15
	 || (ir_htim->Instance == TIM15)
#endif
#ifdef TIM16
     || (ir_htim->Instance == TIM16)
#endif
#ifdef TIM17
	 || (ir_htim->Instance == TIM17)
#endif
	 )
  {
    /* Get the clock prescaler of APB2 */
    apbfrequency = HCLK_Frequency/RCC_ClkInitStruct.APB2CLKDivider;
    apbprescaler = RCC_ClkInitStruct.APB2CLKDivider;
  }
  else
	  if((ir_htim->Instance == TIM2) || (ir_htim->Instance == TIM3) || (ir_htim->Instance == TIM4)
			|| (ir_htim->Instance == TIM5)
			  || (ir_htim->Instance == TIM6)
			  || (ir_htim->Instance == TIM7)
			  )
  {
    /* Get the clock prescaler of APB1 */
    apbfrequency = HCLK_Frequency/RCC_ClkInitStruct.APB1CLKDivider;
    apbprescaler = RCC_ClkInitStruct.APB1CLKDivider;
  }


  /* If APBx clock div >= 4   need apbfrequency * 2 */
  if (apbprescaler >= 2)
  {
    return ((apbfrequency * 2)/(timprescaler + 1));
  }
  else
  {
    return (apbfrequency/(timprescaler+ 1));
  }
}

void IR_RC5_Decode(IR_Frame_TypeDef *rc5_frame)
{
  /* If frame received */
  if(RC5FrameReceived != NO)
  {
    /* RC5 frame field decoding */
    rc5_frame->FieldBit = (RC5TmpPacket.data >> 12) & 0x1;
    rc5_frame->ToggleBit = (RC5TmpPacket.data >> 11) & 0x1;
    rc5_frame->Address = (RC5TmpPacket.data >> 6) & 0x1F;
    rc5_frame->Command = (RC5TmpPacket.data) & 0x3F;

    /* Default state */
    RC5FrameReceived = NO;
    IR_RC5_ResetPacket();

#ifdef USE_LCD
#endif
  }
}


/**
  * @brief  Set the incoming packet structure to default state.
  * @param  None
  * @retval None
  */
void IR_RC5_ResetPacket(void)
{
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

void TIMx_IRQHandler (void)
{
	ICValue1=0;
	ICValue2=0;

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
