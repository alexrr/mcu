/**
  ******************************************************************************
  * @file    rc5_decode.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    08-March-2016
  * @brief   This file contains all the functions prototypes for the RC5
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
#ifndef __RC5_DECODE_H
#define __RC5_DECODE_H

/** @addtogroup IR_REMOTE
  * @{
  */

/** @addtogroup RC5_DECODE
  * @brief RC5 driver modules
  * @{
  */

/** @defgroup RC5_Public_Types
  * @{
  */

/**
  * @brief  RC5 frame structure
  */
typedef struct
{
  __IO uint8_t FieldBit;   /*!< Field bit */
  __IO uint8_t ToggleBit;  /*!< Toggle bit field */
  __IO uint8_t Address;    /*!< Address field */
  __IO uint16_t Command;    /*!< Command field */
}
RC5_Frame_t;

/**
  * @brief  RC5 packet structure
  */
typedef struct
{
  __IO uint32_t data;     /*!< RC5 data */
  __IO uint8_t  status;   /*!< RC5 status */
  __IO uint8_t  lastBit;  /*!< RC5 last bit */
  __IO uint8_t  bitCount; /*!< RC5 bit count */
}
RC5_Packet_t;

/**
  * @brief  RC5 previous bit state type
  */
typedef enum
{
  RC5_ZER = 0,
  RC5_ONE,
  RC5_NAN,
  RC5_INV
} RC5_lastBit_t;

/**
  * @}
  */

/** @defgroup RC5_Exported_Functions
  * @{
  */
void Menu_RC5Decode_Func(void);
void RC5_ResetPacket(void);
void RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge);
void RC5_DeInit(void);
void RC5_Init(IR_handle_type_def *IRH);
void RC5_Decode(RC5_Frame_t *rc5_frame);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __RC5_DECODE_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
