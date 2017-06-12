/**
  ******************************************************************************
  * @file    sirc_decode.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    08-March-2016
  * @brief   This file contains all the functions prototypes for the SIRC
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
#ifndef __SIRC_DECODE_H
#define __SIRC_DECODE_H


/** @addtogroup IR_REMOTE
  * @brief Infra-red remote control
  * @{
  */

/** @addtogroup SIRC_DECODE
  * @brief SIRC decode driver modules
  * @{
  */

/** @defgroup SIRC_Exported_Types
  * @{
  */

/**
  * @brief  SIRC frame structure
  */
typedef struct
{
  __IO uint8_t Command;   /*!< Command field */
  __IO uint8_t Address;   /*!< Address field */
}
SIRC_Frame_t;

/**
  * @brief  SIRC packet structure
  */
typedef struct
{
  uint8_t count;  /*!< Bit count */
  uint8_t status; /*!< Status */
  uint32_t data;  /*!< Data */
}
SIRC_packet_t;

/**
  * @}
  */

/** @defgroup SIRC_Exported_Functions
  * @{
  */
void Menu_SIRCDecode_Func(void);
void SIRC_ResetPacket(void);
void SIRC_DataSampling(uint32_t lowPulseLength, uint32_t wholePulseLength);
void SIRC_DeInit(void);
void SIRC_Init(void);
void SIRC_Decode(SIRC_Frame_t *ir_frame);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __SIRC_DECODE_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
