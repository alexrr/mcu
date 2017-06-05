/*
 * ir_decode.h
 *
 *  Created on: 4 θών. 2017 γ.
 *      Author: alexr
 */

#ifndef IR_DECODE_H_
#define IR_DECODE_H_
#include "Project_Lib.h"

typedef enum { NO = 0, YES = !NO} StatusYesOrNo;

/**
  * @brief  RC5 frame structure
  */
typedef struct
{
	volatile uint8_t FieldBit;   /*!< Field bit */
	volatile uint8_t ToggleBit;  /*!< Toggle bit field */
	volatile uint8_t Address;    /*!< Address field */
	volatile uint8_t Command;    /*!< Command field */

} IR_Frame_TypeDef;
/**
  * @brief  RC5 packet structure
  */
typedef struct
{
	volatile uint16_t data;     /*!< RC5 data */
	volatile uint8_t  status;   /*!< RC5 status */
	volatile uint8_t  lastBit;  /*!< RC5 last bit */
	volatile uint8_t  bitCount; /*!< RC5 bit count */
} tRC5_packet;

enum RC5_lastBitType
{
  RC5_ZER,
  RC5_ONE,
  RC5_NAN,
  RC5_INV
};

typedef enum RC5_lastBitType tRC5_lastBitType;

extern TIM_HandleTypeDef* ir_htim;


#define RC5_1T_TIME                          0x00
#define RC5_2T_TIME                          0x01
#define RC5_WRONG_TIME                       0xFF
#define RC5_TIME_OUT_US                      3600
#define RC5_T_US                             900     /*!< Half bit period */
#define RC5_T_TOLERANCE_US                   300     /*!< Tolerance time */
#define RC5_NUMBER_OF_VALID_PULSE_LENGTH     2
#define RC5_PACKET_BIT_COUNT                 13      /*!< Total bits */

/* Packet struct for reception*/
#define RC5_PACKET_STATUS_EMPTY              1 << 0


#endif /* IR_DECODE_H_ */
