/*
 * motor_cmd.exec.h
 *
 *  Created on: 11 мая 2017 г.
 *      Author: alexr
 */

#ifndef MOTOR_CMD_EXEC_H_
#define MOTOR_CMD_EXEC_H_
#include "Project_Includes.h"

#define mask_MOTOR1 0x0100
#define mask_MOTOR2 0x0200
#define mask_MOTOR3 0x0400
#define mask_MOTOR4 0x0800
#define DIR_MOTOR_FORWARD 0x1000
#define DIR_MOTOR_BACKWARD 0x2000
#define MOTOR_SPEED_MASK 0x00FF


extern uint32_t Motors_mask[4];
void PrintPromtUART(UART_HandleTypeDef *huart);
void PrintMotorPinAll(UART_HandleTypeDef *huart);
void PrintMotorPinP1(UART_HandleTypeDef *huart);
void PrintMotorPinP2(UART_HandleTypeDef *huart);
void PrintMotorPinIdx(uint8_t i,UART_HandleTypeDef *huart);
void PrintTimBaseAll(UART_HandleTypeDef *huart);
void PrintTimBaseP1(UART_HandleTypeDef *huart);
void PrintTimBaseP2(UART_HandleTypeDef *huart);
void PrintTimBase1(uint8_t i,UART_HandleTypeDef *huart);
uint32_t ParseControlCmd(const char *cmd_str,UART_HandleTypeDef *huart);
void PrepareMotorCtrl(uint8_t m_idx,GPIO_TypeDef *_GPIO_Forward,GPIO_TypeDef *_GPIO_Backward,uint16_t  _PinForward,uint16_t  _PinBackward,TIM_HandleTypeDef *_tim,uint8_t _ctrl_CCx,uint16_t  _MaxPWM_value);
void SetTimPWM_CCR(TIM_TypeDef *_tim,uint8_t _ccx,uint16_t value);
uint16_t GetTimPWM_CCR(TIM_TypeDef *_tim,uint8_t _ccx);
void Start_PWM(TIM_HandleTypeDef *_tim, uint8_t _ccx);
void ErrorCMD(UART_HandleTypeDef *huart, uint16_t ErrorCode,uint16_t chr);
void ExecMotorCmd(uint32_t m_cmd);
uint16_t GetShortState(uint8_t i,uint8_t *str);

//----------------------
typedef struct
{
	GPIO_TypeDef *GPIO_Forward;
	GPIO_TypeDef *GPIO_Backward;
	uint16_t  PinForward;
	uint16_t  PinBackward;
	TIM_HandleTypeDef* tim;
	uint8_t ctrl_CCx;
	uint16_t  MaxPWM_value;
} MotorControl_TypeDef;
//----------------------
extern MotorControl_TypeDef MotorCtrl[4];

#endif /* MOTOR_CMD_EXEC_H_ */
