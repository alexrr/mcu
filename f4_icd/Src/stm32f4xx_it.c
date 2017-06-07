/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "../App/Project_Includes.h"
extern QueueHandle_t Queue_user_input;
extern QueueHandle_t Queue_ir_cap;
static uint8_t USART2_IRQHandler_buf[2];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim10;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}


/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
//  for(int i=0;i<7;i++){
//	  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//	  HAL_Delay(20);
//  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	HAL_StatusTypeDef res = HAL_UART_Receive_IT(&huart1, USART2_IRQHandler_buf,
			1);

//	HAL_UART_Transmit_IT(&huart1, USART2_IRQHandler_buf, 1);
	if (res == HAL_OK) {
		if (Queue_user_input != NULL) {
			xQueueSendFromISR(Queue_user_input, USART2_IRQHandler_buf,
					&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* Captured Value */
__IO uint32_t uwIC1Value = 0;
__IO uint32_t uwIC2Value = 0;
/* Duty Cycle Value */
__IO uint32_t uwDutyCycle = 0;
/* Frequency Value */
__IO uint32_t uwFrequency = 0;
TIM_CapValue_TypeDef IC_val;

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if(__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC1)!=RESET){
		HAL_GPIO_WritePin(PCAP1_GPIO_Port,PCAP1_Pin,1);
		uwIC1Value = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
  }
  if(__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC2)!=RESET){

  }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  /* USER CODE END TIM2_IRQn 1 */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == ir_htim->Instance) {
//		if((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)==)0&(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)==0)){
//			HAL_GPIO_TogglePin(PCAP1_GPIO_Port,PCAP1_Pin);
//		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			HAL_GPIO_WritePin(PCAP1_GPIO_Port,PCAP1_Pin,1);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			/* Get the Input Capture value */
			HAL_GPIO_WritePin(PCAP1_GPIO_Port,PCAP1_Pin,0);
			uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			if (uwIC2Value != 0) {
				/* Duty cycle computation */
				uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1))
						* 100) / uwIC2Value;

				/* uwFrequency computation
				 TIM3 counter clock = (RCC_Clocks.HCLK_Frequency) */
				uwFrequency = uwIC2Value - HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			} else {
				uwDutyCycle = 0;
				uwFrequency = 0;
			}
		}

		if (Queue_ir_cap != NULL) {
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			IC_val.Channel = htim->Channel;
			switch (htim->Channel) {
			case HAL_TIM_ACTIVE_CHANNEL_1:
				IC_val.ICvalue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				IC_val.ICvalue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				break;
			default:
				IC_val.ICvalue = 0;
			}

			xQueueSendFromISR(Queue_ir_cap, &IC_val, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
