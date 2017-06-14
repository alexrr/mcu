/*
 * Project_Lib.c
 *
 *  Created on: 9 θών. 2017 γ.
 *      Author: alexey.radchuk
 */
#include "Project_Lib.h"
#include <stdio.h>
#include <stdarg.h>

//QueueHandle_t Queue_user_input;
//QueueHandle_t Queue_ir_cap;
#define MAX_strlen 100

static UART_HandleTypeDef* huart_log;
static uint8_t str_buf[MAX_strlen];

void SetLogDev(){
// TODO: implement SetLogDev()
}

void InitLogDev(UART_HandleTypeDef* new_log_h){
	huart_log = new_log_h;
	__HAL_UART_ENABLE_IT(huart_log, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(huart_log, UART_IT_TXE);

}

uint8_t* LogOut(uint8_t* _str, ...){
	int slen;
	int count=0;
    va_list argptr;
    va_start(argptr, _str);
    slen = vsnprintf((char*)str_buf,MAX_strlen, (char*)_str, argptr);
    va_end(argptr);
    if(slen>MAX_strlen){
    	slen=MAX_strlen;
    }
    if(huart_log->hdmatx!=NULL){
    	HAL_UART_Transmit_DMA(huart_log, str_buf, slen);
    }
    else{
    	HAL_UART_Transmit(huart_log, str_buf, slen,0x1ff);
    }
	return str_buf;
}

void LogOutCh(uint8_t ch){
	uint8_t _ch[2];
	_ch[0]=ch;
	HAL_UART_Transmit(huart_log, _ch, 1,0x1ff);
}

void LogOutFix(uint8_t* pstr,uint16_t len){
	HAL_UART_Transmit(huart_log, pstr, len,0x1ff);
}

void LogOutStr(uint8_t* pstr){
    if(huart_log->hdmatx!=NULL){
    	HAL_UART_Transmit_DMA(huart_log, pstr, strlen(pstr));
    }
    else{
    	HAL_UART_Transmit(huart_log, pstr, strlen(pstr),0x1ff);
    }

}
