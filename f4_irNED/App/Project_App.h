/*
 * Project_App.h
 *
 *  Created on: 11 θών. 2017 γ.
 *      Author: alexr
 */

#ifndef PROJECT_APP_H_
#define PROJECT_APP_H_
#include "Project_HAL.h"
#include "Project_Lib.h"
#include "lcd.h"
#include "DateTime.h"
#include "ir_common.h"
#include "rc5_decode.h"
#include "sirc_decode.h"
#include "NEC_Decode.h"

extern __IO FlagStatus DownStatus;
extern __IO FlagStatus UpStatus;
extern __IO FlagStatus SelStatus;
extern __IO FlagStatus LeftStatus;
extern __IO FlagStatus RightStatus;
extern __IO FlagStatus KeyStatus;


#endif /* PROJECT_APP_H_ */
