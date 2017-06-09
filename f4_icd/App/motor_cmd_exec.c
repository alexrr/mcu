#include "motor_cmd_exec.h"
#include <string.h>
char __promt[10] = "\n\rhi>\0x0";
uint32_t Motors_mask[4] = { mask_MOTOR1, mask_MOTOR2, mask_MOTOR3, mask_MOTOR4 };
MotorControl_TypeDef MotorCtrl[4];

void PrintPromt() {
	LogOutFix((uint8_t *) __promt, strlen(__promt));
}

uint32_t ParseControlCmd(const char *cmd_str) {
	int l;
	uint32_t cmd = 0;
	if (cmd_str[0] == 'P') {
		switch (cmd_str[1]) {
		case 'A':
			PrintMotorPinAll();
			PrintTimBaseAll();
			break;
		case '1':
			PrintMotorPinP1();
			PrintTimBaseP1();
			break;
		case '2':
			PrintMotorPinP2();
			PrintTimBaseP2();
			break;
		default:
			LogOut("\n\r_cmd err");
			PrintMotorPinP1();
			PrintTimBaseP1();
			break;
		}
		goto end;
	}
	if (cmd_str[0] == 'M') {
		LogOut("\n\r decode char %x, %lx", cmd_str[1], cmd);

		if (cmd_str[1] == 'A') {
			cmd |= (mask_MOTOR1 | mask_MOTOR2 | mask_MOTOR3 | mask_MOTOR4);
		} else {
			if (cmd_str[1] > 0x30 && cmd_str[1] < 0x35) {
				cmd |= Motors_mask[cmd_str[1] - 0x31];
			} else {
				cmd = 0;
				ErrorCMD(1, cmd_str[1]);
			}
		}

		switch (cmd_str[2]) {
		case 'F':
			cmd |= DIR_MOTOR_FORWARD;
			LogOut("\n\r %x DIR_MOTOR_FORWARD, %lx", cmd_str[2],cmd);
			break;
		case 'B':
			cmd |= DIR_MOTOR_BACKWARD;
			LogOut("\n\r %x DIR_MOTOR_BACKWARD, %lx", cmd_str[2],cmd);
			break;
		default:
			cmd = 0;
			ErrorCMD(2, cmd_str[2]);
			goto end;
		}
		uint8_t ib;
		uint32_t speed = 0;
		for (ib = 3; ib < 7; ib++) {
			if (cmd_str[ib] >= '0' && cmd_str[ib] <= '9')
				speed = speed*10 + (cmd_str[ib]-0x30);
			else
				break;
		}
		if (ib < 4) {
			cmd = 0;
			ErrorCMD(3, cmd_str[ib]);
			goto end;
		}
		speed = MOTOR_SPEED_MASK & speed;
		LogOut("\n\r_speed=%lu", speed);

		cmd |= speed;

		LogOut("\n\r return cmd=%lX", cmd);
	} else {
		LogOut("\n\r_def cmd");
		PrintTimBaseP1();
		PrintMotorPinP1();
	}
	end: return cmd;
}

uint16_t GetShortState(uint8_t i,uint8_t *str){
	uint8_t p1 = HAL_GPIO_ReadPin(MotorCtrl[i].GPIO_Forward,
			MotorCtrl[i].PinForward);
	uint8_t p2 = HAL_GPIO_ReadPin(MotorCtrl[i].GPIO_Backward,
			MotorCtrl[i].PinBackward);
	uint32_t arr=MotorCtrl[i].tim->Instance->ARR;
	uint16_t speed = GetTimPWM_CCR(MotorCtrl[i].tim->Instance, MotorCtrl[i].ctrl_CCx);
	if(arr>1){
		speed = speed*100.0 / arr;
	}
	return (uint16_t)sprintf((char*)str,"M%1u F%1u B%1u S%3u",i+1,p1,p2,speed);
}

void PrintMotorPinAll() {
	PrintMotorPinP1();
	PrintMotorPinP2();
}

void PrintMotorPinIdx(uint8_t i) {
	int l;
	uint8_t p1 = HAL_GPIO_ReadPin(MotorCtrl[i].GPIO_Forward,
			MotorCtrl[i].PinForward);
	uint8_t p2 = HAL_GPIO_ReadPin(MotorCtrl[i].GPIO_Backward,
			MotorCtrl[i].PinBackward);
	LogOut("\n\rM%1d_F=%u \t M%1d_B=%u",i+1, p1, i+1, p2);
}
void PrintMotorPinP1() {
	PrintMotorPinIdx(0);
	PrintMotorPinIdx(1);
}
void PrintMotorPinP2() {
	PrintMotorPinIdx(2);
	PrintMotorPinIdx(3);
}

void PrintTimBaseAll() {
	for (uint8_t i = 0; i < 4; i++) {
		PrintTimBase1(i);
	}
}

void PrintTimBaseP1() {
	PrintTimBase1(0);
	PrintTimBase1(1);
}
void PrintTimBaseP2() {
	PrintTimBase1(2);
	PrintTimBase1(3);
}

void PrintTimBase1(uint8_t i) {
	uint32_t arr, psc;
	uint32_t ccr_ctrl;
	arr = MotorCtrl[i].tim->Instance->ARR;
	psc = MotorCtrl[i].tim->Instance->PSC;
	ccr_ctrl = GetTimPWM_CCR(MotorCtrl[i].tim->Instance, MotorCtrl[i].ctrl_CCx);
	LogOut("\n\rM%1u ARR=%lu PSC=%lu CC=%lu", i+1, arr, psc, ccr_ctrl);
}

void ErrorCMD(uint16_t ErrorCode, uint16_t chr) {
	int l;
	LogOut("\n\rError decode command:%X %x", ErrorCode, chr);
}

void PrepareMotorCtrl(uint8_t m_idx, GPIO_TypeDef *_GPIO_Forward,
		GPIO_TypeDef *_GPIO_Backward, uint16_t _PinForward,
		uint16_t _PinBackward, TIM_HandleTypeDef *_tim, uint8_t _ctrl_CCx,
		uint16_t _MaxPWM_value) {
	if (m_idx > 4)
		return;
	MotorCtrl[m_idx].GPIO_Forward = _GPIO_Forward;
	MotorCtrl[m_idx].GPIO_Backward = _GPIO_Backward;
	MotorCtrl[m_idx].PinForward = _PinForward;
	MotorCtrl[m_idx].PinBackward = _PinBackward;
	MotorCtrl[m_idx].tim = _tim;
	MotorCtrl[m_idx].ctrl_CCx = _ctrl_CCx;
	MotorCtrl[m_idx].MaxPWM_value = _MaxPWM_value;
	Start_PWM(MotorCtrl[m_idx].tim,MotorCtrl[m_idx].ctrl_CCx);
}

void ExecMotorCmd(uint32_t m_cmd) {
	uint8_t speed = 0xff & m_cmd;
	uint8_t PinState_Frw = GPIO_PIN_RESET;
	uint8_t PinState_Bak = GPIO_PIN_SET;
	if ((m_cmd & DIR_MOTOR_FORWARD)>0) {
		PinState_Frw = GPIO_PIN_SET;
		PinState_Bak = GPIO_PIN_RESET;
	}
	for (int i = 0; i < 4; i++) {
		if ((m_cmd & Motors_mask[i]) > 0) {
			SetTimPWM_CCR(MotorCtrl[i].tim->Instance, MotorCtrl[i].ctrl_CCx,
					(MotorCtrl[i].MaxPWM_value / 100) * speed);
			HAL_GPIO_WritePin(MotorCtrl[i].GPIO_Forward, MotorCtrl[i].PinForward,
					PinState_Frw);
			HAL_GPIO_WritePin(MotorCtrl[i].GPIO_Backward, MotorCtrl[i].PinBackward,
					PinState_Bak);
		}
	}

}

void Start_PWM(TIM_HandleTypeDef *_tim, uint8_t _ccx){
	switch (_ccx) {
	case 1:
		HAL_TIM_PWM_Start(_tim, TIM_CHANNEL_1);
		break;
	case 2:
		HAL_TIM_PWM_Start(_tim, TIM_CHANNEL_2);
		break;
	case 3:
		HAL_TIM_PWM_Start(_tim, TIM_CHANNEL_3);
		break;
	case 4:
		HAL_TIM_PWM_Start(_tim, TIM_CHANNEL_4);
		break;
	}
}

void SetTimPWM_CCR(TIM_TypeDef *_tim, uint8_t _ccx, uint16_t value) {
	switch (_ccx) {
	case 1:
		_tim->CCR1 = value;
		break;
	case 2:
		_tim->CCR2 = value;
		break;
	case 3:
		_tim->CCR3 = value;
		break;
	case 4:
		_tim->CCR4 = value;
		break;
	}
}

uint16_t GetTimPWM_CCR(TIM_TypeDef *_tim, uint8_t _ccx) {
	switch (_ccx) {
	case 1:
		return _tim->CCR1;
	case 2:
		return _tim->CCR2;
	case 3:
		return _tim->CCR3;
	case 4:
		return _tim->CCR4;
	}
	return 0;
}
