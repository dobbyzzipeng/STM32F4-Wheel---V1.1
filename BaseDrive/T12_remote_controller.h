#ifndef _T12_REMOTE_CONTROLLER_H_
#define _T12_REMOTE_CONTROLLER_H_
#include "stdint.h"
/***********´®¿Ú°æ±¾*********/
typedef enum{
	LEFT_STATE = 0X418,
	MID_STATE = 0X5DC,
	RIGHT_STATE = 0X79A,
}E_SWITCH_STATE;

typedef enum{
	BUTTON_OFF = 0X418,
	BUTTON_BLINK  = 0X5DC,
	BUTTON_ON = 0X79A,
}E_FUNBUTTON_STATE;

typedef struct{
	uint16_t CH_X2;//1000~1500~2000
	uint16_t CH_Y2;
	uint16_t CH_Y1;
	uint16_t CH_X1;
	uint16_t LEFT_SWITCH;
	uint16_t RIGHT_SWITCH;
	uint16_t FUN_A_BUTTON;
	uint16_t FUN_B_BUTTON;
	uint16_t LEFT_WHEEL;
	uint16_t RIGHT_WHEEL;
}T_T12_DATA;

extern T_T12_DATA T12_DATA;
void T12_remote_controller_prase(uint8_t buf[100]);

#endif
