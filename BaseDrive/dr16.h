#ifndef _DR16_H
#define _DR16_H
#include "stm32f4xx.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP 	((uint8_t)1)
#define RC_SW_MID 	((uint8_t)3)
#define RC_SW_DOWN  ((uint8_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
/* ----------------------- Data Struct ------------------------------------- */
#define USE_SBUS 1//use futaba remote controller
#define USE_DBUS 0//use dji remote controller
#define USE_DBUS_FUTABA 0
#define USE_10CH_FUTABA 0//是否使用10通道FUTABA遥控器,否则使用14通道FUTABA遥控器
#define USE_10CH_AT9S	0
#define USE_10CH_T8FB	1
#define DEBUG_PC  0

#define left_Switch_UP  3
#define left_Switch_DOWN  12
#define left_Switch_4G  0

#define left_Wheel_UP  1
#define left_Wheel_DOWN  9

#define right_Switch_UP  13
#define right_Switch_MID  15
#define right_Switch_DOWN  1

#define right_Wheel_UP  1
#define right_Wheel_DOWN  4

void RC_Init(void);
extern volatile unsigned short Channel_0,Channel_1,Channel_2,Channel_3,Channel_7,Channel_8;
extern uint8_t Init_Mid_RC_flag;
extern volatile uint8_t Switch_left,Switch_right,pump_switch,fire_switch,last_swleft,last_swright;
//extern int Mouse_X,Mouse_Y,Mouse_Z,Mouse_Left,Mouse_Right,KeyBoard_value;
extern volatile short left_horizontal,left_vertical,right_horizontal,right_vertical;
extern volatile unsigned short left_Switch,left_Wheel,right_Switch,right_Wheel,last_right_Wheel;

extern int rc_data_count;
extern volatile uint8_t rc_data_flag;
#endif
