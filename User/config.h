#ifndef _CONFIG_H_
#define _CONFIG_H_
#include "sys.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "can1.h"
#include "can2.h"
#include "pwm.h"
#include "led.h"
#include "dr16.h"
#include "ZL5SERVO.h"
#include <math.h>
#include "PickPlane.h"
#include "bms.h"
#include "hmcsensor.h"
#include "stdio.h"
#include "stdint.h"
#include "comunication.h"
#include "control.h"

#define USE_IAP 0
#define Version 1

//------------------------------------------------------------------------------
#define VERINFO_ADDR_BASE  (0x8009F00) // 版本信息在FLASH中的存放地址
//const char Hardware_Ver[] __attribute__((at(VERINFO_ADDR_BASE + 0x00)))  = "Hardware: 1.0.0";
//const char Firmware_Ver[] __attribute__((at(VERINFO_ADDR_BASE + 0x20)))  = "Firmware: 1.0.0";
//const char Compiler_Date[] __attribute__((at(VERINFO_ADDR_BASE + 0x40))) = "Date: "__DATE__;
//const char Compiler_Time[] __attribute__((at(VERINFO_ADDR_BASE + 0x60))) = "Time: "__TIME__;
#define	myabs(x)	((x>0)?(x):(-(x)))

//typedef enum{
//	STANDBY = 0,
//	REMOTE_CTR = 1,
//	GO_HOME = 2,
////	PICK_MODE = 3,
//	CV_CTR = 4,
//}E_WORK_MODE;

//typedef enum{
//	NONE = 0,
//	ONE,
//	TWO,
//	THREE,
//	FOUR,
//	FIVE,
//	SIX,
//	DONE,//OK
//}E_PICKPLANE;

//typedef enum{
//	CHARGE = 0,
//	PICK_PLANE_OUT,
//	RELEASE_PLANE_OUT,
//	GOHOME_WITHOUT_PLANE,
//	CHARGE_WITHOUT_PLANE,
//	GET_OUT_FIND_PLANE,
//	CVRTK_FIND_PLANE,
//	PICK_PLANE_IN,
//	RELEASE_PLANE_IN,
//	TASK_OK_CHARGE,
//}E_AGVTASK;
#endif
