#ifndef _CONFIG_H_
#define _CONFIG_H_
#include "sys.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "config.h"
#include "can1.h"
#include "can2.h"
#include "pwm.h"
#include "led.h"
#include "dr16.h"
#include "ZL5SERVO.h"
#include <math.h>
#include "PickPlane.h"

#define USE_IAP 0
#define MAIN_CAN_ID 0x200
#define Cabin_CAN_ID 0x300
#define ANCHOR_Index 0
#define Version 1

#define	myabs(x)	((x>0)?(x):(-(x))) 
#endif
