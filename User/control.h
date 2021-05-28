#ifndef _CONTROL_H_
#define _CONTROL_H_
#include <stdint.h>

#define DEAD_ZONE 300
#define LSB 2275.55f//外部电机轴转过1度，内部磁编码器要走的值
#define RAD_TO_DU 57.32f
#define RATIO	50
#define _90_ANGLE	204800
#define	myabs(x)	((x>0)?(x):(-(x)))

typedef enum{
	STANDBY = 0,
	REMOTE_CTR = 1,
	AUTO_CTR = 2,
	CV_CTR = 3,
	TASK_CANCEL = 4,
}E_WORK_MODE;//AGV受控状态

typedef enum{
	NONE = 0,
	ONE,
	TWO,
	THREE,
	FOUR,
	FIVE,
	SIX,
	DONE,//OK
}E_PICKPLANE;//复杂动作顺序

typedef enum{
	TASK_OK_CHARGE = 0,
	PICK_PLANE_OUT,
	GOBACK_FOR_SPACE,
	GOHOME_WITHOUT_PLANE,
	CHARGE_WITHOUT_PLANE,
	GET_OUT_FIND_PLANE,
	CVRTK_FIND_PLANE,
	PICK_PLANE_IN,
	RELEASE_PLANE_IN,
}E_AGVTASK;//AGV自动任务顺序

typedef enum{
	INIT=0,
	OUT,
	IN,
}E_DIR;//巡线方向

typedef enum{
	NOPLANE= 0,
	WITHPLANE,
}E_PLANE;//是否有飞机

typedef enum{
	CLOSE=0,
	FIND,
}E_PICK;//夹具状态

typedef struct{
	uint8_t g_plane_flag1;
	uint8_t g_plane_flag2;
	uint8_t last_plane_flag1;
	uint8_t last_plane_flag2;
}T_PLANE;
extern T_PLANE T_Plane;//检测飞机触碰开关

typedef enum{
	BLACK = 0,
	RED = 1,
	GREEN = 2,
	BLUE = 3,
	WHITE = 4,
	YELLOW = 5,
	POURPLE = 6,
	PINK = 7,
	ORANGE = 8,
	WARM = 9,
}E_RGBCOLOR_LIST;//RGB颜色表

typedef enum{
	NOFLASH = 0,
	_1HZ = 10,
	_2HZ = 20,
	_5HZ = 50,
	_10HZ = 100,
	_15HZ = 150,
	_20HZ = 200,
}E_FLASH_HZ;//RGB flash hz

typedef enum{
	_BALCK = 0,
	_BREATH = 0X01,
	_FLASH = 0X10,
	_ON = 0X11,
}E_RGB_MODE;//RGB MODE

extern uint8_t g_agv_work_mode;
extern uint8_t g_rgb_cmd;
extern uint8_t g_pick_state;
extern uint8_t g_release_flag;
extern uint8_t g_agv_task_state;

void Auto_Release_Plane(uint8_t agvdir);
void Auto_Pick_Plane(uint8_t flag,uint8_t dir);
void AGV_Work_Mode_Choose(void);
void RGB_Ctr_Task(void);
void Plane_Check_Task(void);
void Auto_FollowLine_Task(uint8_t dir,uint8_t plane);
void FollowLine_process(uint8_t dir);
void Follow_Line_Clear(void);
void Auto_CVRTK_FindPlane(void);
int Limit(int data,int max);
void Pick_Plane_Ctr_Task(int16_t x,int16_t y,int16_t w);
void chassic_control_task(int16_t x,int16_t y,int16_t w);
void Chassic_Motor_Ctr(int16_t sp,float w,int8_t flag);
uint8_t Chassic_Pid_Ctr(float diserr,float omgerr,int8_t flag);
#endif
