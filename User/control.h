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
}E_PICK;

typedef struct{
	uint8_t g_plane_flag1;
	uint8_t g_plane_flag2;
	uint8_t last_plane_flag1;
	uint8_t last_plane_flag2;
}T_PLANE;
extern T_PLANE T_Plane;//检测飞机触碰开关

extern uint8_t g_agv_work_mode;
extern uint8_t g_rgb_cmd;
extern uint8_t g_auto_pick_state;
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
int Limit(int datain,int max);
void Pick_Plane_Ctr_Task(int16_t x,int16_t y,int16_t w);
void chassic_control_task(int16_t x,int16_t y,int16_t w);

#endif
