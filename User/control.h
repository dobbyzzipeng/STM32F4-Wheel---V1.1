#ifndef _CONTROL_H_
#define _CONTROL_H_
#include <stdint.h>

#define DEAD_ZONE 300
#define LSB 2275.55f//外部电机轴转过1度，内部磁编码器要走的值
#define RAD_TO_DU 57.32f
#define RATIO	50
#define _90_ANGLE	204800

typedef enum{
	STANDBY = 0,
	REMOTE_CTR = 1,
	GO_HOME = 2,
//	PICK_MODE = 3,
	CV_CTR = 4,
}E_WORK_MODE;

typedef enum{
	NONE = 0,
	ONE,
	TWO,
	THREE,
	FOUR,
	FIVE,
	SIX,
	DONE,//OK
}E_PICKPLANE;

typedef enum{
	CHARGE = 0,
	PICK_PLANE_OUT,
	RELEASE_PLANE_OUT,
	GOHOME_WITHOUT_PLANE,
	CHARGE_WITHOUT_PLANE,
	GET_OUT_FIND_PLANE,
	CVRTK_FIND_PLANE,
	PICK_PLANE_IN,
	RELEASE_PLANE_IN,
	TASK_OK_CHARGE,
}E_AGVTASK;

typedef struct{
	uint8_t g_plane_flag1;
	uint8_t g_plane_flag2;
	uint8_t last_plane_flag1;
	uint8_t last_plane_flag2;
}T_PLANE;
extern T_PLANE T_Plane;

extern uint8_t g_agv_work_mode;
extern uint8_t g_rgb_cmd;
extern uint8_t g_auto_pick_state;
extern uint8_t g_release_flag;

void Auto_Release_Plane(void);
void Auto_Pick_Plane(void);
void AGV_Work_Mode_Choose(void);
void RGB_Ctr_Task(void);
void Plane_Check_Task(void);
void Auto_GoHome_Task(void);
int Limit(int datain,int max);
#endif
