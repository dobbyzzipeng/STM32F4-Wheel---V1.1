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
	ONE_1,
	ONE_11,
	TWO,
	THREE,
	FOUR,
	FIVE,
	SIX,
	SEVEN,
	EIGHT,
	DONE,//OK
}E_PICKPLANE;//复杂动作顺序

typedef enum{
	TASK_OK_CHARGE = 0,
	PICK_PLANE_OUT,//1
	GOBACK_FOR_SPACE,//2
	GOHOME_WITHOUT_PLANE,//3
	CHARGE_WITHOUT_PLANE,//4
	GET_OUT_FIND_PLANE,//5
	GOFRONT_FOR_SPACE,//6
	CVRTK_FIND_PLANE,//7
	FIIND_BLACK_LINE,//8
	PICK_PLANE_IN,//9
	RELEASE_PLANE_IN,//10
}E_AGVTASK;//AGV自动任务顺序

typedef enum{
	INIT=0,
	OUT = 1,
	IN = -1,
	REBACK = 3,
	GOFRONT = 4,
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

enum{
	FB = 0,
	TURN = 1,
	LR = 2,
};

enum{
	SONG = 0,
	LIMIT = 1,
};

enum{
	STOP = 0,
	GO,
	TURN_LEFT,
	TURN_RIGHT,
	RUN_LEFT,
	RUN_RIGHT,
};

typedef enum{
	AGV_ARM_UNKNOW,
	AGV_ARM_OPENING,
	AGV_ARM_CLOSING,
	AGV_ARM_OPEN,
	AGV_ARM_CLOSE,
	AGV_ARM_ERR,
} agv_arm_state_t;

extern uint8_t g_agv_work_mode;
extern uint8_t g_rgb_cmd;
extern uint8_t g_pick_state;
extern uint8_t g_release_flag;
extern uint8_t g_agv_task_state;
extern uint8_t g_find_blackline_state;
extern uint8_t g_cvrtk_findplane_state;
extern uint8_t g_dragonfish_flag;
extern uint8_t g_agv_arm_flag;

void Auto_Release_Plane(int8_t agvdir);
void Auto_Pick_Plane(uint8_t flag,int8_t dir);
uint8_t is_release(void);
uint8_t is_close(void);
void AGV_Work_Mode_Choose(void);
void RGB_Ctr_Task(void);
void Plane_Check_Task(void);
void Auto_FollowLine_Task(int8_t dir,uint8_t plane);
void FollowLine_process(int8_t dir,uint8_t lrlimit);
void Follow_Line_Clear(void);
void Auto_CVRTK_FindPlane(void);
void Auto_CVRTK_FindState_Clear(void);
void Auto_Find_BlackLine(void);
void Auto_Find_BlackLine_State_Clear(void);
int Limit(int data,int max);
void Pick_Plane_Ctr_Task(int16_t x,int16_t y,int16_t w);
void chassic_control_task(int16_t x,int16_t y,int16_t w);
void Chassic_Motor_Ctr(int16_t sp,float w,int8_t flag);
uint8_t Chassic_Pid_Ctr(float diserr,float omgerr,int8_t turn);
void agv_open_arm(void);
#endif
