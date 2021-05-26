#ifndef _COMUNICATION_H_
#define _COMUNICATION_H_
#include <stdint.h>

typedef struct{
	uint8_t cv_data_flag;
	uint8_t cv_cmd;
	uint8_t rgb_cmd;
	uint8_t eject_cmd;
	float agv_spx;
	float agv_spy;
	float agv_spw;
	float pick_spcatch;
	float pick_sppp;
	float pick_spupdown;
}T_CV;
extern T_CV CV;

typedef enum{
	NOTASK=0,
	RUN,
	PAUSE,
	CANCEL,
}E_AUTO_MODE;//上位机控制指令表

typedef struct
{
	uint8_t agv_cmd;
	uint8_t last_agv_cmd;
	uint8_t agv_charge;
	uint8_t agv_pause;
	uint8_t agv_cancel;
}T_CMD;
extern T_CMD AGV_CMD;

void AGV_Data_Upload(void);
void DownLoad_prase(uint8_t buf[]);
void NX_Data_prase(uint8_t buf[]);
void NX_Data_Return(void);
#endif

