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

typedef struct
{
	uint8_t agv_charge;
	uint8_t agv_start;
	uint8_t agv_back;
}T_CMD;
extern T_CMD AGV_CMD;

void Data_Upload(void);
void DownLoad_prase(uint8_t buf[]);
void NX_Data_prase(uint8_t buf[]);
void NX_Data_Return(void);
#endif

