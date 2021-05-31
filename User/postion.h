#ifndef _POSTION_H_
#define _POSTION_H_
#include <stdio.h>

typedef struct{
	float agv_x;
	float agv_y;
	float agv_ang_org;
	float agv_ang;

	float drg_x;
	float drg_y;
	float drg_ang_org;
	float drg_ang;
	
	float Q_x;
	float Q_y;
	
	float vert_to_drg2Q;
	float agv2drg;
	float agv2Q;
	
}T_POS;
extern T_POS Pos;
void pos_analysis(double agv_lon,double agv_lat,double agv_ang,double drg_lon,double drg_lat,double drg_ang,T_POS *p);
#endif
