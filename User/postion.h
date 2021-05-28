#ifndef _POSTION_H_
#define _POSTION_H_
#include <stdio.h>

typedef struct{
	double agv_x;
	double agv_y;
	double agv_ang;
	double agv_dis;
	double drg_x;
	double drg_y;
	double drg_ang;
	double drg_dis;
}T_POS;
extern T_POS Pos;
void pos_analysis(double agv_lon,double agv_lat,double agv_ang,double drg_lon,double drg_lat,double drg_ang,T_POS *p);
#endif
