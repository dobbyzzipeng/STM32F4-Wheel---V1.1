#include "postion.h"
#include "rtk.h"

#define DRGON_FISH_LON0	118.7968950//×óÉÏ½Ç  WGS84
#define DRGON_FISH_LAT0	30.8603061
#define DRGON_FISH_LON1	118.7968906//ÓÒÉÏ½Ç  WGS84
#define DRGON_FISH_LAT1	30.8602769

#define AGV_LON0	118.796947666//×óÉÏ½Ç  WGS84
#define AGV_LAT0	30.86024978
#define AGV_LON1	118.796949668//ÓÒÉÏ½Ç	WGS84
#define AGV_LAT1	30.8602148816

T_POS Pos = {0};
void pos_analysis(double agv_lon,double agv_lat,double agv_ang,double drg_lon,double drg_lat,double drg_ang,T_POS *p)
{
	double x1 = 0,y1 = 0,dis1 = 0;
	double x2 = 0,y2 = 0,dis2 = 0;
	
	RTK_TO_XY(AGV_LON0,AGV_LAT0,agv_lon,agv_lat,&x1,&y1,&dis1);
	RTK_TO_XY(DRGON_FISH_LON0,DRGON_FISH_LAT0,drg_lon,drg_lat,&x2,&y2,&dis2);
	p->agv_x = x1;
	p->agv_y = y1;
	p->agv_ang = agv_ang;
	p->agv_dis = dis1;
	
	p->drg_x = x2;
	p->drg_y = y2;
	p->drg_ang = drg_ang;
	p->drg_dis = dis2;
}

