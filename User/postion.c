#include "postion.h"
#include "rtk.h"

#define DRGON_FISH_LON0	118.7968950//×óÉÏ½Ç  WGS84
#define DRGON_FISH_LAT0	30.8603061
#define DRGON_FISH_LON1	118.7968906//ÓÒÉÏ½Ç  WGS84
#define DRGON_FISH_LAT1	30.8602769

#define AGV_LON0	118.79685211//×óÉÏ½Ç  WGS84
#define AGV_LAT0	30.86029053
#define AGV_LON1	118.79689026//ÓÒÉÏ½Ç	WGS84
#define AGV_LAT1	30.86025620

#define AGV_LONA	118.79685211//×óÉÏ½Ç  WGS84
#define AGV_LATA	30.86029053
#define AGV_LONB	118.79689026//ÓÒÉÏ½Ç	WGS84
#define AGV_LATB	30.86025620

T_POS Pos = {0};
void pos_analysis(double agv_lon,double agv_lat,double agv_ang,double drg_lon,double drg_lat,double drg_ang,T_POS *p)
{
	double x1 = 0,y1 = 0,dis1 = 0;
	double x2 = 0,y2 = 0,dis2 = 0;
	
	get_agv_pos(&x1,&y1,agv_lon,agv_lat);
//	get_drg_pos(&x2,&y2,drg_lon,drg_lat);
	
	p->agv_x = x1;
	p->agv_y = y1;
	p->agv_ang = agv_ang;
	p->agv_dis = dis1;
	
	p->drg_x = x2;
	p->drg_y = y2;
	p->drg_ang = drg_ang;
	p->drg_dis = dis2;
}

