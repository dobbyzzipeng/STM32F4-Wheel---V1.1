#include "postion.h"
#include "rtk.h"
#include <math.h>

#define D (1.2f)
#define C_DEG2RAD_F (0.0174532925f)
T_POS Pos = {0};

static float yaw_corret(float yaw)
{ 
	float yaw_out = yaw;
	while ((yaw_out < -180.0f) || (yaw_out > 180.0f))
	{
		if (yaw_out > 180.0f) yaw_out -= 360.0f;
		else if (yaw_out < -180.0f) yaw_out += 360.0f;
	}
	return yaw_out;
}

static void get_Q_point_coordinate(T_POS *p)
{
	p->Q_x = p->drg_x + cosf(p->drg_ang * C_DEG2RAD_F) * D;
	p->Q_y = p->drg_y + sinf(p->drg_ang * C_DEG2RAD_F) * D;
}

static void get_vert_to_drg2Q(T_POS *p)
{
	float delta_p = 0.0f, delta_S = 0.0f;
	float sign = 0.0f;
	float PGx = 0.0f, PGy = 0.0f, QGx = 0.0f, QGy = 0.0f;
	p->agv2drg = sqrtf(powf(p->agv_x - p->drg_x,2) + powf(p->agv_y - p->drg_y,2));
	p->agv2Q = sqrtf(powf(p->agv_x - p->Q_x,2) + powf(p->agv_y - p->Q_y,2));

	delta_p = (p->agv2drg + p->agv2Q + D) / 2.0f;
	delta_S = sqrtf(delta_p * (delta_p - D) * (delta_p - p->agv2drg) * (delta_p - p->agv2Q));
	p->vert_to_drg2Q = 2.0f * delta_S / D;
	
	PGx = p->drg_x - p->agv_x;
	PGy = p->drg_y - p->agv_y;
	QGx = p->drg_x - p->Q_x;
	QGy = p->drg_y - p->Q_y;		
	sign = (PGx * QGy - QGx * PGy);
	
	if (sign < 0.0f) p->vert_to_drg2Q *= -1.0f;
}

void pos_analysis(double agv_lon,double agv_lat,double agv_ang,double drg_lon,double drg_lat,double drg_ang,T_POS *p)
{
	double x1 = 0,y1 = 0;
	double x2 = 0,y2 = 0;
	
	get_agv_pos(&x1,&y1,agv_lon,agv_lat);
	get_drg_pos(&x2,&y2,drg_lon,drg_lat);
	
	p->agv_x = x1;
	p->agv_y = y1;
	p->agv_ang_org = agv_ang;
	
	p->drg_x = x2;
	p->drg_y = y2;
	p->drg_ang_org = drg_ang;
	
	p->agv_ang = yaw_corret(-1.0f * p->agv_ang_org + 270.0f);
	p->drg_ang = yaw_corret(-1.0f * p->drg_ang_org + 90.0f);
	
	get_Q_point_coordinate(p);
	get_vert_to_drg2Q(p);
}

