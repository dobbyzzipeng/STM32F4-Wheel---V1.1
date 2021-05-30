#ifndef _RTK_H_
#define _RTK_H_
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void llh2ecef(const double *pos, double *r);
double DmToDd(unsigned long long data);
double rtk_dis_analysis(double lon1,double lat1,double lon2,double lat2);
void RTK_TO_XY(double lon0,double lat0,double lonx,double laty,double *x,double *y,double *dis);
void DM_TO_DD(double lon1,double lat1,double *lon2,double *lat2);
void get_drg_pos(double *x,double *y, double lon,double lat);
void get_agv_pos(double *x,double *y, double lon,double lat);
#endif
