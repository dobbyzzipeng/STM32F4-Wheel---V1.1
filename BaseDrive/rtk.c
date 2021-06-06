#include "rtk.h"
#include <math.h>
#include "gps.h"
#include "bsp_usart.h"

#define AGV_A_LAT (30.8602958)//A靠近机巢，B远离机巢	A->B X正方向，垂直方向，Y
#define AGV_A_LNG (118.7968536)
#define AGV_B_LAT (30.8602958)
#define AGV_B_LNG (118.7968896)

#define DRG_A_LAT (30.8602733)
#define DRG_A_LNG (118.7968389)
#define DRG_B_LAT (30.8602735)
#define DRG_B_LNG (118.7968785)

void llh2ecef(const double *pos, double *r)
{
    double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);
    double e2=FE_WGS84*(2.0-FE_WGS84),v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);

    r[0]=(v+pos[2])*cosp*cosl;
    r[1]=(v+pos[2])*cosp*sinl;
    r[2]=(v*(1.0-e2)+pos[2])*sinp;
}

double DmToDd(unsigned long long data)
{
    double result;
	unsigned int dd;
	unsigned int mm;

    dd     = data / 1000000000;
    mm     = data % 1000000000;
    result = (double)mm/10000000/60;
    result = dd + result;

	return result;
}
//输入度分，NEMA0183刚解析完的数据
double rtk_dis_analysis(double lon1,double lat1,double lon2,double lat2)
{
	double h1,h2;
    unsigned long long temp;
    double posbuf1[3]={0};
    double posbuf2[3]={0};
    double ecef1[3]={0};
    double ecef2[3]={0};
    double len=0;

    //dm转dd
    temp = lon1 * 10000000;
    posbuf1[0] = DmToDd(temp);
    temp = lat1 * 10000000;
    posbuf1[1] = DmToDd(temp);
    posbuf1[2] = h1;
    //printf("%lf,%lf,%lf\r\n",posbuf1[0],posbuf1[1],posbuf1[2]);

    temp = lon2 * 10000000;
    posbuf2[0] = DmToDd(temp);
    temp = lat2 * 10000000;
    posbuf2[1] = DmToDd(temp);
    posbuf2[2] = h2;
    //printf("%lf,%lf,%lf\r\n",posbuf2[0],posbuf2[1],posbuf2[2]);

    posbuf1[0] = posbuf1[0] *D2R;
    posbuf1[1] = posbuf1[1] *D2R;
    llh2ecef(posbuf1, ecef1);
    //printf("%lf,%lf,%lf\r\n",ecef1[0],ecef1[1],ecef1[2]);

    posbuf2[0] = posbuf2[0] *D2R;
    posbuf2[1] = posbuf2[1] *D2R;
    llh2ecef(posbuf2, ecef2);
    //printf("%lf,%lf,%lf\r\n",ecef2[0],ecef2[1],ecef2[2]);

    len = sqrt(pow(ecef2[0] - ecef1[0],2) + pow(ecef2[1] - ecef1[1],2) + pow(ecef2[2] - ecef1[2],2));
    len = sqrt(pow(ecef2[0] - ecef1[0],2) + pow(ecef2[1] - ecef1[1],2));
//    u5_printf("dis is:%fm",len);

    return len;
}

void RTK_TO_XY(double lon0,double lat0,double lonx,double laty,double *x,double *y,double *dis)
{
	#if 0
	double ang = 0;
	*dis = gps_get_distance(lon0,lat0,lonx,laty);
	ang = gps_get_angle(lon0,lat0,lonx,laty);
	*x = dis*cos(ang);
	*y = dis*sin(ang);
	#else
	*y = gps_get_distance(lon0,lat0,lonx,lat0);
	*x = gps_get_distance(lon0,lat0,lon0,laty);
	*dis = gps_get_distance(lon0,lat0,lonx,laty);
	#endif
//	u5_printf("x:%.3f\t,y:%.3f\r\n",*x,*y);
}

void get_agv_pos(double *x,double *y, double lon,double lat)
{
	double x_norm = 0.0, x_sign = 0.0, y_norm = 0.0, y_sign = 0.0;
	double L = 0.0, l0 = 0.0, l1 = 0.0;
	double p = 0.0, S = 0.0;
	double x_ap = 0.0, y_ap = 0.0, x_ab = 0.0, y_ab = 0.0;
	
    L  = gps_get_distance(AGV_A_LNG,AGV_A_LAT,AGV_B_LNG,AGV_B_LAT);
    l0 = gps_get_distance(AGV_A_LNG,AGV_A_LAT,lon,lat);
    l1 = gps_get_distance(lon,lat,AGV_B_LNG,AGV_B_LAT);
	
    p = (L + l0 + l1) / 2.0;
    S = sqrt(p * (p - L) * (p - l0) * (p - l1));
    y_norm = 2.0 * S / L;
	
	x_ap = lon - AGV_A_LNG;
	y_ap = lat - AGV_A_LAT;
	x_ab = AGV_B_LNG - AGV_A_LNG;
	y_ab = AGV_B_LAT - AGV_A_LAT;
	
    y_sign = x_ap * y_ab - x_ab * y_ap;
	
	if (y_sign > 0.0)
	{
		*y = -1.0 *y_norm;
	}
	else
	{
		*y = y_norm;
	}
	
	x_norm = sqrt(l0 * l0 - y_norm * y_norm);
	x_sign = x_ap * x_ab + y_ab * y_ap;
	
	if (x_sign > 0.0)
	{
		*x = x_norm;
	}
	else
	{
		*x = -1.0f * x_norm;
	}
}

void get_drg_pos(double *x,double *y, double lon,double lat)
{
	double x_norm = 0.0, x_sign = 0.0, y_norm = 0.0, y_sign = 0.0;
	double L = 0.0, l0 = 0.0, l1 = 0.0;
	double p = 0.0, S = 0.0;
	double x_ap = 0.0, y_ap = 0.0, x_ab = 0.0, y_ab = 0.0;
	
    L  = gps_get_distance(DRG_A_LNG,DRG_A_LAT,DRG_B_LNG,DRG_B_LAT);
    l0 = gps_get_distance(DRG_A_LNG,DRG_A_LAT,lon,lat);
    l1 = gps_get_distance(lon,lat,DRG_B_LNG,DRG_B_LAT);
	
    p = (L + l0 + l1) / 2.0;
    S = sqrt(p * (p - L) * (p - l0) * (p - l1));
    y_norm = 2.0 * S / L;
	
	x_ap = lon - DRG_A_LNG;
	y_ap = lat - DRG_A_LAT;
	x_ab = DRG_B_LNG - DRG_A_LNG;
	y_ab = DRG_B_LAT - DRG_A_LAT;
	
    y_sign = x_ap * y_ab - x_ab * y_ap;
	
	if (y_sign > 0.0)
	{
		*y = -1.0 * y_norm;
	}
	else
	{
		*y = y_norm;
	}
	
	x_norm = sqrt(l0 * l0 - y_norm * y_norm);
	x_sign = x_ap * x_ab + y_ab * y_ap;
	
	if (x_sign > 0.0)
	{
		*x = x_norm;
	}
	else
	{
		*x = -1.0f * x_norm;
	}
}

void DM_TO_DD(double lon1,double lat1,double *lon2,double *lat2)
{
	unsigned long long temp;
	temp = lon1 * 10000000;
    *lon2 = DmToDd(temp);
	temp = lat1 * 10000000;
    *lat2 = DmToDd(temp);
}


