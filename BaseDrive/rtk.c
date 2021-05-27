#include "rtk.h"
#include <math.h>
#include "gps.h"
#include "bsp_usart.h"

#define FE_WGS84    (1.0/298.257223563)
#define RE_WGS84    6378137.0
#define MYPI          3.1415926535897932  /* pi */
#define D2R         (MYPI/180.0)

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

double rtk_dis_analysis(double lon1,double lat1,double lon2,double lat2)
{
	double h1,h2;
    unsigned long long temp;
    double posbuf1[3]={0};
    double posbuf2[3]={0};
    double ecef1[3]={0};
    double ecef2[3]={0};
    double len=0;
//     printf("*******输入两点经纬度坐标计算距离*******\r\n");
//    printf("请输坐标1：纬度，经度，高度\r\n");
//    scanf("%lf,%lf,%lf",&lon1,&lat1,&h1);
//    printf("输入的坐标1：纬度=%lf 经度=%lf 高度=%lf\n",lon1,lat1,h1);

//    printf("请输坐标2：纬度，经度，高度\r\n");
//    scanf("%lf,%lf,%lf",&lon2,&lat2,&h2);
//    printf("输入的坐标2：纬度=%lf 经度=%lf 高度=%lf\n",lon2,lat2,h2);

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
//    u1_printf("dis is:%fm",len);

    return len;
}

void RTK_TO_XY(double lon0,double lat0,double lonx,double laty,float *x,float *y,double *dis)
{
	double ang = 0;
	#if 0
	*dis = gps_get_distance(lon0,lat0,lonx,laty);
	ang = gps_get_angle(lon0,lat0,lonx,laty);
	*x = dis*cos(ang);
	*y = dis*sin(ang);
	#else
	*x = gps_get_distance(lon0,lat0,lonx,lat0);
	*y = gps_get_distance(lon0,lat0,lon0,laty);
	*dis = gps_get_distance(lon0,lat0,lonx,laty);
	#endif
//	u1_printf("x:%.3f\t,y:%.3f\r\n",*x,*y);
}

