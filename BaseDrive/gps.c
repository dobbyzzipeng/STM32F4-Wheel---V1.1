#include "gps.h" 
#include "bsp_delay.h"				   
//#include "usart.h"		   
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "math.h"
#include "bsp_usart.h"

//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}

void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;
	p1=(uint8_t*)strstr((const char *)buf,"$GPRMC");
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;
	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");	
	posx=NMEA_Comma_Pos(p1,2);	//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'	
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为°
	}
	posx=NMEA_Comma_Pos(p1,3);
	if(posx!=0XFF)
	{
		gpsx->nshemi=*(p1+posx);//南纬还是北纬
	}
 	posx=NMEA_Comma_Pos(p1,4);	//得到经度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'	
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,5);								
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);//东经还是西经
	posx=NMEA_Comma_Pos(p1,6);
	if(posx!=0XFF)
	{
		temp = *(p1+posx);
		if(temp==4)
		{
			gpsx->fixmode = 4;//唯一解
		}
		else if(temp==5)
		{
			gpsx->fixmode = 5;//多个解
		}
		else if(temp==1)
		{
			gpsx->fixmode = 1;//多个解
		}
		else{
			gpsx->fixmode = 0;//无效
		}
	}
}

void NMEA_GPTRA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1;
	uint8_t dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	p1=(uint8_t*)strstr((const char *)buf,"$GPTRA");	
	posx=NMEA_Comma_Pos(p1,2);
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->ang = temp;
	}
}
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GNRMC_Analysis(gpsx,buf);
	NMEA_GPTRA_Analysis(gpsx,buf);
	NMEA_GPGGA_Analysis(gpsx,buf);
}


nmea_msg gpsx = {0}; 											//GPS信息
RTK agvrtk = {0},drgrtk = {0};
//显示GPS定位信息 
void Gps_Msg_Prf(void)
{
	double get_lon_atk = gpsx.longitude/10000000.0f;
	double get_lat_atk = gpsx.latitude/10000000.0f;
//	if(get_lon_atk>90&&get_lon_atk<150&&get_lat_atk>5&&get_lat_atk<70){     //过滤到中国区域
		agvrtk.lon=get_lon_atk;
		agvrtk.lat=get_lat_atk;
//	}
//	agvrtk.gps_speed = gpsx.speed/3600.0f;//m/s
//	agvrtk.gps_num = gpsx.svnum%100;
	agvrtk.hour=gpsx.utc.hour+8;
	agvrtk.min=gpsx.utc.min;
	agvrtk.sec=gpsx.utc.sec;
	agvrtk.month=gpsx.utc.month;
	agvrtk.date=gpsx.utc.date;
	agvrtk.year=gpsx.utc.year;
	agvrtk.ang=gpsx.ang/100.0f;
	agvrtk.datamode=gpsx.fixmode;
//	u1_printf("fixmode:%d\t lng:%lf\t lat:%lf\t gps_num:%d\t hour:%d\t min:%d\t sec:%d\r\n",
//	rtk.fixmode,rtk.lon_atk,rtk.lat_atk,rtk.gps_num,rtk.hour,rtk.min,rtk.sec);
}

uint8_t gps_data_prase(uint8_t buf[1024])
{
	GPS_Analysis(&gpsx,buf);//分析字符串
	Gps_Msg_Prf();
	return 0;
}
/*********************************************************************/
#define PI 3.1415926535897932f
#define EARTH_RADIUS 6378.137f 

double radian(double d)
{
	return d * PI / 180.0; 
}

double toRadian(double point)
{
	return (point*PI)/180.0;
}

double toDegree(double point)
{
	return point*180.0/ PI;
}

int toBearing(double angle)
{
	double bearing = toDegree(angle)+ 360.0;
	return fmod(bearing, 360);
}


#define REGTORAG 0.01745329
#define RAGTOREG 57.2957805
double gps_get_distance(double lon1, double lat1, double lon2,double lat2)
{
	double a, b, R;
	R = 6378137; // 地球半径（米）
	lat1 = lat1 * REGTORAG;
	lat2 = lat2 * REGTORAG;
	a = lat1 - lat2;
	b = (lon1 - lon2) * REGTORAG;
	double d;
	double sa2, sb2;
	sa2 = sin(a / 2.0);
	sb2 = sin(b / 2.0);
	d = 2* R* sin(sqrt(sa2 * sa2 + cos(lat1)* cos(lat2) * sb2 * sb2));
	return d;
}


double gps_get_angle(double longtitude_start, double latitude_start, double longtitude_end, double latitude_end)
{
	double rad_latitude_start = toRadian(latitude_start);

	double rad_latitude_end = toRadian(latitude_end);

	double deltaLongtitude = toRadian(longtitude_end)- toRadian(longtitude_start);

	double x = sin(deltaLongtitude)*cos(rad_latitude_end);

	double y = cos(rad_latitude_start)*sin(rad_latitude_end)-sin(rad_latitude_start)*cos(rad_latitude_end)*cos(deltaLongtitude);

	return toBearing(atan2(x, y));
}



