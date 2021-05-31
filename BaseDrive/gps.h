#ifndef __GPS_H
#define __GPS_H	 
#include "sys.h"

#define FE_WGS84    (1.0/298.257223563)
#define RE_WGS84    6378137.0
#define MYPI          3.1415926535897932  /* pi */
#define D2R         (MYPI/180.0)//GPS NMEA-0183协议重要参数结构体定义 
//卫星信息
__packed typedef struct  
{										    
 	uint8_t num;		//卫星编号
	uint8_t eledeg;	//卫星仰角
	uint16_t azideg;	//卫星方位角
	uint8_t sn;		//信噪比		   
}nmea_slmsg; 
//GLONASS NMEA-0183协议重要参数结构体定义 
//卫星信息
__packed typedef struct  
{	
 	uint8_t num;		//卫星编号
	uint8_t eledeg;	//卫星仰角
	uint16_t azideg;	//卫星方位角
	uint8_t sn;		//信噪比		   
}glonass_nmea_slmsg; 
//北斗 NMEA-0183协议重要参数结构体定义 
//卫星信息
__packed typedef struct  
{	
 	uint8_t beidou_num;		//卫星编号
	uint8_t beidou_eledeg;	//卫星仰角
	uint16_t beidou_azideg;	//卫星方位角
	uint8_t beidou_sn;		//信噪比		   
}beidou_nmea_slmsg; 

//UTC时间信息
__packed typedef struct  
{										    
 	uint16_t year;	//年份
	uint8_t month;	//月份
	uint8_t date;	//日期
	uint8_t hour; 	//小时
	uint8_t min; 	//分钟
	uint8_t sec; 	//秒钟
}nmea_utc_time;   	   
//NMEA 0183 协议解析后数据存放结构体
__packed typedef struct  
{										    
 	uint8_t svnum;					//可见GPS卫星数
	uint8_t glonassnum;				//可见GLONASS卫星数
	uint8_t beidou_svnum;			//可见北斗卫星数
	nmea_slmsg slmsg[12];		//最多12颗GPS卫星
	glonass_nmea_slmsg glonassmsg[12];
	beidou_nmea_slmsg beidou_slmsg[12];	//暂且算最多12颗北斗卫星
	nmea_utc_time utc;			//UTC时间
	uint32_t latitude;				//纬度 分扩大100000倍,实际要除以100000
	uint8_t nshemi;					//北纬/南纬,N:北纬;S:南纬				  
	uint32_t longitude;		
	uint32_t ang;	//经度 分扩大100000倍,实际要除以100000
	uint8_t ewhemi;					//东经/西经,E:东经;W:西经
	uint8_t gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.				  
 	uint8_t posslnum;				//用于定位的GPS卫星数,0~12.
 	uint8_t possl[12];				//用于定位的卫星编号
	uint8_t fixmode;					//定位类型:4：唯一定位，0或者5无效或者多个定位
	uint16_t pdop;					//位置精度因子 0~500,对应实际值0~50.0
	uint16_t hdop;					//水平精度因子 0~500,对应实际值0~50.0
	uint16_t vdop;					//垂直精度因子 0~500,对应实际值0~50.0 

	int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m	 
	uint16_t speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时	 
}nmea_msg;
extern nmea_msg gpsx; 	
//RTK信息
typedef struct  
{
	uint8_t hour; 	//小时
	uint8_t min; 	//分钟
	uint8_t sec; 	//秒钟
	uint8_t date; 	//小时
	uint8_t month; 	//分钟
	uint16_t year; 	//秒钟
	double lon;
	double lat;
	double ang;
	float gps_speed;
	uint8_t gps_num;
	uint8_t datamode;//定位状态 4唯一定位，5多解定位，1,0 无效
	uint8_t good_ok;

	uint8_t res0;
	uint8_t res1;
}RTK; 

extern RTK agvrtk,drgrtk;

int NMEA_Str2num(uint8_t *buf,uint8_t*dx);
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPTRA_Analysis(nmea_msg *gpsx,uint8_t *buf);

uint8_t ascii_to_num(uint8_t ascii);
double parse_str_to_num(uint8_t buf[],uint8_t len);
uint8_t gps_data_prase(uint8_t *buf);
uint8_t drgrtk_prase(uint8_t *buf,uint8_t len);
void Gps_Msg_Prf(void);
double gps_get_distance(double lon1, double lat1, double lon2,double lat2);
double gps_get_angle(double lng1,double lat1, double lng2, double lat2) ;
#endif

 



