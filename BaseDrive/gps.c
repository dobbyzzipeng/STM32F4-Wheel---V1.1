#include "gps.h" 
#include "bsp_delay.h"				   
#include "usart.h"		   
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
	if(flen>7)flen=7;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}


double parse_str_to_num(uint8_t buf[],uint8_t len)
{
	uint8_t i = 0,res = 0,mark = 0;
	double weishu[8] = {0},xiaoshu[16] = {0};
	double val = 0;
	
	for(i=0;i<len;i++)
	{
		if(buf[i]=='.'){
			mark = i;
		}
	}
	for(i=0;i<len;i++){
		if(buf[i]>=0X30&&buf[i]<=0X39){
			res = ascii_to_num(buf[i]);
			if(i<mark){
				weishu[mark-i] = res;
			}
			else if(i>mark){
				xiaoshu[i-mark] = res;
			}
		}
	}
	val = weishu[5]*10000+weishu[4]*1000+ weishu[3]*100+weishu[2]*10+ weishu[1]\
	+xiaoshu[1]*0.1f+xiaoshu[2]*0.01f+xiaoshu[3]*0.001f+xiaoshu[4]*0.0001f+xiaoshu[5]*0.00001f+\
	+xiaoshu[6]*0.000001f+xiaoshu[7]*0.0000001f+xiaoshu[8]*0.00000001f+xiaoshu[9]*0.000000001f;
	return val;
}

uint8_t ascii_to_num(uint8_t ascii)
{
	uint8_t res = 0;
	switch(ascii)
	{
		case 0X30:
		res = 0;
		break;
		case 0X31:
		res = 1;
		break;
		case 0X32:
		res = 2;
		break;
		case 0X33:
		res = 3;
		break;
		case 0X34:
		res = 4;
		break;
		case 0X35:
		res = 5;
		break;
		case 0X36:
		res = 6;
		break;
		case 0X37:
		res = 7;
		break;
		case 0X38:
		res = 8;
		break;
		case 0X39:
		res = 9;
		break;
		default:
			return 0;
	}
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

extern void DM_TO_DD(double lon1,double lat1,double *lon2,double *lat2);;
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
static double lat = 0,lon = 0;
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;
	
	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");	
	posx=NMEA_Comma_Pos(p1,2);	//得到纬度
	if(posx!=0XFF)
	{
		lat = parse_str_to_num(p1+posx,12);
	}
	posx=NMEA_Comma_Pos(p1,3);
	if(posx!=0XFF)
	{
		gpsx->nshemi=*(p1+posx);//南纬还是北纬
	}
 	posx=NMEA_Comma_Pos(p1,4);	//得到经度
	if(posx!=0XFF)
	{
		lon = parse_str_to_num(p1+posx,13);
	}
	DM_TO_DD(lon,lat,&lon,&lat);
	posx=NMEA_Comma_Pos(p1,5);								
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);//东经还是西经
	posx=NMEA_Comma_Pos(p1,6);
	if(posx!=0XFF)
	{
		temp = NMEA_Str2num(p1+posx,&dx);
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
uint8_t gps_data_prase(uint8_t *buf)
{
	GPS_Analysis(&gpsx,buf);//分析字符串
	Gps_Msg_Prf();
	return 0;
}
//[30.8601707,118.7968119,90.00026775492921]
uint8_t drgrtk_prase(uint8_t *buf,uint8_t len)
{
	static double drg_lon = 0,drg_lat = 0,drg_ang = 0;
	uint8_t *head_ptr = NULL;
	uint8_t counter = 0u;
	uint8_t chars_num1 = 0u, chars_num2 = 0u, chars_num3 = 0u;
	uint8_t remain_len = len;
	
//	uint8_t RBUF[128] = {0};
//	sprintf(,"");
//	send_data_dma_u5(buf,100);
//	delay_ms(10);
	// 1. find head of msg and point to 1st num
	for(counter=0;counter < remain_len;counter++)
	{
		if(*(buf+counter)=='[')
		{
			head_ptr = buf + counter + 1; 
			remain_len = remain_len - counter - 1;
			break;
		}
		if(counter >= remain_len)
		{
			return 1u;
		}
	}

	// 2. find ',' between 1st and 2nd,
	//    get chars_num1 between '[' and ','
	for(counter = 0; counter < len; counter++)
	{
		if(*(head_ptr + counter)==',')
		{
			remain_len = remain_len - counter - 1;
			chars_num1 = counter;
			break;
		}
		if(counter >= remain_len)
		{
			return 1u;
		}
	}
	drg_lat = parse_str_to_num(head_ptr,chars_num1);
	//u5_printf("lat:%lf r:%d num:%d\r\n",drg_lat,remain_len,chars_num1);
    // 3. point to next num, next char at ','
	head_ptr = head_ptr + chars_num1 + 1;
	
	// 4. find ',' between 2nd and 3rd,
	//    get chars_num2 between ',' and ','
	for(counter = 0; counter < len; counter++)
	{
		if(*(head_ptr + counter)==',')
		{
			remain_len = remain_len - counter - 1;
			chars_num2 = counter;
			break;
		}
		if(counter >= remain_len)
		{
			return 1u;
		}
	}
	drg_lon = parse_str_to_num(head_ptr,chars_num2);
	//u5_printf("lon:%lf r:%d num:%d\r\n",drg_lon,remain_len,chars_num2);
    // 5. point to next num, next char at ','
	head_ptr = head_ptr + chars_num1 + 1;
	
	// 6. find ']' at end,
	//    get chars_num3 between ',' and ']'
	for(counter = 0; counter < len; counter++)
	{
		if(*(head_ptr + counter)==']')
		{
			remain_len = remain_len - counter - 1;
			chars_num3 = counter;
			break;
		}
		if(counter > remain_len)
		{
			return 1u;
		}
	}
	drg_ang = parse_str_to_num(head_ptr,chars_num3);	
	u5_printf("lat:%f lng:%f ang:%f r:%d n1:%d n2:%d n3:%d\r\n",drg_lat, drg_lon, drg_ang, remain_len, chars_num1, chars_num2, chars_num3);
	
	if(drg_lon>90&&drg_lon<150&&drg_lat>5&&drg_lat<60&&drg_ang>0.001f){
		drgrtk.ang = drg_ang;
		drgrtk.lon = drg_lon;
		drgrtk.lat = drg_lat;
		drgrtk.good_ok = 1;
	}
	else{
		drgrtk.good_ok = 0;
		return 1u;
	}
	
	return 0u;
}

void Gps_Msg_Prf(void)
{
	if(lon>90&&lon<150&&lat>5&&lat<60){
		agvrtk.lon = lon;
		agvrtk.lat = lat;
	}
	if(gpsx.ang>0&&gpsx.ang<36000){
		agvrtk.ang = gpsx.ang/100.0;
		agvrtk.good_ok = 1;
	}
	else{
		agvrtk.ang=0;
		agvrtk.good_ok = 0;//not good
	}
	agvrtk.hour=gpsx.utc.hour+8;
	agvrtk.min=gpsx.utc.min;
	agvrtk.sec=gpsx.utc.sec;
	agvrtk.month=gpsx.utc.month;
	agvrtk.date=gpsx.utc.date;
	agvrtk.year=gpsx.utc.year;
	agvrtk.datamode=gpsx.fixmode;
	
	drgrtk.ang = 279.4;
	drgrtk.lon = 118.7968784;
	drgrtk.lat = 30.8602223;
	drgrtk.good_ok = 1;
//	u5_printf("fixmode:%d\t lng:%lf\t lat:%lf\t gps_num:%d\t hour:%d\t min:%d\t sec:%d\r\n",
//	rtk.fixmode,rtk.lon_atk,rtk.lat_atk,rtk.gps_num,rtk.hour,rtk.min,rtk.sec);
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
	lat1 = lat1 * D2R;
	lat2 = lat2 * D2R;
	a = lat1 - lat2;
	b = (lon1 - lon2) * D2R;
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

