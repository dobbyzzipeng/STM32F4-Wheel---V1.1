#include "gps.h" 
#include "led.h" 
#include "bsp_delay.h" 								   
#include "usart.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
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
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}
//分析GLGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GLGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GLGSV");
	len=p1[7]-'0';								//得到GLGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->glonassnum=NMEA_Str2num(p1+posx,&dx);//GLONASS 卫星总数
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GLGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到GLONASS卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到GLONASS卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到GLONASS卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到GLONASS卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GLGSV信息
	}
}
//分析BDGSV信息
//gpsx:nmea信息结构体
//buf:接收到的北斗数据缓冲区首地址
void NMEA_BDGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$BDGSV");
	len=p1[7]-'0';								//得到BDGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见北斗卫星总数
	if(posx!=0XFF)gpsx->beidou_svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$BDGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个BDGSV信息
	}   
}
//分析GNGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//分析GNGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=(uint8_t*)strstr((const char *)buf,"$GNGSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
char data_mode;
//分析GNRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;
	#if (USE_GPS_TYPE == USE_ATK_MODULE)
	p1=(uint8_t*)strstr((const char *)buf,"GNRMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
	#elif (USE_GPS_TYPE == USE_M8N_MODULE)
	p1=(uint8_t*)strstr((const char *)buf,"$GNRMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
	#endif
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		#if (USE_GPS_TYPE == USE_ATK_MODULE)
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为°
		#elif (USE_GPS_TYPE == USE_M8N_MODULE)
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为°
		#endif
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		
		#if (USE_GPS_TYPE == USE_ATK_MODULE)		
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为° 
		#elif (USE_GPS_TYPE == USE_M8N_MODULE)
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为° 
		#endif
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,12);
	if(posx!=0XFF)
	{
		temp = *(p1+posx);
		if(temp=='A')
		{
			data_mode = 'A';
			rtk.datamode=1;
		}
		else if(temp=='D')
		{
			data_mode = 'D';
			rtk.datamode=2;
		}
		else if(temp=='F') //米
		{
			data_mode = 'F';
			rtk.datamode=3;
		}
		else if(temp=='R')//亚米
		{
			data_mode = 'R';
			rtk.datamode=4;
		}
		else if(temp=='N')//厘米
		{
			data_mode = 'N';
			rtk.datamode=5;
		}
		else{
			data_mode = 10;
			rtk.datamode=0;
		}
	}
	
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);	
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
//分析GNVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}
}  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析  可见GPS定位信息 !
	#if (USE_GPS_TYPE == USE_ATK_MODULE)
	NMEA_BDGSV_Analysis(gpsx,buf);	//BDGSV解析  可见北斗定位信息 !
	#elif (USE_GPS_TYPE == USE_M8N_MODULE)
	NMEA_GLGSV_Analysis(gpsx,buf);  //GLGSV解析  可见GLONASS定位信息 !
	#endif
	NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析  GPS/北斗定位信息	!!!
	#if (USE_GPS_TYPE == USE_ATK_MODULE)
	NMEA_GNGSA_Analysis(gpsx,buf);	//GNGSA解析  当前卫星信息  基本没有用
	#endif	
	NMEA_GNRMC_Analysis(gpsx,buf);	//GNRMC解析  推荐定位信息!!!
	NMEA_GNVTG_Analysis(gpsx,buf);	//GNVTG解析  地面速度信息						
									//GNGLL解析  大地坐标信息/定位地理信息
									//GNZDA解析  当前时间信息
}


nmea_msg gpsx; 											//GPS信息
__align(4) uint8_t dtbuf[50];   								//打印缓存器
//const uint8_t*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode字符串 

void Gps_Msg_Show(void)
{
 	float tp;		    	 
	tp=gpsx.longitude;	   
	sprintf((char *)dtbuf,"Longitude:%.5f %1c   ",tp/=100000,gpsx.ewhemi);	//得到经度字符串  
	tp=gpsx.latitude;	   
	sprintf((char *)dtbuf,"Latitude:%.5f %1c   ",tp/=100000,gpsx.nshemi);	//得到纬度字符串	 
	tp=gpsx.altitude;	   
 	sprintf((char *)dtbuf,"Altitude:%.1fm     ",tp/=10);	    			//得到高度字符串	 			   
	tp=gpsx.speed;	   
 	sprintf((char *)dtbuf,"Speed:%.3fkm/h     ",tp/=1000);		    		//得到速度字符串	 	 				    
//	if(gpsx.fixmode<=3)														//定位状态
//	{
//		sprintf((char *)dtbuf,"Fix Mode:%s",fixmode_tbl[gpsx.fixmode]);				   
//	}	 	
	
	sprintf((char *)dtbuf,"GPS+BD Valid satellite:%02d",gpsx.posslnum);	 		//用于定位的GPS卫星数	    
	sprintf((char *)dtbuf,"GPS Visible satellite:%02d",gpsx.svnum%100);	 		//可见GPS卫星数
	
	sprintf((char *)dtbuf,"BD Visible satellite:%02d",gpsx.beidou_svnum%100);	 		//可见北斗卫星数
	
	sprintf((char *)dtbuf,"UTC Date:%04d/%02d/%02d   ",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date);	//显示UTC日期	    
	sprintf((char *)dtbuf,"UTC Time:%02d:%02d:%02d   ",gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);	//显示UTC时间
}

RTK rtk;
//显示GPS定位信息 
void Gps_Msg_Prf(void)
{
	double get_lon_atk = gpsx.longitude/10000000.0f;
	double get_lat_atk = gpsx.latitude/10000000.0f;
	if(get_lon_atk>90&&get_lon_atk<150&&get_lat_atk>5&&get_lat_atk<70){     //过滤到中国区域
		rtk.lon_atk=get_lon_atk;
		rtk.lat_atk=get_lat_atk;
	}
	rtk.gps_speed = gpsx.speed/3600.0f;//m/s
	rtk.gps_num = gpsx.svnum%100;
	rtk.hour=gpsx.utc.hour+8;
	rtk.min=gpsx.utc.min;
	rtk.sec=gpsx.utc.sec;
	rtk.fixmode=gpsx.fixmode;
	u1_printf("fixmode:%d\t lng:%lf\t lat:%lf\t gps_num:%d\t hour:%d\t min:%d\t sec:%d\r\n",
	rtk.fixmode,rtk.lon_atk,rtk.lat_atk,rtk.gps_num,rtk.hour,rtk.min,rtk.sec);
}

uint16_t USART4_RX_STA = 0;
uint8_t gps_data_buff[1024] = {0};
uint8_t atk_gps_flag = 0,gps_num_flag = 0,atk_gps_count = 0;
uint8_t gps_data_analysis(void)
{
	uint16_t i,rxlen;
	if(USART4_RX_STA&0X8000)		//接收到一次数据了
	{
//		u1_printf("[RTK]rtk prase\r\n");
		rxlen=USART4_RX_STA&0X7FFF;	//得到数据长度
		for(i=0;i<rxlen;i++)gps_data_buff[i]=USART4_RX_BUF[i];	   
		USART4_RX_STA=0;		   	//启动下一次接收
		gps_data_buff[i]=0;			//自动添加结束符
		GPS_Analysis(&gpsx,(uint8_t*)gps_data_buff);//分析字符串
		Gps_Msg_Prf();
		atk_gps_flag = 1;
		atk_gps_count = 0;
	}
	return 0;
}

/*********************************************************************/
#define PI 3.1415926f
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



