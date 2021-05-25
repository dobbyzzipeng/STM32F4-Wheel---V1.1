#include "gps.h" 
#include "led.h" 
#include "bsp_delay.h" 								   
#include "usart.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "bsp_usart.h"

//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������							  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//�õ�GPGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}   
}
//����GLGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GLGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GLGSV");
	len=p1[7]-'0';								//�õ�GLGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->glonassnum=NMEA_Str2num(p1+posx,&dx);//GLONASS ��������
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GLGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ�GLONASS���Ǳ��
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ�GLONASS�������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ�GLONASS���Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->glonassmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ�GLONASS���������
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��GLGSV��Ϣ
	}
}
//����BDGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ��ı������ݻ������׵�ַ
void NMEA_BDGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$BDGSV");
	len=p1[7]-'0';								//�õ�BDGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ�������������
	if(posx!=0XFF)gpsx->beidou_svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$BDGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��BDGSV��Ϣ
	}   
}
//����GNGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS/�������ݻ������׵�ַ
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//����GNGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS/�������ݻ������׵�ַ
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=(uint8_t*)strstr((const char *)buf,"$GNGSA");
	posx=NMEA_Comma_Pos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
char data_mode;
//����GNRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS/�������ݻ������׵�ַ
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;
	#if (USE_GPS_TYPE == USE_ATK_MODULE)
	p1=(uint8_t*)strstr((const char *)buf,"GNRMC");//"$GNRMC",������&��GNRMC�ֿ������,��ֻ�ж�GPRMC.
	#elif (USE_GPS_TYPE == USE_M8N_MODULE)
	p1=(uint8_t*)strstr((const char *)buf,"$GNRMC");//"$GNRMC",������&��GNRMC�ֿ������,��ֻ�ж�GPRMC.
	#endif
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		#if (USE_GPS_TYPE == USE_ATK_MODULE)
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//ת��Ϊ��
		#elif (USE_GPS_TYPE == USE_M8N_MODULE)
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//ת��Ϊ��
		#endif
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		
		#if (USE_GPS_TYPE == USE_ATK_MODULE)		
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//ת��Ϊ�� 
		#elif (USE_GPS_TYPE == USE_M8N_MODULE)
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//ת��Ϊ�� 
		#endif
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
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
		else if(temp=='F') //��
		{
			data_mode = 'F';
			rtk.datamode=3;
		}
		else if(temp=='R')//����
		{
			data_mode = 'R';
			rtk.datamode=4;
		}
		else if(temp=='N')//����
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
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}
//����GNVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS/�������ݻ������׵�ַ
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��
	}
}  
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS/�������ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV����  �ɼ�GPS��λ��Ϣ !
	#if (USE_GPS_TYPE == USE_ATK_MODULE)
	NMEA_BDGSV_Analysis(gpsx,buf);	//BDGSV����  �ɼ�������λ��Ϣ !
	#elif (USE_GPS_TYPE == USE_M8N_MODULE)
	NMEA_GLGSV_Analysis(gpsx,buf);  //GLGSV����  �ɼ�GLONASS��λ��Ϣ !
	#endif
	NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA����  GPS/������λ��Ϣ	!!!
	#if (USE_GPS_TYPE == USE_ATK_MODULE)
	NMEA_GNGSA_Analysis(gpsx,buf);	//GNGSA����  ��ǰ������Ϣ  ����û����
	#endif	
	NMEA_GNRMC_Analysis(gpsx,buf);	//GNRMC����  �Ƽ���λ��Ϣ!!!
	NMEA_GNVTG_Analysis(gpsx,buf);	//GNVTG����  �����ٶ���Ϣ						
									//GNGLL����  ���������Ϣ/��λ������Ϣ
									//GNZDA����  ��ǰʱ����Ϣ
}


nmea_msg gpsx; 											//GPS��Ϣ
__align(4) uint8_t dtbuf[50];   								//��ӡ������
//const uint8_t*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode�ַ��� 

void Gps_Msg_Show(void)
{
 	float tp;		    	 
	tp=gpsx.longitude;	   
	sprintf((char *)dtbuf,"Longitude:%.5f %1c   ",tp/=100000,gpsx.ewhemi);	//�õ������ַ���  
	tp=gpsx.latitude;	   
	sprintf((char *)dtbuf,"Latitude:%.5f %1c   ",tp/=100000,gpsx.nshemi);	//�õ�γ���ַ���	 
	tp=gpsx.altitude;	   
 	sprintf((char *)dtbuf,"Altitude:%.1fm     ",tp/=10);	    			//�õ��߶��ַ���	 			   
	tp=gpsx.speed;	   
 	sprintf((char *)dtbuf,"Speed:%.3fkm/h     ",tp/=1000);		    		//�õ��ٶ��ַ���	 	 				    
//	if(gpsx.fixmode<=3)														//��λ״̬
//	{
//		sprintf((char *)dtbuf,"Fix Mode:%s",fixmode_tbl[gpsx.fixmode]);				   
//	}	 	
	
	sprintf((char *)dtbuf,"GPS+BD Valid satellite:%02d",gpsx.posslnum);	 		//���ڶ�λ��GPS������	    
	sprintf((char *)dtbuf,"GPS Visible satellite:%02d",gpsx.svnum%100);	 		//�ɼ�GPS������
	
	sprintf((char *)dtbuf,"BD Visible satellite:%02d",gpsx.beidou_svnum%100);	 		//�ɼ�����������
	
	sprintf((char *)dtbuf,"UTC Date:%04d/%02d/%02d   ",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date);	//��ʾUTC����	    
	sprintf((char *)dtbuf,"UTC Time:%02d:%02d:%02d   ",gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);	//��ʾUTCʱ��
}

RTK rtk;
//��ʾGPS��λ��Ϣ 
void Gps_Msg_Prf(void)
{
	double get_lon_atk = gpsx.longitude/10000000.0f;
	double get_lat_atk = gpsx.latitude/10000000.0f;
	if(get_lon_atk>90&&get_lon_atk<150&&get_lat_atk>5&&get_lat_atk<70){     //���˵��й�����
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
	if(USART4_RX_STA&0X8000)		//���յ�һ��������
	{
//		u1_printf("[RTK]rtk prase\r\n");
		rxlen=USART4_RX_STA&0X7FFF;	//�õ����ݳ���
		for(i=0;i<rxlen;i++)gps_data_buff[i]=USART4_RX_BUF[i];	   
		USART4_RX_STA=0;		   	//������һ�ν���
		gps_data_buff[i]=0;			//�Զ���ӽ�����
		GPS_Analysis(&gpsx,(uint8_t*)gps_data_buff);//�����ַ���
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
	R = 6378137; // ����뾶���ף�
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



