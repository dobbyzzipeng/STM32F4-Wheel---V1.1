#ifndef __GPS_H
#define __GPS_H	 
#include "sys.h"

#define FE_WGS84    (1.0/298.257223563)
#define RE_WGS84    6378137.0
#define MYPI          3.1415926535897932  /* pi */
#define D2R         (MYPI/180.0)//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ
__packed typedef struct  
{										    
 	uint8_t num;		//���Ǳ��
	uint8_t eledeg;	//��������
	uint16_t azideg;	//���Ƿ�λ��
	uint8_t sn;		//�����		   
}nmea_slmsg; 
//GLONASS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ
__packed typedef struct  
{	
 	uint8_t num;		//���Ǳ��
	uint8_t eledeg;	//��������
	uint16_t azideg;	//���Ƿ�λ��
	uint8_t sn;		//�����		   
}glonass_nmea_slmsg; 
//���� NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ
__packed typedef struct  
{	
 	uint8_t beidou_num;		//���Ǳ��
	uint8_t beidou_eledeg;	//��������
	uint16_t beidou_azideg;	//���Ƿ�λ��
	uint8_t beidou_sn;		//�����		   
}beidou_nmea_slmsg; 

//UTCʱ����Ϣ
__packed typedef struct  
{										    
 	uint16_t year;	//���
	uint8_t month;	//�·�
	uint8_t date;	//����
	uint8_t hour; 	//Сʱ
	uint8_t min; 	//����
	uint8_t sec; 	//����
}nmea_utc_time;   	   
//NMEA 0183 Э����������ݴ�Žṹ��
__packed typedef struct  
{										    
 	uint8_t svnum;					//�ɼ�GPS������
	uint8_t glonassnum;				//�ɼ�GLONASS������
	uint8_t beidou_svnum;			//�ɼ�����������
	nmea_slmsg slmsg[12];		//���12��GPS����
	glonass_nmea_slmsg glonassmsg[12];
	beidou_nmea_slmsg beidou_slmsg[12];	//���������12�ű�������
	nmea_utc_time utc;			//UTCʱ��
	uint32_t latitude;				//γ�� ������100000��,ʵ��Ҫ����100000
	uint8_t nshemi;					//��γ/��γ,N:��γ;S:��γ				  
	uint32_t longitude;		
	uint32_t ang;	//���� ������100000��,ʵ��Ҫ����100000
	uint8_t ewhemi;					//����/����,E:����;W:����
	uint8_t gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.				  
 	uint8_t posslnum;				//���ڶ�λ��GPS������,0~12.
 	uint8_t possl[12];				//���ڶ�λ�����Ǳ��
	uint8_t fixmode;					//��λ����:4��Ψһ��λ��0����5��Ч���߶����λ
	uint16_t pdop;					//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t hdop;					//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t vdop;					//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0 

	int altitude;			 	//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m	 
	uint16_t speed;					//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ	 
}nmea_msg;
extern nmea_msg gpsx; 	
//RTK��Ϣ
typedef struct  
{
	uint8_t hour; 	//Сʱ
	uint8_t min; 	//����
	uint8_t sec; 	//����
	uint8_t date; 	//Сʱ
	uint8_t month; 	//����
	uint16_t year; 	//����
	double lon;
	double lat;
	double ang;
	float gps_speed;
	uint8_t gps_num;
	uint8_t datamode;//��λ״̬ 4Ψһ��λ��5��ⶨλ��1,0 ��Ч
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

 



