#ifndef _BMS_H_
#define _BMS_H_
#include "sys.h"
//���ȼ� + ����ID + ��λ����ַ+ BMS��ַ
#define PROPOTY 0x18	//���ȼ�

#define BMS_ADDR 0x01	//BMS��ַ
#define PC_ADDR	0x40//��λ����ַ

#define GET_TOTAL_SOC 0x90		//�ܵ���
#define GET_VOL 0x91		//���������͵�ѹ
#define GET_TEM 0x92		//�����������¶�
#define GET_MOS 0x93		//��ŵ硢MOS״̬
#define GET_MSG 0x94		//״̬��Ϣ
#define GET_CELL_VOL 0x95		//�����ѹ
#define GET_CELL_TEM 0x96		//�����¶�
#define GET_JUNHENG 0x97		//�������״̬
#define GET_ERROR 0x98		//��ع���״̬

typedef struct{

	float Voltage;
	float Current;
	float Soc;
  uint8_t state;
}Battery;
extern Battery Battery_Msg;

void Bms_Get_Soc(void);
#endif

