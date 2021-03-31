#include "T12_remote_controller.h"

T_T12_DATA T12_DATA;
void T12_remote_controller_prase(uint8_t buf[100])
{
	if(buf[0] == 0X66 && buf[1] == 0X19)
	{
		T12_DATA.CH_X2 = ((uint16_t)buf[4]<<4)|((buf[5]&0XF0)>>4);
		T12_DATA.CH_Y2 = (((uint16_t)buf[5]&0X0F)<<8)|buf[6];
		T12_DATA.CH_Y1 = ((uint16_t)buf[7]<<4)|buf[8]>>4;
		T12_DATA.CH_X1 = (((uint16_t)buf[8]&0X0F)<<8)|buf[9];
		T12_DATA.LEFT_SWITCH = ((uint16_t)buf[10]<<4)|buf[11]>>4;
		T12_DATA.RIGHT_SWITCH = (((uint16_t)buf[11]&0X0F)<<8)|buf[12];
		T12_DATA.FUN_A_BUTTON = ((uint16_t)buf[13]<<4)|buf[14]>>4;
		T12_DATA.FUN_B_BUTTON = (((uint16_t)buf[14]&0X0F)<<8)|buf[15];
		T12_DATA.LEFT_WHEEL = ((uint16_t)buf[16]<<4)|buf[17]>>4;
		T12_DATA.RIGHT_WHEEL = (((uint16_t)buf[17]&0X0F)<<8)|buf[18];
	}
}

