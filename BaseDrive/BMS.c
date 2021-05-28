#include "bms.h"
#include "can2.h"
#include "can1.h"

Battery Battery_Msg;

void Bms_Get_Soc(void)
{
	uint8_t CANBUF[8] = {0};
	uint32_t extid = PROPOTY<<24|GET_TOTAL_SOC<<16|BMS_ADDR<<8|PC_ADDR;
	CAN2_TX_EXTID(extid,CANBUF,8);
}
