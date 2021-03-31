#include "AS5048.h"
#include "bsp_delay.h"

void AS5048A_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟

	//GPIOFB3,4,5初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	SPI1_Init();
}
// as5048读取接收函数
//uint16_t SPI_Read5048Data(uint16_t TxData)
//{
//    uint16_t data;
//    AS5048A_CS() = 0;
//    data=SPI1_ReadWriteByte(TxData);
//    AS5048A_CS() = 1;
//    return data&0x3fff;
//}

uint16_t SPI_Read5048Data(uint16_t TxData)
{
    uint16_t data;
    AS5048A_CS() = 0;
	delay_us(10);
    SPI1_ReadWriteWorld(TxData);
    AS5048A_CS() = 1;
	delay_us(10);
	AS5048A_CS() = 0;
	delay_us(10);
	data=SPI1_ReadWriteWorld(CMD_NOP);
    AS5048A_CS() = 1;
	return data&0x3fff;
}

struct as5048_data Mag5048a_data;
struct as5048_data CollectData()
{
    uint16_t anglereg = 0, magreg = 0, agcreg = 0;
    uint16_t mag = 0, value = 0;
    double angle = 0.0;
    uint8_t agc = 0;
    struct as5048_data Temp = {1,0,0,0,0.0};

    SPI_Read5048Data(CMD_ANGLE);
	SPI_Read5048Data(CMD_ANGLE);
    anglereg = SPI_Read5048Data(CMD_MAG); 
	value = anglereg & 0x3fff;
    magreg = SPI_Read5048Data(CMD_AGC);   
	mag = magreg & 0x3fff;
    agcreg = SPI_Read5048Data(CMD_NOP);   
	agc = (uint8_t)agcreg & 0x00ff;
    angle = (value * 360.0)/16384.0;
    
    if ((anglereg & 0x4000) | (magreg & 0x4000) | (agcreg & 0x4000))
    {
        ClearAndNop();
        //rt_kprintf("There is and error!\n");
        Temp.iserror = 1;
    }
    else
    {
        Temp.iserror = 0;
        Temp.angle = angle;
        Temp.mag = mag;
        Temp.agc = agc;
        Temp.value = value;
    }
    return Temp;
}

// as5048清除错误标志位
uint16_t ClearAndNop(void)
{
    AS5048A_CS() = 0;
    SPI_Read5048Data(CMD_CLAER);              // 附加偶校验的错误标志位清除命令
    AS5048A_CS() = 1;
    delay_us(10);              // 两次命令之间有350ns的间隔，源自官方datasheet
    AS5048A_CS() = 0;
    SPI_Read5048Data(CMD_NOP);               // 附加偶校验的错误标志位清除命令
    AS5048A_CS() = 1;
	return 0;
}

/**********************ddddddddddd****************************/
// Calculates even parity of 16it value, returns 1 (odd) or 0 (even)
/*uint8_t parity_even(uint16_t v)
{
  if(v == 0) return 0;
  v ^= v >> 8;
  v ^= v >> 4;
  v ^= v >> 2;
  v ^= v >> 1;
 
  return v & 1;
}

uint8_t error_flag;
uint16_t data = 0;
uint16_t res;
uint16_t command = 0x4000;// PAR=0 R/W=R
uint16_t Read_As5048A_Reg(uint16_t cmd)
{
	command = command | cmd;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
    SPI_Read5048Data(command);

	command = 0x4000;// PAR=0 R/W=R
	command = command | CMD_NOP;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
    res = SPI_Read5048Data(command);
	error_flag = 1;

	if ((res & (1 << 14)) == 0)//判断第14位是否为0,0为正确值
	{
		data = (res & 0x3FFF);
		error_flag = (parity_even(data) ^ (res >> 15));
	} 
	else//读数据错误，发送清错误命令?
	{
		command = 0x4000;
		command = command | CMD_CLEAR_ERROR;
		command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
		SPI_Read5048Data(command);
	}
	return data;
}


//写寄存器
void Write_As5048A_Reg(uint16_t cmd,uint16_t value)
{
	uint16_t data = 0;
	uint16_t res;
	uint16_t command = 0x0000; // PAR=0 R/W=W

	command = command | cmd;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
	SPI_Read5048Data(command);


	command = 0x0000; // PAR=0 R/W=W
	command = command | value;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
	SPI_Read5048Data(command);

	command = 0x4000;// PAR=0 R/W=R
	command = command | CMD_NOP;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
	res = SPI_Read5048Data(command);
	error_flag = 1;

	if ((res & (1 << 14)) == 0)//判断第14位是否为0,0为正确值
	{
		data = (res & 0x3FFF);
		error_flag = (parity_even(data) ^ (res >> 15));
	} 
	else//读数据错误，发送清错误命令?
	{
		command = 0x4000;
		command = command | CMD_CLEAR_ERROR;
		command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
		SPI_Read5048Data(command);
	}
}

uint16_t Read_As5048A_Value(uint16_t cmd)
{
	uint16_t val;
	val = Read_As5048A_Reg(cmd);
	if(error_flag)//奇偶校验错误
	{
	val = 0;
	}
	return val;
}*/

/************************************
OTP Write Zero Position: 0 for No error
1. Read angle information
2. Set the Programming Enable bit in the OTP control register
3. Write previous read angle position into OTP zero position register
4. Read back for verification the zero position register data
5. Set the Burn bit to start the automatic programming procedure
6. Read angle information (equals to 0)
7. Set the Verify bit to load the OTP data again into the internal registers with modified threshold comparator levels
8. Read angle information (equals to 0)
******************************************/
//uint8_t Write_As5048A_ZeroPosition(void)
//{
//	uint16_t Angle_val;
//	uint8_t Angle_High,Angle_Low;
//	uint16_t cmd;
//	Angle_val = Read_As5048A_Value(CMD_ANGLE);
//	cmd=Read_As5048A_Value(CMD_ProgramControl);

//	Write_As5048A_Reg(CMD_ProgramControl,0x01);
//	delay_us(10);
//	cmd=Read_As5048A_Value(CMD_ProgramControl);
//	delay_us(10);
//	Write_As5048A_Reg(CMD_OTPHigh,Angle_val>>8);
//	delay_us(10);
//	Write_As5048A_Reg(CMD_OTPLow,Angle_val&0xFF);
//	delay_us(10);
//	Angle_High=Read_As5048A_Value(CMD_OTPHigh);
//	delay_us(10);
//	Angle_Low=Read_As5048A_Value(CMD_OTPLow);
//	delay_us(10);
//	if(Angle_High != (uint8_t)(Angle_val>>8))
//	return 1;
//	if(Angle_Low != (uint8_t)(Angle_val&0xFF))
//	return 2;

//	Write_As5048A_Reg(CMD_ProgramControl,0x09);
//	delay_us(10);
//	cmd=Read_As5048A_Value(CMD_ProgramControl);
//	delay_us(10);
//	Angle_val = Read_As5048A_Value(CMD_ANGLE);
//	delay_us(10);
//	if(Angle_val != 0 )
//	return 3;

//	Write_As5048A_Reg(CMD_ProgramControl,0x49);
//	delay_us(10);
//	cmd=Read_As5048A_Value(CMD_ProgramControl);
//	delay_us(10);
//	Angle_val = Read_As5048A_Value(CMD_ANGLE);
//	delay_us(10);
//	if(Angle_val != 0 )
//	{return 4;}

//	return 0;
//}
