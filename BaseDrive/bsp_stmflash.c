#include "bsp_stmflash.h"

/**
 * [STMFLASH_GetFlashSector 获取某个地址所在的flash扇区]
 * @param  addr [flash地址]
 * @return      [0~11,即addr所在的扇区]
 */
uint16_t STMFLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
	return FLASH_Sector_11;
}


/**
 * [STMFLASH_ReadWord 读取指定地址的字(32位数据)]
 * @param  faddr [读地址]
 * @return       [对应数据]
 */
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(uint32_t*)faddr;
}

/**
 * [STMFLASH_Write 从指定地址开始写入指定长度的数据]
 * @param WriteAddr  [起始地址(此地址必须为4的倍数!!)]
 * @param pBuffer    [数据指针]
 * @param NumToWrite [写入的32位数据的个数]
 * @tips 1.因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
 *         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
 *         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
 *         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写.
 *       2.OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
 */
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)
{
    FLASH_Status status = FLASH_COMPLETE;
	uint32_t addrx=0;
	uint32_t endaddr=0;
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
		FLASH_Unlock();									//解锁
	FLASH->ACR&=~(1<<10);  
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存

	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_2);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		}
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		}
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH->ACR|=1<<10; 
	FLASH_Lock();//上锁
}

void STMFLASH_Erase(uint32_t WriteAddr)
{

}


/**
 * [STMFLASH_Read 从指定地址开始读出指定长度的数据]
 * @param ReadAddr  [起始地址]
 * @param pBuffer   [数据指针]
 * @param NumToRead [字(4位)数]
 */
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.
	}
}





