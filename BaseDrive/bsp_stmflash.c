#include "bsp_stmflash.h"

/**
 * [STMFLASH_GetFlashSector ��ȡĳ����ַ���ڵ�flash����]
 * @param  addr [flash��ַ]
 * @return      [0~11,��addr���ڵ�����]
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
 * [STMFLASH_ReadWord ��ȡָ����ַ����(32λ����)]
 * @param  faddr [����ַ]
 * @return       [��Ӧ����]
 */
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(uint32_t*)faddr;
}

/**
 * [STMFLASH_Write ��ָ����ַ��ʼд��ָ�����ȵ�����]
 * @param WriteAddr  [��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)]
 * @param pBuffer    [����ָ��]
 * @param NumToWrite [д���32λ���ݵĸ���]
 * @tips 1.��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
 *         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
 *         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
 *         û����Ҫ����,��������������Ȳ�����,Ȼ����������д.
 *       2.OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
 */
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)
{
    FLASH_Status status = FLASH_COMPLETE;
	uint32_t addrx=0;
	uint32_t endaddr=0;
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
		FLASH_Unlock();									//����
	FLASH->ACR&=~(1<<10);  
	FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���

	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_2);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		}
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		}
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH->ACR|=1<<10; 
	FLASH_Lock();//����
}

void STMFLASH_Erase(uint32_t WriteAddr)
{

}


/**
 * [STMFLASH_Read ��ָ����ַ��ʼ����ָ�����ȵ�����]
 * @param ReadAddr  [��ʼ��ַ]
 * @param pBuffer   [����ָ��]
 * @param NumToRead [��(4λ)��]
 */
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.
	}
}




