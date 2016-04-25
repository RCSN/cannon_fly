/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ���汣��   ����BUG   ����崻�

   ��Ի:
        д��¥��д�ּ䣬д�ּ������Ա��
        ������Աд�������ó��򻻾�Ǯ��
        ����ֻ���������������������ߣ�
        ��������ո��գ����������긴�ꡣ
        ��Ը�������Լ䣬��Ը�Ϲ��ϰ�ǰ��
        ���۱������Ȥ���������г���Ա��
				����Ц��߯��񲣬��Ц�Լ���̫����
        ��������Ư���ã��ĸ���ó���Ա��
*/

#include "flash_parameter.h"

#define DATA_32           ((uint32_t)0x12345678)

void Flash_WR_Test(void)
{
	uint32_t flash_address = 0;
	uint32_t data32 = 0;
	
	HAL_FLASH_Unlock();
	
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_RDERR);
	
	
	FLASH_Erase_Sector(FLASH_USER_SECTOR,FLASH_VOLTAGE_RANGE_3);
	
	flash_address = FLASH_USER_START_ADDR;
	
	while(flash_address < FLASH_USER_END_ADDR)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,flash_address,DATA_32)==HAL_OK)
		{
			flash_address+=4;
		}
		else
		{
			while(1);
		}
	}
	
	HAL_FLASH_Lock();
	
	flash_address = FLASH_USER_START_ADDR;
	
	while(flash_address < FLASH_USER_END_ADDR)
	{
		data32 = *(__IO uint32_t*)flash_address;
		flash_address+=4;
	}
	if(data32 == DATA_32)
	{
		printf("�ڲ�FLASH��д���Գɹ���\nд���ֵΪ0x%x",data32);
	}
}

/********************************************************************************
*�������ƣ�FLASH_If_Init
*�������ܣ�flash��ʼ��������flashд����
*�����������
*�����������
*����ֵ����
***********************************************************************************/
void FLASH_If_Init(void)
{ 
  HAL_FLASH_Unlock();

  //�����־λ
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
}

/********************************************************************************
*�������ƣ�GetSector
*�������ܣ���ȡ��ַ���ڵ�����
*���������Address����ַ
*�����������
*����ֵ��������ַ������
***********************************************************************************/
static uint32_t Flash_GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if(Address >= ADDR_FLASH_SECTOR_7)
  {
    sector = FLASH_SECTOR_7;  
  }
    return sector;
}

/********************************************************************************
*�������ƣ�FLASH_If_Erase
*�������ܣ�����app flash
*���������EndSector��app flash ����������
*�����������
*����ֵ��0 �����ɹ�
         1 ���ִ���
***********************************************************************************/
uint32_t FLASH_If_Erase(uint32_t Address,uint32_t Filesize)
{
  uint32_t UserStartSector = FLASH_SECTOR_1, i = 0;
  uint32_t EndSector= FLASH_SECTOR_1;
  //��ȡӦ�ó�����ʼ��ַ���ڵ�����
  UserStartSector=Flash_GetSector(Address);
  EndSector=Flash_GetSector(Address+Filesize);
  for(i = UserStartSector; i <= EndSector; i += 8)
  {
    // ������ѹ��Χ�� [2.7V to 3.6V], ����Ϊ��Ԫ����
		FLASH_Erase_Sector(i,FLASH_VOLTAGE_RANGE_3);

  }
  
  return (0);
}