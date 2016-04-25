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
        佛祖保佑   永无BUG   永不宕机

   佛曰:
        写字楼里写字间，写字间里程序员；
        程序人员写程序，又拿程序换酒钱。
        酒醒只在网上坐，酒醉还来网下眠；
        酒醉酒醒日复日，网上网下年复年。
        但愿老死电脑间，不愿鞠躬老板前；
        奔驰宝马贵者趣，公交自行程序员。
				别人笑我忒疯癫，我笑自己命太贱；
        不见满街漂亮妹，哪个归得程序员？
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
		printf("内部FLASH读写测试成功！\n写入的值为0x%x",data32);
	}
}

/********************************************************************************
*函数名称：FLASH_If_Init
*函数功能：flash初始化，允许flash写访问
*输入参数：无
*输出参数：无
*返回值：无
***********************************************************************************/
void FLASH_If_Init(void)
{ 
  HAL_FLASH_Unlock();

  //清除标志位
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
}

/********************************************************************************
*函数名称：GetSector
*函数功能：获取地址所在的扇区
*输入参数：Address：地址
*输出参数：无
*返回值：所给地址的扇区
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
*函数名称：FLASH_If_Erase
*函数功能：擦除app flash
*输入参数：EndSector：app flash 结束的扇区
*输出参数：无
*返回值：0 操作成功
         1 出现错误
***********************************************************************************/
uint32_t FLASH_If_Erase(uint32_t Address,uint32_t Filesize)
{
  uint32_t UserStartSector = FLASH_SECTOR_1, i = 0;
  uint32_t EndSector= FLASH_SECTOR_1;
  //获取应用程序起始地址所在的扇区
  UserStartSector=Flash_GetSector(Address);
  EndSector=Flash_GetSector(Address+Filesize);
  for(i = UserStartSector; i <= EndSector; i += 8)
  {
    // 擦除电压范围是 [2.7V to 3.6V], 以字为单元操作
		FLASH_Erase_Sector(i,FLASH_VOLTAGE_RANGE_3);

  }
  
  return (0);
}