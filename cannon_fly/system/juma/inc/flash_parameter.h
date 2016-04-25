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

#ifndef __FLASH_PATAMETER_H
#define __FLASH_PATAMETER_H

#include "stm32f4xx_hal.h"

typedef enum Flash_Status
{
	Flash_OK=0,
	Flash_Fail=1,
} Flash_Status_t;

/**************************FLASH��ҳ����ʼ��ַ*****************************/
#define ADDR_FLASH_SECTOR_0         ((uint32_t)0x08000000)
#define ADDR_FLASH_SECTOR_1         ((uint32_t)0x08004000)
#define ADDR_FLASH_SECTOR_2         ((uint32_t)0x08008000)
#define ADDR_FLASH_SECTOR_3         ((uint32_t)0x0800C000)
#define ADDR_FLASH_SECTOR_4         ((uint32_t)0x08010000)
#define ADDR_FLASH_SECTOR_5         ((uint32_t)0x08020000)
#define ADDR_FLASH_SECTOR_6         ((uint32_t)0x08040000)
#define ADDR_FLASH_SECTOR_7         ((uint32_t)0x08060000)

#define FLASH_USER_SECTOR           FLASH_SECTOR_5
#define FLASH_USER_START_ADDR       ((uint32_t)0x08020000)
#define FLASH_USER_END_ADDR         ((uint32_t)0x08020004)

void Flash_WR_Test(void);

#endif
