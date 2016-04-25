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

#ifndef __FLASH_PATAMETER_H
#define __FLASH_PATAMETER_H

#include "stm32f4xx_hal.h"

typedef enum Flash_Status
{
	Flash_OK=0,
	Flash_Fail=1,
} Flash_Status_t;

/**************************FLASH各页的起始地址*****************************/
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
