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

#ifndef __ANO_DT_H
#define __ANO_DT_H

#include "stm32f4xx_hal_msp.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

#ifndef u8
#define u8      uint8_t
#endif

#ifndef u16
#define u16     uint16_t
#endif

#ifndef vs16
#define vs16    volatile int16_t
#endif

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


void sendware(uint8_t *wareaddr,uint32_t waresize);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw);
void ANO_DT_Send_Senser(float a_x,float a_y,float a_z,float g_x,float g_y,float g_z,float m_x,float m_y,float m_z,int32_t bar);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);

void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);

#endif
