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

#include "ANO_DT.h"

#define ANO_DT_Send_Data(data,cnt)  HAL_UART_Transmit(&UartHandle,data,cnt,0xFFFF);


uint8_t data_to_send[50];

/* ��������ǲ�����������վ�����ݷ��ͺ�������ɽ��๦�ܵ������ֵ�����ʾ�������ݷ��ͺ��� */
void sendware(uint8_t *wareaddr,uint32_t waresize)
{
#define CMD_WARE 3
	uint8_t cmdf[2]={CMD_WARE,~CMD_WARE};
	uint8_t cmdr[2]={~CMD_WARE,CMD_WARE};
	
	HAL_UART_Transmit(&UartHandle, cmdf, sizeof(cmdf), 0xFFFF);
	HAL_UART_Transmit(&UartHandle, wareaddr, waresize, 0xFFFF);
	HAL_UART_Transmit(&UartHandle, cmdr, sizeof(cmdr), 0xFFFF);
}

/************************************* ��������վ����ͨ��Э�� *************************************/

/***** ���ͷɻ���������ֵ *****/
void ANO_DT_Send_Senser(float a_x,float a_y,float a_z,float g_x,float g_y,float g_z,float m_x,float m_y,float m_z,int32_t bar)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = (int16_t)a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***** ���ͷɻ���ǰ��̬*****/
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw)
{
	uint8_t _cnt=0;
	int16_t	_temp;
	int32_t _temp2 = 0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = 0;
	
	data_to_send[_cnt++] = 0;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***** У������ *****/
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/***** �汾��Ϣ *****/
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***** ���ͷɻ��յ��Ŀ������� *****/
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***** ���͵������ *****/
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***** ���͵��PWM *****/
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***** ����PID����֡ *****/
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã����紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
	
	if(*(data_buf+2)==0X01)             //������0x01 ����ϼ�1
	{
		if(*(data_buf+4)==0X01)           //01   ACCУ׼
		{
			printf("ACCУ׼���\n");
			//mpu6050.Acc_CALIBRATE = 1;
		}
		if(*(data_buf+4)==0X02)           //02   GYROУ׼
		{
			printf("GYROУ׼���\n");
			//mpu6050.Gyro_CALIBRATE = 1;
		}
		if(*(data_buf+4)==0X03)           //03   MAGУ׼
		{
			printf("MAGУ׼���\n");
			//mpu6050.Acc_CALIBRATE = 1;		
			//mpu6050.Gyro_CALIBRATE = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)						  //������0x02 ����ϼ�2
	{
		if(*(data_buf+4)==0X01)           //01 ��ȡPID���󣨷���AAAA 10\11\12\13\14\15����֡��
		{
			printf("PID��ȡ����\n");
			/*f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;*/
		}
		if(*(data_buf+4)==0X02)           //02 ��ȡ����ģʽ�������󣨷���AAAA 0A����֡��
		{
			printf("��ȡ����ģʽ��������\n");
		}
		if(*(data_buf+4)==0XA0)		        //A0 ��ȡ�汾��Ϣ
		{
			printf("��ȡ�汾��Ϣ��\n");
			//f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		        //A1 �ָ�Ĭ�ϲ���
		{
			printf("�ָ�Ĭ�ϲ�����\n");
			//Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
			printf("PID1��\n");
        /*ctrl_1.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_1.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_1.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_1.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_1.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_1.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_1.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_1.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_1.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
				Param_SavePID();*/
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
			printf("PID2��\n");
        /*ctrl_1.PID[PID4].kp 	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_1.PID[PID4].ki 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_1.PID[PID4].kd 	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_1.PID[PID5].kp 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_1.PID[PID5].ki 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_1.PID[PID5].kd 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_1.PID[PID6].kp	  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_1.PID[PID6].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_1.PID[PID6].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
				Param_SavePID();*/
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
			printf("PID3��\n");
        /*ctrl_2.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_2.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_2.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_2.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_2.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_2.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
				Param_SavePID();*/
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		printf("PID4��\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		printf("PID5��\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		printf("PID6��\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

/******************* END OF FILE *******************/

