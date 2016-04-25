
#include "app.h"

#include <math.h>
#if NO_PRINTF
#define printf(...)
#endif

#define SAMPLINGRATE 400

static void adv_name_generate(uint8_t* uni_name);
static void sensor_print(void* arg);

static uint8_t sendFLAG = FALSE;
static uint8_t sensor_data_ready_FLAG = FALSE;
static imu_sensor_data_t local_data;
static imu_sensor_angle_t local_angle;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

#ifdef CANNON_V2
char name[20] = "CANNON_V2";
#endif
#ifdef CANNON_V1
char name[20] = "CANNON_V1";
#endif

void on_ready(void)
{
  uint8_t tx_power_level = 5;
  uint16_t adv_interval = 100;

  uint8_t bdAddr[6];
  uint32_t data_rate = SAMPLINGRATE;
  HCI_get_bdAddr(bdAddr);
  adv_name_generate(bdAddr + 4);
  /*Config Adv Parameter And Ready to Adv*/
  ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
  ble_device_start_advertising();

  BSP_LED_Off(LED0);

  imu_sensor_9_axis_reset();

  imu_sensor_9_axis_set_data_rate(&data_rate, LSM6DS3_XG_FIFO_MODE_CONTINUOUS_OVERWRITE);

  imu_sensor_9_axis_start();

}


static void adv_name_generate(uint8_t* uni_name) {
  char temp[3] = "_";
  /*adv name aplice*/
  sprintf(temp + 1, "%01d%01d", *uni_name, *(uni_name + 1));
  strcat(name, temp);
}


static void sensor_print(void* arg)
{
  static int counter = 0;
  if (++counter > 5) {
    counter = 0;

    //static char st = '$';
    //UART_Transmit((uint8_t *)&st, 1);
    //UART_Transmit((uint8_t *)q, sizeof(q));
    if (sendFLAG) {
      ble_device_send(0x01, 16, (uint8_t*)q);
    }
  }
}

/*received data callback*/
void on_imu_sensor_9_axis_data(imu_sensor_data_t* data)
{
  local_data = *data;
  sensor_data_ready_FLAG = TRUE;

  //quick jump out sensor ISR for other events
}

/* Device On Message */
void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value)
{


}
/* Device on connect */
void ble_device_on_connect(void)
{
  sendFLAG = TRUE;

}
/* Device on disconnect */
void ble_device_on_disconnect(uint8_t reason)
{
  sendFLAG = FALSE;
  /* Make the device connectable again. */
  Ble_conn_state = BLE_CONNECTABLE;
  ble_device_start_advertising();
}


void attitude_sensor(void)
{
	
	static int i=0;
	static float gorysetx=0,gorysety=0,gorysetz=0;
	static float accsetx=0,accsety=0,accsetz=0;
	static float magsetx=0,magsety=0,magsetz=0;
	static float coef = 3.1415926599f/180;
	float pitch=0;float roll=0;float yaw;
	static unsigned int PWM_pri=0;
	unsigned char jieshou=0;
	uint8_t tmp_buf[33];	
	uint8_t angle[2];
	SetRX_Mode();
	if(flag_5ms==Flag_RESET)
	{
		//SetRX_Mode();
		flag_5ms=Flag_SET;
		if(i<1000)
		{
		//calc gyro offset with 1000 samples
			gorysetx+=local_data.gyro[0];gorysety+=local_data.gyro[1];gorysetz+=local_data.gyro[2];
			accsetx+=local_data.acc[0];accsety+=local_data.acc[1];accsetz+=local_data.acc[2];
			magsetx+= local_data.mag[0];magsety+= local_data.mag[1];magsetz+= local_data.mag[2];
			i++;
			if(i==1000)
			{
				BSP_LED_On(LED0);
				gorysetx/=1000;gorysety/=1000;gorysetz/=1000;
				accsetx/=1000;accsety/=1000;accsetz/=1000;
				magsetx/=1000;magsety/=1000;magsetz/=1000;
				gorysetx=gorysetx* GYRO_SCALE * M_PI_F /180.f;
				gorysety=gorysety* GYRO_SCALE * M_PI_F /180.f;
				gorysetz=gorysetz* GYRO_SCALE * M_PI_F /180.f;
				accsetx=accsetx*ACC_SCALE * CONSTANTS_ONE_G;
				accsety=accsety*ACC_SCALE * CONSTANTS_ONE_G;
				accsetz=accsetz*ACC_SCALE * CONSTANTS_ONE_G;
				BSP_LED_Off(LED0);
			}
		}
		else
		{
				
			
			MadgwickAHRSupdate(q, 0.006,local_data.gyro[0]-gorysetx,local_data.gyro[1]-gorysety,local_data.gyro[2]-gorysetz,local_data.acc[0]-accsetx,local_data.acc[1]-accsety,local_data.acc[2]-accsetz,(local_data.mag[0]*-1),(local_data.mag[1]*-1),(local_data.mag[2]));
	
			local_angle.roll= -(atan2(2.0*(q[0]*q[1] + q[2]*q[3]),
	                       1 - 2.0*(q[1]*q[1] + q[2]*q[2])))* 180/3.1415926f;
	    // we let safe_asin() handle the singularities near 90/-90 in pitch
    	local_angle.pitch = dmpsafe_asin(2.0*(q[0]*q[2] - q[3]*q[1]))* 180/3.1415926f;
	    //注意：此处计算反了，非右手系。
     	local_angle.yaw =-atan2(2.0*(q[0]*q[3] + q[1]*q[2]),
	                     1 - 2.0*(q[2]*q[2] + q[3]*q[3]))* 180/3.1415926f;
	

			ANO_DT_Send_Senser(local_data.acc[0]-accsetx,local_data.acc[1]-accsety,local_data.acc[2]-accsetz,
													 local_data.gyro[0]-gorysetx,local_data.gyro[1]-gorysety,local_data.gyro[2]-gorysetz,
													 local_data.mag[0],local_data.mag[1],local_data.mag[2],0);
			ANO_DT_Send_Status(local_angle.roll,local_angle.pitch,local_angle.yaw);
			
			}

		if(NRF24L01_RxPacket(tmp_buf)==0)
		{
			BSP_LED_On(LED0);
			jieshou=tmp_buf[1];
	

			
			Motor_PWM_Duty.Motor1 =(1000*jieshou)/255;
			Motor_PWM_Duty.Motor2 =(1000*jieshou)/255;
			Motor_PWM_Duty.Motor3 =(1000*jieshou)/255;
			Motor_PWM_Duty.Motor4 =(1000*jieshou)/255;
			
			
				
			Motor_Control(&Motor_PWM_Duty);
		}
			sensor_print(NULL);
		}
}


