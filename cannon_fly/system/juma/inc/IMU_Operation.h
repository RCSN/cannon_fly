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

#ifndef __IMU_OPERATION_H
#define __IMU_OPERATION_H

#include "cube_hal.h"
#include "imu_sensor.h"
#include "fly_control_9-axis.h"

typedef struct _sensor_fusion_angle_t{
 
   float pitch;
   
   float roll;

   float yaw;
 
} sensor_fusion_angle_t;

typedef struct _imu_sensor_fusion_1_context_t{
 
   float k_acc_1;
 
   float k_acc_2;
  
   float k_gyr_1;
 
   float k_gyr_2;
  
   float k_mag_1;
 
   float k_mag_2;
 
   float k_offset;
 
   float gyro_offset_x;
 
   float gyro_offset_y;
  
   float gyro_offset_z;

} imu_sensor_fusion_1_context_t;

extern void complementary_filter(float acc_raw[3], float gyr_raw[3], float mag_raw[3], float *pitch, float *roll, float *yaw);

extern void imu_sensor_fusion_1(imu_sensor_data_t* sensor_raw, sensor_fusion_angle_t* sensor_angle, imu_sensor_fusion_1_context_t* sensor_context);

extern void MadgwickAHRSupdate(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
extern void MahonyAHRSupdate(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
extern float dmpsafe_asin(float v);
#endif /*_IMU_SENSOR_FUSION_H_*/
