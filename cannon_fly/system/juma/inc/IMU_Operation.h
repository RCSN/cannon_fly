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
