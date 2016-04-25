#ifndef _IMU_SENSOR_H_
#define _IMU_SENSOR_H_
#include "cube_hal.h"
#include "lsm6ds3.h"
#include "lsm303agr.h"
#include "stm32f4xx_hal_msp.h"
#include "bluenrg_sdk_api.h"

#ifndef FLY_CONTROL
#error FLY_CONTROL must be defined!
#endif

/*sensor feature*/
#define IMU_SENSOR_FEATURE_ACC  0x01

#define IMU_SENSOR_FEATURE_GYRO 0x02

#define IMU_SENSOR_FEATURE_MAG  0x04


//enum{ROLL,PITCH,YAW};
#define M_PI_F 3.1415926
#define CONSTANTS_ONE_G											9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C	1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST							287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH						6371000			/* meters (m)		*/

#define SENSOR_MAX_G     										8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W     										2000.0f	//deg/s
#define ACC_SCALE        										(SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE       									  (SENSOR_MAX_W/32768.0f)


/*fifo threthold level*/
#define FIFO_WATER_MARK         48

/*FIFO CTRL MASK*/
#define LSM6DS3_XG_FIFO_THRESHOLD_MASK                      ((uint8_t)0x0F)
#define LSM6DS3_XG_FIFO_INT_THRESHOLD_MASK                 ((uint8_t)0x08)

/* Six axes sensor IO functions */
extern IMU_6AXES_StatusTypeDef LSM6DS3_IO_Init( void );
extern IMU_6AXES_StatusTypeDef LSM6DS3_IO_Write( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
    uint16_t NumByteToWrite );
extern IMU_6AXES_StatusTypeDef LSM6DS3_IO_Read( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
    uint16_t NumByteToRead );
extern void LSM6DS3_IO_ITConfig( void );

enum _imu_status{
   imu_status_ok = 0,
   imu_status_fail = 1,
};

typedef enum
{
	Flag_SET = 0,
	Flag_RESET = 1
} timer_flag;

typedef enum 
{
   OUTPUT_DISABLE = 0, 
   OUTPUT_ENABLE = !OUTPUT_DISABLE
}output_state;

typedef enum _sensor_selection{
   ACC_ENABLE           = 1,
   GYRO_ENABLE          = 2,
   ACC_AND_GYRO_ENABLE  = 3,
   MAG_ENABLE           = 4,
   ALL_ENABLE           = 7,
} sensor_selsection_t;

typedef enum sensor_data_type{
   TYPE_ACC_DATA  = 0,
   TYPE_GYRO_DATA = 1,
   TYPE_MAG_DATA  = 2,
} sensor_data_type_t;
typedef struct _imu_sensor_data_t {


    float acc[3];

    float gyro[3];

    float mag[3];

} imu_sensor_data_t;

typedef struct _imu_sensor_angle_t {
	
	float pitch;
	
	float roll;
	
	float yaw;
	
} imu_sensor_angle_t;

extern timer_flag flag_5ms;

typedef enum _imu_status imu_status_t;
typedef struct _imu_sensor_data_t imu_sensor_data_t; 

/*IMU Sensor API */

imu_status_t imu_sensor_9_axis_reset(void);

imu_status_t imu_sensor_9_axis_select_features(sensor_selsection_t features);

imu_status_t imu_sensor_9_axis_set_data_rate(uint32_t* p_data_rate, uint8_t mode); 

imu_status_t imu_sensor_9_axis_start(void); 

imu_status_t imu_sensor_9_axis_stop(void);

void imu_sensor_9_axis_read_data_from_fifo(void);


#endif /*_IMU_SENSOR_H_*/



