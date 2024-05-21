/*
 * MPU6050.h
 *
 *  Created on: Apr 1, 2024
 *      Author: Vivek
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_


#include "main.h"

#define MPU6050_ADDR 0xD0


#define MPU6050_SMPRT_DIV 0X19
#define MPU6050_WHO_AM_I 0X75
#define MPU6050_CONFIG 0X1A
#define MPU6050_GYRO_CONFIG 0X1B
#define MPU6050_ACCEL_CONFIG 0X1C
#define MPU6050_INT_PIN_CFG 0X37
#define MPU6050_INT_ENABLE 0X38
#define MPU6050_INT_STATUS 0X3A
#define MPU6050_ACCEL_XOUT_H 0X3B
#define MPU6050_ACCEL_XOUT_L 0X3C

#define MPU6050_ACCEL_YOUT_H 0X3D
#define MPU6050_ACCEL_ZOUT_H 0X3F

#define MPU6050_TEMP_XOUT_H 0X41
#define MPU6050_TEMP_XOUT_L 0X42

#define MPU6050_GYRO_XOUT_H 0X43
#define MPU6050_GYRO_XOUT_L 0X44

#define MPU6050_GYRO_YOUT_H 0X45
#define MPU6050_GYRO_ZOUT_H 0X47

#define MPU6050_PWR_MGMT_1 0X6B //most important



#define MPU6050_INT_PORT 	GPIOB
#define MPU6050_INT_PIN 	GPIO_PIN_5


typedef struct _MPU6050{
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	short temperature_raw;
	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

	float acc_x;
	float acc_y;
	float acc_z;
	float temperature;
	float gyro_pitch;
	float gyro_roll;
	float gyro_yaw;

	float calib_gyro_pitch;
	float calib_gyro_roll;
	float calib_gyro_yaw;

	float calib_acc_x;
	float calib_acc_y;
	float calib_acc_z;
}Struct_MPU6050;




void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data);
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void SS_Deassert();
void SS_Assert();

void MPU6050_Initialization(void);
void MPU6050_Get6AxisRawData(Struct_MPU6050*MPUu6050);
int MPU6050_DataReady(void);
void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC);
void MPU6050_DataConvert(Struct_MPU6050* mpu6050);
void MPU6050_ProcessData(Struct_MPU6050* mpu6050);

void Calibrate_Gyro(Struct_MPU6050* MPU6050);
void MPU6050_Data(Struct_MPU6050* MPU6050);

#endif /* SRC_MPU6050_H_ */
