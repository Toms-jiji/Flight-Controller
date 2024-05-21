#include "MPU6050.h"
#include "main.h"

extern Struct_MPU6050 MPU6050;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

#define RETRY_LIMIT 10
#define CALIBRATION_ITERATION_NUMBER 2000
//#define CALIBRATE_ACC
//#define MPU9250


void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
#ifdef MPU9250
//	uint8_t data_verify=0;
//	do{
	SS_Assert();
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &val, 1, HAL_MAX_DELAY);
	SS_Deassert();
//	MPU6050_Readbyte(reg_addr, &data_verify);
//	}while(data_verify!=val);
#else
	while(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000)!=0);
#endif

}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
#ifdef MPU9250
	SS_Assert();
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, data, len, HAL_MAX_DELAY);
	SS_Deassert();
#else
	while(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1000)!=0);
#endif

}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
#ifdef MPU9250
	SS_Assert();
	reg_addr = reg_addr | 0x80;
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, data, 1, HAL_MAX_DELAY);
	SS_Deassert();
#else
	int i=0;
	while(HAL_I2C_Mem_Read(&hi2c1, 0xD1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1000)!=0){
		i++;
		if(i==RETRY_LIMIT)
			break;
	}
#endif
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
#ifdef MPU9250
	SS_Assert();
	reg_addr = reg_addr | 0x80;
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, data, len, HAL_MAX_DELAY);
	SS_Deassert();
#else
	int i=0;
	while(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1000)!=0){
		i++;
		if(i==RETRY_LIMIT)
			break;
	}
#endif
}
#ifdef MPU9250
void SS_Assert(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
}

void SS_Deassert(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
}

#endif
void MPU6050_Initialization(void)
{
	HAL_Delay(50);
	uint8_t who_am_i = 0;
#ifdef MPU9250
	while(who_am_i != 0x70){
		MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);
		Led_Rotate_CW(100,1);
	}
#else
	while(who_am_i != 0x68){
		MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);
		Led_Rotate_CW(100,1);
	}
#endif


	//Reset the whole module before initialization
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x1<<7);
	HAL_Delay(100);

	//Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU6050*/
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	//Sample rate divider
	/*Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV) */
	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz

	//FSYNC and DLPF setting
	/*DLPF is set to 10Hz*/
	MPU6050_Writebyte(MPU6050_CONFIG, 0x05);

	//GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x1;
	MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO<<3);


	//ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x2;
	MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC<<3);

	MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);


}

/*Get Raw Data from sensor*/
void MPU6050_Get6AxisRawData(Struct_MPU6050* MPU6050)
{
	uint8_t data[14]={0};
	MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);

	MPU6050->acc_x_raw = (data[0] << 8) | data[1];
	MPU6050->acc_y_raw = (data[2] << 8) | data[3];
	MPU6050->acc_z_raw = (data[4] << 8) | data[5];

	MPU6050->temperature_raw = (data[6] << 8) | data[7];

	MPU6050->gyro_x_raw = ((data[8] << 8) | data[9]);
	MPU6050->gyro_y_raw = ((data[10] << 8) | data[11]);
	MPU6050->gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch(FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

/*Convert Unit. acc_raw -> g, gyro_raw -> degree per second*/
void MPU6050_DataConvert(Struct_MPU6050* MPU6050)
{
	MPU6050->acc_x = MPU6050->acc_x_raw / LSB_Sensitivity_ACC;
	MPU6050->acc_y = MPU6050->acc_y_raw / LSB_Sensitivity_ACC;
	MPU6050->acc_z = MPU6050->acc_z_raw / LSB_Sensitivity_ACC;

	MPU6050->temperature = (float)(MPU6050->temperature_raw)/340+36.53;

	MPU6050->gyro_pitch = MPU6050->gyro_x_raw / LSB_Sensitivity_GYRO;
	MPU6050->gyro_roll = MPU6050->gyro_y_raw / LSB_Sensitivity_GYRO;
	MPU6050->gyro_yaw = MPU6050->gyro_z_raw / LSB_Sensitivity_GYRO;
}


void MPU6050_ProcessData(Struct_MPU6050* MPU6050)
{
	MPU6050_Get6AxisRawData(MPU6050);
//	HAL_Delay(50);
	MPU6050_DataConvert(MPU6050);
	MPU6050->gyro_pitch = MPU6050->gyro_pitch - MPU6050->calib_gyro_pitch;
	MPU6050->gyro_roll = MPU6050->gyro_roll - MPU6050->calib_gyro_roll;
	MPU6050->gyro_yaw = MPU6050->gyro_yaw - MPU6050->calib_gyro_yaw;
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void Calibrate_Gyro(Struct_MPU6050* MPU6050){
	uint16_t CalibrationNumber=0;
	float GyroCalibration_X=0, GyroCalibration_Y=0, GyroCalibration_Z=0;
	#ifdef CALIBRATE_ACC
		float AccCalibration_X=0, AccCalibration_Y=0, AccCalibration_Z=0;
	#endif
	for(CalibrationNumber=0; CalibrationNumber<CALIBRATION_ITERATION_NUMBER; CalibrationNumber++)
	{
		MPU6050_ProcessData(MPU6050);
		#ifdef CALIBRATE_ACC
		AccCalibration_X += MPU6050->acc_x;
		AccCalibration_Y += MPU6050->acc_y;
		AccCalibration_Z += MPU6050->acc_z;
		#endif

		GyroCalibration_X += MPU6050->gyro_pitch;
		GyroCalibration_Y += MPU6050->gyro_roll;
		GyroCalibration_Z += MPU6050->gyro_yaw;
		HAL_Delay(1);
	}
	#ifdef CALIBRATE_ACC
	MPU6050->calib_acc_x = AccCalibration_X / CALIBRATION_ITERATION_NUMBER;
	MPU6050->calib_acc_y = AccCalibration_Y / CALIBRATION_ITERATION_NUMBER;
	MPU6050->calib_acc_z = AccCalibration_Z / CALIBRATION_ITERATION_NUMBER;
	#endif

	MPU6050->calib_gyro_pitch = GyroCalibration_X / CALIBRATION_ITERATION_NUMBER;
	MPU6050->calib_gyro_roll = GyroCalibration_Y / CALIBRATION_ITERATION_NUMBER;
	MPU6050->calib_gyro_yaw = GyroCalibration_Z / CALIBRATION_ITERATION_NUMBER;
}


void MPU6050_Data(Struct_MPU6050* MPU6050){
	MPU6050_ProcessData(MPU6050);
	#ifdef CALIBRATE_ACC
	MPU6050->acc_x = MPU6050->acc_x - MPU6050->calib_acc_x;
	MPU6050->acc_y = MPU6050->acc_y - MPU6050->calib_acc_y;
	MPU6050->acc_z = MPU6050->acc_z - MPU6050->calib_acc_z;
	#endif
	MPU6050->gyro_pitch = MPU6050->gyro_pitch - MPU6050->calib_gyro_pitch;
	MPU6050->gyro_roll = MPU6050->gyro_roll - MPU6050->calib_gyro_roll;
	MPU6050->gyro_yaw = MPU6050->gyro_yaw - MPU6050->calib_gyro_yaw;
}
