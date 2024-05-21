/*
 * Quad.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Toms
 */


/*
 *
 * Calibrate Accelerometer
 */

#include "main.h"
#include "MPU6050.h"
#include "PWM.h"
#include "Quad.h"
#include "DEBUG_UART.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define Reset_time() __HAL_TIM_SET_COUNTER(&htim12, 0)
#define Get_time() __HAL_TIM_GET_COUNTER(&htim12)

//#define TEST_MOTOR_DIRECTIONS_ON_QUAD
#define PD_TUNNABLE
//#define PID_TUNNABLE

#define QUAD_DEBUG
//#define TUNE_COMPLEMENTARY_FILTER

#define TUNE_PID

#ifdef TEST_MOTOR_DIRECTIONS_ON_QUAD
	#ifndef TUNE_PID
		#define TUNE_PID
	#endif
#endif

#define PID_RX_SCALE_P 0.01
#define PID_RX_SCALE_D 0.1
#define GYRO_LPF_FACTOR 0.3


#ifdef QUAD_DEBUG
	uint8_t message[100] = {'\0'};
#endif

extern Struct_MPU6050 MPU6050;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
extern Struct_MPU6050 MPU6050;
extern volatile Struct_Receiver RX_DATA;
extern volatile Quad_PID_DATA QUAD_PID;
extern volatile QUAD_ORIENTATION QUAD;

void quad_init(){
	HAL_TIM_Base_Start(&htim5); 		//Start PPM Read

//	manual_ESC_calibration();
	SET_PWM_Value(1000,1000,1000,1000);
	Calibrate_Receiver();
	Calibrate_ESCs();
	MPU6050_Initialization();
	Calibrate_Gyro(&MPU6050);

	QUAD_PID.pid_max_roll = 400;
	QUAD_PID.pid_max_pitch = 400;
	QUAD_PID.pid_max_yaw = 400;

	QUAD_PID.pid_p_gain_yaw = 4.0;
	QUAD_PID.pid_i_gain_yaw = 0.02;
	QUAD_PID.pid_d_gain_yaw = 0.0;

//	QUAD_PID.pid_d_gain_roll = 100;
//	QUAD_PID.pid_d_gain_pitch = 100;

	QUAD_PID.pid_i_gain_roll = 0.02;
	QUAD_PID.pid_i_gain_pitch = 0.02;

#ifdef QUAD_DEBUG
//	sprintf(message, "All Initialization done..!!\r\n");
//	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

	ARM_Quad();

	__HAL_TIM_SET_COUNTER(&htim12, 0);
}

void quad_fly(){
#ifdef TUNE_PID
	GET_PID_Constants();
#endif

	Check_for_UnARM_Motor_Command();
	static _Bool starting =1;
	if(starting == 1){
		Reset_time();
	}
	uint16_t time_elapsed=0;
	time_elapsed = Get_time();
	Reset_time();
	MPU6050_ProcessData(&MPU6050);

	QUAD_PID.gyro_roll_input  = (QUAD_PID.gyro_roll_input*(1-GYRO_LPF_FACTOR))  + (MPU6050.gyro_roll*(GYRO_LPF_FACTOR));
	QUAD_PID.gyro_pitch_input = (QUAD_PID.gyro_pitch_input*(1-GYRO_LPF_FACTOR)) + (MPU6050.gyro_pitch*(GYRO_LPF_FACTOR));
	QUAD_PID.gyro_yaw_input   = (QUAD_PID.gyro_yaw_input*(1-GYRO_LPF_FACTOR))   + (MPU6050.gyro_yaw*(GYRO_LPF_FACTOR));



	QUAD.angle_pitch -= (float)((double)MPU6050.gyro_pitch*(double)time_elapsed/1000000);
	QUAD.angle_roll -=  (float)((double)MPU6050.gyro_roll* (double)time_elapsed/1000000);

	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
	QUAD.angle_pitch += QUAD.angle_roll * (float)sin((double)MPU6050.gyro_yaw *(double)time_elapsed/57295779);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
	QUAD.angle_roll -= QUAD.angle_pitch * (float)sin((double)MPU6050.gyro_yaw *(double)time_elapsed/57295779);                  //If the IMU has yawed transfer the pitch angle to the roll angel.


	//Accelerometer angle calculations
	QUAD.acc_total_vector = sqrt((MPU6050.acc_x*MPU6050.acc_x)+(MPU6050.acc_y*MPU6050.acc_y)+(MPU6050.acc_z*MPU6050.acc_z));       //Calculate the total accelerometer vector.

	if(abs(MPU6050.acc_y) < QUAD.acc_total_vector){                                        //Prevent the asin function to produce a NaN
		QUAD.angle_pitch_acc = -asin((float)MPU6050.acc_y/QUAD.acc_total_vector)* 57.296;          //Calculate the pitch angle.
	}
	  if(abs(MPU6050.acc_x) < QUAD.acc_total_vector){                                        //Prevent the asin function to produce a NaN
		QUAD.angle_roll_acc = -asin((float)MPU6050.acc_x/QUAD.acc_total_vector)* -57.296;          //Calculate the roll angle.
	}

	if(starting == 1){
		starting =0;
		QUAD.angle_pitch = QUAD.angle_pitch_acc;
		QUAD.angle_roll  = QUAD.angle_roll_acc;


		//Reset the PID controllers for bumpless starting
		QUAD_PID.pid_i_mem_roll=0;
		QUAD_PID.pid_i_mem_pitch=0;
		QUAD_PID.pid_i_mem_yaw=0;

		QUAD_PID.pid_last_roll_d_error=0;
		QUAD_PID.pid_last_pitch_d_error=0;
		QUAD_PID.pid_last_yaw_d_error=0;
	}
	else{

#ifdef TUNE_COMPLEMENTARY_FILTER
		QUAD.angle_pitch = QUAD.angle_pitch * (1-(((float)RX_DATA.CH5-1000)/1000000)) + QUAD.angle_pitch_acc * (((float)RX_DATA.CH5-1000)/1000000);            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
		QUAD.angle_roll  = QUAD.angle_roll * (1-(((float)RX_DATA.CH5-1000)/1000000))  + QUAD.angle_roll_acc * (((float)RX_DATA.CH5-1000)/1000000);               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
#else
		QUAD.angle_pitch = QUAD.angle_pitch * 0.9993 + QUAD.angle_pitch_acc * 0.0007;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
		QUAD.angle_roll  = QUAD.angle_roll * 0.9993  + QUAD.angle_roll_acc * 0.0007;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
#endif
	}

	QUAD.pitch_level_adjust = QUAD.angle_pitch*15;
	QUAD.roll_level_adjust = QUAD.angle_roll*15;


	//The PID set point in degrees per second is determined by the roll receiver input.
	//In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	QUAD_PID.pid_roll_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if(RX_DATA.CH1 > 1508)QUAD_PID.pid_roll_setpoint = RX_DATA.CH1 - 1508;
	else if(RX_DATA.CH1 < 1492)QUAD_PID.pid_roll_setpoint = RX_DATA.CH1 - 1492;

	QUAD_PID.pid_roll_setpoint -= QUAD.roll_level_adjust;
	QUAD_PID.pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.



	//The PID set point in degrees per second is determined by the pitch receiver input.
	//In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	QUAD_PID.pid_pitch_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if(RX_DATA.CH2 > 1508)QUAD_PID.pid_pitch_setpoint = RX_DATA.CH2 - 1508;
	else if(RX_DATA.CH2 < 1492)QUAD_PID.pid_pitch_setpoint = RX_DATA.CH2 - 1492;

	QUAD_PID.pid_pitch_setpoint -= QUAD.pitch_level_adjust;
	QUAD_PID.pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.


	//The PID set point in degrees per second is determined by the yaw receiver input.
	//In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	QUAD_PID.pid_yaw_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if(RX_DATA.CH3 > 1050){ //Do not yaw when turning off the motors.
	  if(RX_DATA.CH4 > 1508)QUAD_PID.pid_yaw_setpoint = (RX_DATA.CH4 - 1508)/3.0;
	  else if(RX_DATA.CH4 < 1492)QUAD_PID.pid_yaw_setpoint = (RX_DATA.CH4 - 1492)/3.0;
	}


	Calculate_PID();

	QUAD.throttle = RX_DATA.CH3;
	if (QUAD.throttle > 1800) QUAD.throttle = 1800;



	QUAD.esc_LB = QUAD.throttle - QUAD_PID.pid_output_pitch - QUAD_PID.pid_output_roll - QUAD_PID.pid_output_yaw;
	QUAD.esc_LT = QUAD.throttle + QUAD_PID.pid_output_pitch - QUAD_PID.pid_output_roll + QUAD_PID.pid_output_yaw;
	QUAD.esc_RT = QUAD.throttle + QUAD_PID.pid_output_pitch + QUAD_PID.pid_output_roll - QUAD_PID.pid_output_yaw;
	QUAD.esc_RB = QUAD.throttle - QUAD_PID.pid_output_pitch + QUAD_PID.pid_output_roll + QUAD_PID.pid_output_yaw;

    if(QUAD.esc_LB > 2000)QUAD.esc_LB = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(QUAD.esc_LT > 2000)QUAD.esc_LT = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(QUAD.esc_RT > 2000)QUAD.esc_RT = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(QUAD.esc_RB > 2000)QUAD.esc_RB = 2000;                                           //Limit the esc-4 pulse to 2000us.

#ifndef TEST_MOTOR_DIRECTIONS_ON_QUAD
    if(QUAD.esc_LB < 1100)QUAD.esc_LB = 1100;                                           //Limit the esc-1 pulse to 1100us.
    if(QUAD.esc_LT < 1100)QUAD.esc_LT = 1100;                                           //Limit the esc-2 pulse to 1100us.
    if(QUAD.esc_RT < 1100)QUAD.esc_RT = 1100;                                           //Limit the esc-3 pulse to 1100us.
    if(QUAD.esc_RB < 1100)QUAD.esc_RB = 1100;                                           //Limit the esc-4 pulse to 1100us.
#endif
    SET_PWM_Value(QUAD.esc_RT, QUAD.esc_RB, QUAD.esc_LT, QUAD.esc_LB);


#ifdef QUAD_DEBUG
//	sprintf(message, "time: %u \t pitch: %f \t roll: %f\r\n", time_elapsed, QUAD.angle_pitch_acc, QUAD.angle_roll_acc);
#ifdef TUNE_COMPLEMENTARY_FILTER
	sprintf(message, "\rtime: %u \t CF: %f \tpitch: %f \t roll: %f\r\n", time_elapsed,(((float)RX_DATA.CH5-1000)/1000000) , QUAD.angle_pitch, QUAD.angle_roll);
#else
//	sprintf(message, "time: %u \t pitch: %f \t roll: %f\r\n", time_elapsed, QUAD.angle_pitch, QUAD.angle_roll);
//	sprintf(message, "time: %u \t roll_setpoint: %f \t pitch_setpoint: %f \t yaw_setpoint: %f\n", time_elapsed, QUAD_PID.pid_roll_setpoint, QUAD_PID.pid_pitch_setpoint, QUAD_PID.pid_yaw_setpoint);
//	sprintf(message, "time: %u \t roll: %f \t pitch: %f \t yaw: %f\n", time_elapsed, QUAD_PID.pid_output_roll, QUAD_PID.pid_output_pitch, QUAD_PID.pid_output_yaw);
//	sprintf(message, "time: %u \t throttle: %d \t LB: %d \t LT: %d \t RT: %d \t RB: %d\n", time_elapsed, QUAD.throttle, QUAD.esc_LB, QUAD.esc_LT, QUAD.esc_RT, QUAD.esc_RB);

#endif

//	sprintf(message, "time: %u \t pitch: %f \t roll: %f\r\n", time_elapsed, QUAD.angle_pitch, QUAD.angle_roll);
//	sprintf(message, "roll: %f  pitch: %f  yaw: %f\r\n", QUAD_PID.gyro_roll_input, QUAD_PID.gyro_pitch_input, QUAD_PID.gyro_yaw_input);
//	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif
}

void Calculate_PID(){
	//Roll calculations
	float pid_error_temp;

	pid_error_temp = QUAD_PID.gyro_roll_input + QUAD_PID.pid_roll_setpoint;
	QUAD_PID.pid_i_mem_roll += QUAD_PID.pid_i_gain_roll * pid_error_temp;
	if(QUAD_PID.pid_i_mem_roll > QUAD_PID.pid_max_roll)QUAD_PID.pid_i_mem_roll = QUAD_PID.pid_max_roll;
	else if(QUAD_PID.pid_i_mem_roll < QUAD_PID.pid_max_roll * -1)QUAD_PID.pid_i_mem_roll = QUAD_PID.pid_max_roll * -1;

	QUAD_PID.pid_output_roll = QUAD_PID.pid_p_gain_roll * pid_error_temp + QUAD_PID.pid_i_mem_roll + QUAD_PID.pid_d_gain_roll * (pid_error_temp - QUAD_PID.pid_last_roll_d_error);
	if(QUAD_PID.pid_output_roll > QUAD_PID.pid_max_roll)QUAD_PID.pid_output_roll = QUAD_PID.pid_max_roll;
	else if(QUAD_PID.pid_output_roll < QUAD_PID.pid_max_roll * -1)QUAD_PID.pid_output_roll = QUAD_PID.pid_max_roll * -1;

	QUAD_PID.pid_last_roll_d_error = pid_error_temp;

#ifdef QUAD_DEBUG
//	sprintf(message, "roll_error: %f\r\n", pid_error_temp);
//	sprintf(message, "P: %f\r\n", QUAD_PID.pid_output_roll);
//	sprintf(message, "roll_error: %f \t setpoint: %f \t measured:%f \r\n", pid_error_temp,QUAD_PID.pid_roll_setpoint ,QUAD_PID.gyro_roll_input );
//	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

	//Pitch calculations
	pid_error_temp = QUAD_PID.gyro_pitch_input + QUAD_PID.pid_pitch_setpoint;
	QUAD_PID.pid_i_mem_pitch += QUAD_PID.pid_i_gain_pitch * pid_error_temp;
	if(QUAD_PID.pid_i_mem_pitch > QUAD_PID.pid_max_pitch)QUAD_PID.pid_i_mem_pitch = QUAD_PID.pid_max_pitch;
	else if(QUAD_PID.pid_i_mem_pitch < QUAD_PID.pid_max_pitch * -1)QUAD_PID.pid_i_mem_pitch = QUAD_PID.pid_max_pitch * -1;

	QUAD_PID.pid_output_pitch = QUAD_PID.pid_p_gain_pitch * pid_error_temp + QUAD_PID.pid_i_mem_pitch + QUAD_PID.pid_d_gain_pitch * (pid_error_temp - QUAD_PID.pid_last_pitch_d_error);
	if(QUAD_PID.pid_output_pitch > QUAD_PID.pid_max_pitch)QUAD_PID.pid_output_pitch = QUAD_PID.pid_max_pitch;
	else if(QUAD_PID.pid_output_pitch < QUAD_PID.pid_max_pitch * -1)QUAD_PID.pid_output_pitch = QUAD_PID.pid_max_pitch * -1;

	QUAD_PID.pid_last_pitch_d_error = pid_error_temp;

#ifdef QUAD_DEBUG
//	sprintf(message, "roll_error: %f\r\n", pid_error_temp);
//	sprintf(message, "P: %f\r\n", pid_error_temp);
//	sprintf(message, "roll_error: %f \t setpoint: %f \t measured:%f \r\n", pid_error_temp,QUAD_PID.pid_pitch_setpoint ,QUAD_PID.gyro_pitch_input );
//	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

	//Yaw calculations
	pid_error_temp = QUAD_PID.gyro_yaw_input + QUAD_PID.pid_yaw_setpoint;
	QUAD_PID.pid_i_mem_yaw += QUAD_PID.pid_i_gain_yaw * pid_error_temp;
	if(QUAD_PID.pid_i_mem_yaw > QUAD_PID.pid_max_yaw)QUAD_PID.pid_i_mem_yaw = QUAD_PID.pid_max_yaw;
	else if(QUAD_PID.pid_i_mem_yaw < QUAD_PID.pid_max_yaw * -1)QUAD_PID.pid_i_mem_yaw = QUAD_PID.pid_max_yaw * -1;

	QUAD_PID.pid_output_yaw = QUAD_PID.pid_p_gain_yaw * pid_error_temp + QUAD_PID.pid_i_mem_yaw + QUAD_PID.pid_d_gain_yaw * (pid_error_temp - QUAD_PID.pid_last_yaw_d_error);
	if(QUAD_PID.pid_output_yaw > QUAD_PID.pid_max_yaw)QUAD_PID.pid_output_yaw = QUAD_PID.pid_max_yaw;
	else if(QUAD_PID.pid_output_yaw < QUAD_PID.pid_max_yaw * -1)QUAD_PID.pid_output_yaw = QUAD_PID.pid_max_yaw * -1;

	QUAD_PID.pid_last_yaw_d_error = pid_error_temp;
}

void GET_PID_Constants(){
#ifdef PID_TUNNABLE
	if(RX_DATA.CH6>1750 && RX_DATA.CH6<2000)
		QUAD_PID.pid_d_gain_roll = RX_DATA.CH5*PID_RX_SCALE - (1000*PID_RX_SCALE);
	if(RX_DATA.CH6>1350 && RX_DATA.CH6<1750)
		QUAD_PID.pid_i_gain_roll = (RX_DATA.CH5*PID_RX_SCALE - (1000*PID_RX_SCALE))/10;
	if(RX_DATA.CH6>1000 && RX_DATA.CH6<1350)
		QUAD_PID.pid_p_gain_roll = RX_DATA.CH5*PID_RX_SCALE - (1000*PID_RX_SCALE);


	QUAD_PID.pid_p_gain_pitch = QUAD_PID.pid_p_gain_roll;
	QUAD_PID.pid_i_gain_pitch = QUAD_PID.pid_i_gain_roll;
	QUAD_PID.pid_d_gain_pitch = QUAD_PID.pid_d_gain_roll;

#else if PD_TUNNABLE
	QUAD_PID.pid_p_gain_roll = RX_DATA.CH5*PID_RX_SCALE_P - (1000*PID_RX_SCALE_P);
	QUAD_PID.pid_d_gain_roll = 2*(RX_DATA.CH6*PID_RX_SCALE_D - (1000*PID_RX_SCALE_D));

	QUAD_PID.pid_p_gain_pitch = QUAD_PID.pid_p_gain_roll;
	QUAD_PID.pid_d_gain_pitch = QUAD_PID.pid_d_gain_roll;
#endif
#ifdef QUAD_DEBUG
	sprintf(message, "roll_p: %f  roll_d: %f  roll_i: %f\r\n", QUAD_PID.pid_p_gain_roll, QUAD_PID.pid_d_gain_roll, QUAD_PID.pid_i_gain_roll);
	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif
}

void Calibrate_Receiver(){
#ifdef QUAD_DEBUG
	sprintf(message, "Calibration Started\r\n");
	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

	while(RX_DATA.RX_Valid == 0);

	set_Leds(1,1,1,1);
	while(1){
		if(RX_DATA.CH3 < 1100 && RX_DATA.CH4<1100 && RX_DATA.CH1 > 1900 && RX_DATA.CH2 > 1900)
			break;
		if (RX_DATA.CH1 < 100 || RX_DATA.CH2 < 100 || RX_DATA.CH3 < 100 || RX_DATA.CH4 < 100 || RX_DATA.CH5 < 100 || RX_DATA.CH6 < 100)
			continue;

	    if(RX_DATA.CH1_max < RX_DATA.CH1) RX_DATA.CH1_max = RX_DATA.CH1;
	    if(RX_DATA.CH2_max < RX_DATA.CH2) RX_DATA.CH2_max = RX_DATA.CH2;
	    if(RX_DATA.CH3_max < RX_DATA.CH3) RX_DATA.CH3_max = RX_DATA.CH3;
	    if(RX_DATA.CH4_max < RX_DATA.CH4) RX_DATA.CH4_max = RX_DATA.CH4;

	    if(RX_DATA.CH1_min > RX_DATA.CH1) RX_DATA.CH1_min = RX_DATA.CH1;
	    if(RX_DATA.CH2_min > RX_DATA.CH2) RX_DATA.CH2_min = RX_DATA.CH2;
	    if(RX_DATA.CH3_min > RX_DATA.CH3) RX_DATA.CH3_min = RX_DATA.CH3;
	    if(RX_DATA.CH4_min > RX_DATA.CH4) RX_DATA.CH4_min = RX_DATA.CH4;
		}
#ifdef QUAD_DEBUG
	sprintf(message, "Calibration Done..!!!\r\n");
	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

	RX_DATA.CH1_offset = ((RX_DATA.CH1_max + RX_DATA.CH1_min) / 2) - 1500;		//offset to be subtracted from receiver data
	RX_DATA.CH2_offset = ((RX_DATA.CH2_max + RX_DATA.CH2_min) / 2) - 1500;
	RX_DATA.CH3_offset = ((RX_DATA.CH3_max + RX_DATA.CH3_min) / 2) - 1500;
	RX_DATA.CH4_offset = ((RX_DATA.CH4_max + RX_DATA.CH4_min) / 2) - 1500;

	RX_DATA.CH1_scale = 1000 / (float)(RX_DATA.CH1_max - RX_DATA.CH1_min);				//scale to be multiplied to the receiver data
	RX_DATA.CH2_scale = 1000 / (float)(RX_DATA.CH2_max - RX_DATA.CH2_min);
	RX_DATA.CH3_scale = 1000 / (float)(RX_DATA.CH3_max - RX_DATA.CH3_min);
	RX_DATA.CH4_scale = 1000 / (float)(RX_DATA.CH4_max - RX_DATA.CH4_min);

	RX_DATA.Calibration_status = 1;


	HAL_Delay(50);

	while(RX_DATA.CH1<1900)Toggle_Leds_with_delay(50);
	while(RX_DATA.CH2<1900)Toggle_Leds_with_delay(50);
	while(RX_DATA.CH3<1900)Toggle_Leds_with_delay(50);
	while(RX_DATA.CH4<1900)Toggle_Leds_with_delay(50);

	while(RX_DATA.CH1>1100)Toggle_Leds_with_delay(50);
	while(RX_DATA.CH2>1100)Toggle_Leds_with_delay(50);
	while(RX_DATA.CH3>1100)Toggle_Leds_with_delay(50);
	while(RX_DATA.CH4>1100)Toggle_Leds_with_delay(50);

	set_Leds(0,0,0,0);

}

void manual_ESC_calibration(){
//	SET_PWM_Value(2000,2000,2000,2000);
	while((RX_DATA.CH1 < 100 || RX_DATA.CH2 < 100 || RX_DATA.CH3 < 100 || RX_DATA.CH4 < 100 || RX_DATA.CH5 < 100 || RX_DATA.CH6 < 100));
	PWM_Start();
//	while((RX_DATA.CH4>1100)){
//		if(RX_DATA.CH3>1600)
//			SET_PWM_Value(2000,2000,2000,2000);
//		else if(RX_DATA.CH3<1400)
//			SET_PWM_Value(1000,1000,1000,1000);
//	}
	while(1)
		SET_PWM_Value(RX_DATA.CH3,RX_DATA.CH3,RX_DATA.CH3,RX_DATA.CH3);
}

void Calibrate_ESCs(){
	SET_PWM_Value(2000,2000,2000,2000);
	PWM_Start();
	SET_PWM_Value(2000,2000,2000,2000);
	HAL_Delay(4000);
	SET_PWM_Value(1000,1000,1000,1000);
	HAL_Delay(6000);
//	SET_PWM_Value(900,900,900,900);			//TO make sure the motors does not start
//	while(1)
//		SET_PWM_Value(RX_DATA.CH3,RX_DATA.CH3,RX_DATA.CH3,RX_DATA.CH3);
}

void set_Leds(_Bool Green, _Bool Orange, _Bool Red, _Bool Blue){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, Green);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, Orange);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, Red);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, Blue);
}

void Check_for_UnARM_Motor_Command(){
	if(RX_DATA.CH3 < 1100 && RX_DATA.CH4<1100){
		while(RX_DATA.CH4<1400 && RX_DATA.CH3 < 1100) {
			SET_PWM_Value(900, 900, 900, 900);
			Led_Rotate_CCW(50,1);
			#ifdef QUAD_DEBUG
				sprintf(message, "Quad Motors UnARMED\r\n");
				HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
			#endif

		}
		while(1){
			if(RX_DATA.CH4<1400 && RX_DATA.CH3 < 1100)
				break;

			if(RX_DATA.CH4>1600 && RX_DATA.CH3 > 1800){
				while(RX_DATA.CH3 > 1100);
				while(RX_DATA.CH4>1100)
					SET_PWM_Value(RX_DATA.CH3,RX_DATA.CH3,RX_DATA.CH3,RX_DATA.CH3);
				while(RX_DATA.CH4<1400 || RX_DATA.CH3 > 1100);
			}
			else{
				SET_PWM_Value(900, 900, 900, 900);
			 	Led_Rotate_CCW(50,1);
			}


		}
		while(RX_DATA.CH4<1400 || RX_DATA.CH3 > 1100){
			SET_PWM_Value(900, 900, 900, 900);
			Led_Rotate_CCW(50,1);
		}

#ifdef QUAD_DEBUG
	sprintf(message, "Quad Motors ARMED...!!!\r\n");
	HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif
	}
}

void ARM_Quad(){
	 //Wait until the receiver is active and the throtle is set to the lower position.
	int start=0;
	int i=0;
	set_Leds(0,0,0,0);
	  while(1){
		if(RX_DATA.CH3 < 1100 && RX_DATA.CH4<1100 && RX_DATA.CH1 > 1900 && RX_DATA.CH2 > 1900)
			break;
	    start ++;                                                               //While waiting increment start whith every loop.
	    SET_PWM_Value(1000,1000,1000,1000);													//To avoid escs beeping. Give 1000us pulse
	    HAL_Delay(5);
	    if(start == 125){                                                       //Every 125 loops (500ms).
	      Toggle_Leds_without_delay();
	      start = 0;                                                            //Start again at 0.
	    }
	  }
	  start =0;
	  while(1){
		if(RX_DATA.CH3 < 1100 && RX_DATA.CH4<1600 && RX_DATA.CH4>1400 && RX_DATA.CH2 > 1400 && RX_DATA.CH2 < 1600 && RX_DATA.CH1 < 1600 && RX_DATA.CH1 > 1400)
		{
			i++;
			if(i==1000)
				break;
		}
		else
			i=0;
	    start ++;                                                               //While waiting increment start whith every loop.
	    SET_PWM_Value(1000,1000,1000,1000);													//To avoid escs beeping. Give 1000us pulse
	    HAL_Delay(1);
	    if(start == 125){                                                       //Every 125 loops (500ms).
	      Toggle_Leds_without_delay();
	      start = 0;                                                            //Start again at 0.
	    }
	  }
	  set_Leds(0,0,0,0);
}

void Toggle_Leds_with_delay(int delay){
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_12);
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_13);
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_14);
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_15);
	HAL_Delay(delay);
}

void Toggle_Leds_without_delay(){
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_12);
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_13);
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_14);
	HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_15);
}

void Toggle_Leds_with_delay_cycles(int delay, int cycles){
	for(int i=0;i<cycles;i++){
		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_12);
		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_13);
		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_14);
		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_15);
		HAL_Delay(delay);
	}
}

void Led_Rotate_CW(int delay,int cycles){
	for(int i=0;i<cycles;i++){
		set_Leds(1,0,0,0);
		HAL_Delay(delay);

		set_Leds(0,1,0,0);
		HAL_Delay(delay);

		set_Leds(0,0,1,0);
		HAL_Delay(delay);

		set_Leds(0,0,0,1);
		HAL_Delay(delay);
	}
	set_Leds(0,0,0,0);
}

void Led_Rotate_CCW(int delay,int cycles){
	for(int i=0;i<cycles;i++){
		set_Leds(0,0,0,1);
		HAL_Delay(delay);

		set_Leds(0,0,1,0);
		HAL_Delay(delay);

		set_Leds(0,1,0,0);
		HAL_Delay(delay);

		set_Leds(1,0,0,0);
		HAL_Delay(delay);
	}
	set_Leds(0,0,0,0);
}
