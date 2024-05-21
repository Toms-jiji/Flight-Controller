/*
 * Quad.h
 *
 *  Created on: Apr 9, 2024
 *      Author: Vivek
 */

#ifndef QUAD_H_
#define QUAD_H_

typedef struct Receiver{
	int CH1;
	int CH2;
	int CH3;
	int CH4;
	int CH5;
	int CH6;

	int CH1_max;
	int CH2_max;
	int CH3_max;
	int CH4_max;

	int CH1_min;
	int CH2_min;
	int CH3_min;
	int CH4_min;

	float CH1_scale;
	float CH2_scale;
	float CH3_scale;
	float CH4_scale;

	int CH1_offset;
	int CH2_offset;
	int CH3_offset;
	int CH4_offset;

	_Bool Calibration_status;
	_Bool RX_Valid;

}Struct_Receiver;

typedef struct Quad_PID{
	float pid_p_gain_roll;
	float pid_i_gain_roll;
	float pid_d_gain_roll;
	int pid_max_roll;

	float pid_p_gain_pitch;
	float pid_i_gain_pitch;
	float pid_d_gain_pitch;
	int pid_max_pitch;

	float pid_p_gain_yaw;
	float pid_i_gain_yaw;
	float pid_d_gain_yaw;
	int pid_max_yaw;

	float gyro_roll_input;
	float gyro_pitch_input;
	float gyro_yaw_input;


	float pid_i_mem_roll;
	float pid_i_mem_pitch;
	float pid_i_mem_yaw;

	float pid_last_roll_d_error;
	float pid_last_pitch_d_error;
	float pid_last_yaw_d_error;

	float pid_roll_setpoint;
	float pid_pitch_setpoint;
	float pid_yaw_setpoint;

	float pid_output_roll;
	float pid_output_pitch;
	float pid_output_yaw;

}Quad_PID_DATA;

typedef struct Quad{
	float angle_pitch;
	float angle_roll;
	long acc_total_vector;
	float angle_pitch_acc;
	float angle_roll_acc;

	float pitch_level_adjust;
	float roll_level_adjust;

	int throttle;
	int esc_LB;
	int esc_LT;
	int esc_RB;
	int esc_RT;
}QUAD_ORIENTATION;



void quad_init();
void quad_fly();
void ARM_Quad();
void set_Leds(_Bool Green, _Bool Orange, _Bool Red, _Bool Blue);
void Led_Rotate_CW(int delay,int cycles);
void Led_Rotate_CCW(int delay,int cycles);
void Toggle_Leds();
void Toggle_Leds_with_delay_cycles(int delay, int cycles);
void Toggle_Leds_without_delay();
void Calibrate_ESCs();
void Toggle_Leds_with_delay(int delay);
void Calibrate_Receiver();
void GET_PID_Constants();
void Check_for_UnARM_Motor_Command();
void Calculate_PID();
void manual_ESC_calibration();

#endif /* QUAD_H_ */
