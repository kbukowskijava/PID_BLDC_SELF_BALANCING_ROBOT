/*
 * PID_Motor_Speed_Calculation.c
 *
 *  Created on: Jan 4, 2022
 *      Author: Kacper Bukowski
 */

#include "main.h"
#include "stdlib.h"


void PID_Motor_Speed_Calculation_Init(PID_Motor_Speed_Calculation_Structure *pid_bldc_controller_data, uint16_t maximum_motor_power,uint16_t max_duty_cycle,uint16_t min_duty_cycle, float min_pid_correction_value, float max_pid_correction_value){
	pid_bldc_controller_data->max_duty_cycle = ((float)maximum_motor_power/100 * (max_duty_cycle-min_duty_cycle))+min_duty_cycle;
	pid_bldc_controller_data->min_duty_cycle = min_duty_cycle;
	pid_bldc_controller_data->maximum_motor_power = maximum_motor_power;
	pid_bldc_controller_data->min_pid_correction_value = min_pid_correction_value;
	pid_bldc_controller_data->max_pid_correction_value = max_pid_correction_value;
}

float PID_Motor_Speed_Calculation_duty_cycle_ms_left(PID_Motor_Speed_Calculation_Structure *pid_bldc_controller_data, int current_pid_value){
	float ms_value_after_limit = ((((pid_bldc_controller_data->max_duty_cycle-pid_bldc_controller_data->min_duty_cycle)/100)*pid_bldc_controller_data->maximum_motor_power));
	if (current_pid_value < 0){
		float output = (abs(current_pid_value*ms_value_after_limit/pid_bldc_controller_data->min_pid_correction_value))+pid_bldc_controller_data->min_duty_cycle;
		return output;
	}
	else{
		return 1210;
	}
}

float PID_Motor_Speed_Calculation_duty_cycle_ms_right(PID_Motor_Speed_Calculation_Structure *pid_bldc_controller_data, int current_pid_value){
	float ms_value_after_limit = (((pid_bldc_controller_data->max_duty_cycle-pid_bldc_controller_data->min_duty_cycle)/100)*pid_bldc_controller_data->maximum_motor_power);
	if (current_pid_value > 0){
		return (abs(current_pid_value*ms_value_after_limit/pid_bldc_controller_data->min_pid_correction_value))+pid_bldc_controller_data->min_duty_cycle;
	}
	else{
		return 1210;
	}
}

