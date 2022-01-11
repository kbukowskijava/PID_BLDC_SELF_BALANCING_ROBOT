/*
 * pid_bldc_regulator.c
 *
 *  Created on: Dec 25, 2021
 *      Author: Kacper Bukowski
 */

#include "main.h"

void pid_init(pid_bldc_controller_structure *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_init)
{
	pid_data->previous_error = 0;
	pid_data->total_error = 0;

	pid_data->Kp = kp_init;
	pid_data->Ki = ki_init;
	pid_data->Kd = kd_init;

	pid_data->anti_windup = anti_windup_init;
}

void pid_reset(pid_bldc_controller_structure *pid_data)
{
	pid_data->total_error = 0;
	pid_data->previous_error = 0;
}

int pid_calculate(pid_bldc_controller_structure *pid_data, int setpoint, int process_variable)
{
	int error;
	float p_term, i_term, d_term;

	error = setpoint - process_variable;
	pid_data->total_error += error;

	p_term = (float)(pid_data->Kp * error);
	i_term = (float)(pid_data->Ki * pid_data->total_error);
	d_term = (float)(pid_data->Kd * (error - pid_data->previous_error));

	if(i_term >= pid_data->anti_windup) i_term = pid_data->anti_windup;
	else if(i_term <= -pid_data->anti_windup) i_term = -pid_data->anti_windup;

	pid_data->previous_error = error;

	return (int)(p_term + i_term + d_term);
}
