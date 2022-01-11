/*
 * pid_bldc_regulator.h
 *
 *  Created on: Dec 25, 2021
 *      Author: Kacper Bukowski
 */

#ifndef INC_PID_BLDC_REGULATOR_H_
#define INC_PID_BLDC_REGULATOR_H_

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int previous_error;
	int total_error;
	int anti_windup;
}pid_bldc_controller_structure;

void pid_init(pid_bldc_controller_structure *, float, float, float, int);
void pid_reset(pid_bldc_controller_structure *);
int pid_calculate(pid_bldc_controller_structure *, int, int);

#endif /* INC_PID_BLDC_REGULATOR_H_ */
