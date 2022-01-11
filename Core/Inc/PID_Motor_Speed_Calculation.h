/*
 * PID_Motor_Speed_Calculation.h
 *
 *  Created on: Jan 4, 2022
 *      Author: Kacper Bukowski
 */

#ifndef INC_PID_MOTOR_SPEED_CALCULATION_H_
#define INC_PID_MOTOR_SPEED_CALCULATION_H_

typedef struct
{
	uint16_t maximum_motor_power;
	int current_pid_value;
	uint16_t max_duty_cycle;
	uint16_t min_duty_cycle;
	float min_pid_correction_value;
	float max_pid_correction_value;

}PID_Motor_Speed_Calculation_Structure;

void PID_Motor_Speed_Calculation_Init(PID_Motor_Speed_Calculation_Structure *,uint16_t,uint16_t,uint16_t,float,float);
float PID_Motor_Speed_Calculation_duty_cycle_ms_left(PID_Motor_Speed_Calculation_Structure *, int);
float PID_Motor_Speed_Calculation_duty_cycle_ms_right(PID_Motor_Speed_Calculation_Structure *, int);

#endif /*INC_PID_MOTOR_SPEED_CALCULATION_H_*/
