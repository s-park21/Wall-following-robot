/*
 * Motors.c
 *
 *  Created on: 9 Aug. 2023
 *      Author: thean
 */

#include "Motors.h"

/*
 *	Fuction: drive_left_motor_speed
 *	uint16_t speed_rps: Speed in rad/s - can be negative
 */
void drive_motor_speed(motor_t *motor, float speed_rps) {
	if (speed_rps == 0) {
		stop_motor(motor);
	}
	// Clamp input to reasonable bounds
	if (speed_rps > MAX_MOTOR_SPEED)
		speed_rps = MAX_MOTOR_SPEED;
	else if (speed_rps < MIN_MOTOR_SPEED && speed_rps > 0)
		speed_rps = MIN_MOTOR_SPEED;
	else if (speed_rps < 0 && speed_rps > -MIN_MOTOR_SPEED)
		speed_rps = -MIN_MOTOR_SPEED;
	else if (speed_rps < -MAX_MOTOR_SPEED)
		speed_rps = -MAX_MOTOR_SPEED;

	uint8_t direction;
	if (speed_rps < 0)
		direction = 0;
	else
		direction = 1;

	// Update current motor rate
	motor->rate_rps = speed_rps;

	// PWM frequency = (core clock / prescalar * period)
	HAL_GPIO_WritePin(motor->dir_pin_port, motor->dir_pin, direction);
	float speed_ticks_per_s = (abs((int) speed_rps) / (2.0 * M_PI)) * STEPS_PER_REV; // This is the same as the counter freq
	const int Clock_frequency = 36E6;	// 72MHz
	// Set timer frequency
	motor->motor_timer->Init.Prescaler = 2;
	motor->motor_timer->Init.Period = (Clock_frequency / (motor->motor_timer->Init.Prescaler + 1)) / speed_ticks_per_s - 1;
	motor->motor_timer->Instance->CCR1 = motor->motor_timer->Init.Period / 2;
	HAL_StatusTypeDef res = HAL_TIM_PWM_Init(motor->motor_timer);
	res = HAL_TIM_PWM_Start(motor->motor_timer, TIM_CHANNEL_1);
}

void stop_motor(motor_t *motor) {
	HAL_TIM_Base_Stop(motor->motor_timer);
}

void calibrate_spin(uint8_t num_rotations, float speed_rps, motor_t *left_motor, motor_t *right_motor) {
	float motor_rate = speed_rps * BASELINE_DISTANCE_M / 2;
	float run_time = num_rotations / (speed_rps / 2 * M_PI);
	drive_motor_speed(left_motor, motor_rate);
	drive_motor_speed(right_motor, -motor_rate);
	HAL_Delay(run_time * 1000);
	stop_motor(left_motor);
	stop_motor(right_motor);
}

void calibrate_drive(float dist_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor) {
	float motor_rate = speed_metres_per_s / WHEEL_RADIUS_M ;
	float run_time = speed_metres_per_s / dist_m;
	drive_motor_speed(left_motor, motor_rate);
	drive_motor_speed(right_motor, motor_rate);
	HAL_Delay(run_time * 1000);
	stop_motor(left_motor);
	stop_motor(right_motor);
}
