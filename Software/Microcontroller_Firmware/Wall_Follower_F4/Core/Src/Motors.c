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
 *	returns: written motor speed in rps
 */
float drive_motor_speed(motor_t *motor, float speed_rps) {
	if (speed_rps == 0) {
		disable_motors(motor);
	}
	HAL_GPIO_WritePin(motor->motor_enable_port, motor->motor_enable_pin, GPIO_PIN_SET);
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
		direction = 1;
	else
		direction = 0;

	// Update current motor rate
	motor->rate_rps = speed_rps;

	// PWM frequency = (core clock / prescalar * period)
	HAL_GPIO_WritePin(motor->dir_pin_port, motor->dir_pin, direction);
	float speed_ticks_per_s = (fabs(speed_rps) / (2.0 * M_PI)) * STEPS_PER_REV; // This is the same as the counter freq
	// Set timer frequency
	const int Clock_frequency = 72E6 / 8;
	motor->motor_timer->Init.Prescaler = 32;
	motor->motor_timer->Init.Period = (Clock_frequency / (motor->motor_timer->Init.Prescaler + 1)) / speed_ticks_per_s - 1;
	motor->motor_timer->Instance->CCR1 = motor->motor_timer->Init.Period / 2;
	HAL_StatusTypeDef res = HAL_TIM_PWM_Init(motor->motor_timer);
	res = HAL_TIM_PWM_Start(motor->motor_timer, TIM_CHANNEL_1);

	return fabs(speed_rps);
}

void lock_motors(motor_t *left_motor, motor_t *right_motor) {
	HAL_TIM_PWM_Stop(left_motor->motor_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(right_motor->motor_timer, TIM_CHANNEL_1);
}

void disable_motors(motor_t *motor) {
	HAL_GPIO_WritePin(motor->motor_enable_port, motor->motor_enable_pin, GPIO_PIN_RESET);
}

void calibrate_spin(float num_rotations, float speed_rps, motor_t *left_motor, motor_t *right_motor) {
	float motor_rate = (speed_rps * BASELINE_DISTANCE_M / 2) / WHEEL_RADIUS_M;
	motor_rate = drive_motor_speed(left_motor, motor_rate);
	drive_motor_speed(right_motor, motor_rate);
	speed_rps = 2 * WHEEL_RADIUS_M * motor_rate / BASELINE_DISTANCE_M;
	float run_time = (num_rotations * 2 * M_PI) / speed_rps;
	HAL_Delay(run_time * 1000);
	lock_motors(left_motor, right_motor);
	HAL_Delay(50);
	disable_motors(right_motor);
}

void calibrate_drive(float dist_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor) {
	float motor_rate = speed_metres_per_s / WHEEL_RADIUS_M;
	drive_motor_speed(left_motor, -motor_rate);
	motor_rate = drive_motor_speed(right_motor, motor_rate);
	float run_time = dist_m / (motor_rate * (float) WHEEL_RADIUS_M);
	HAL_Delay(run_time * 1000);
	lock_motors(left_motor, right_motor);
	HAL_Delay(50);
	disable_motors(right_motor);
}

void calibrate_drive_circle(float arc_length, float radius_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor) {
	float robot_angular_vel = speed_metres_per_s / radius_m;
	float left_wheel_vel = robot_angular_vel * (radius_m - BASELINE_DISTANCE_M / 2);
	float right_wheel_vel = robot_angular_vel * (radius_m + BASELINE_DISTANCE_M / 2);
	float left_wheel_rate = left_wheel_vel / WHEEL_RADIUS_M;
	float right_wheel_rate = right_wheel_vel / WHEEL_RADIUS_M;
	left_wheel_vel = drive_motor_speed(left_motor, -left_wheel_rate) * WHEEL_RADIUS_M;
	right_wheel_vel = drive_motor_speed(right_motor, right_wheel_rate) * WHEEL_RADIUS_M;
	robot_angular_vel = right_wheel_vel / (radius_m + BASELINE_DISTANCE_M / 2);
	float run_time = arc_length / robot_angular_vel;
	HAL_Delay(run_time * 1000);
	lock_motors(left_motor, right_motor);
	HAL_Delay(50);
	disable_motors(right_motor);
}

float turn_angle(float angle_rad, float speed_rps, motor_t *left_motor, motor_t *right_motor) {
	float motor_rate = (speed_rps * BASELINE_DISTANCE_M / 2) / WHEEL_RADIUS_M;
	float dir = 1;
	if (angle_rad >= 0) {
		dir = 1;
	} else {
		dir = -1;
	}
	motor_rate = drive_motor_speed(left_motor, dir * motor_rate);
	drive_motor_speed(right_motor, dir * motor_rate);

	speed_rps = 2 * WHEEL_RADIUS_M * motor_rate / BASELINE_DISTANCE_M;
	float run_time = fabs(angle_rad / speed_rps);
	HAL_Delay(run_time * 1000);
	lock_motors(left_motor, right_motor);
	HAL_Delay(50);
	disable_motors(right_motor);
	return dir * run_time * speed_rps;
}

//float turn_angle_smooth(float angle_rad, float speed_rps, motor_t *left_motor, motor_t *right_motor) {
//	float desired_motor_rate = (speed_rps * BASELINE_DISTANCE_M / 2) / WHEEL_RADIUS_M;
//	float dir = 1;
//	if (angle_rad >= 0) {
//		dir = 1;
//	} else {
//		dir = -1;
//	}
//	// Rate ramp period
//	uint8_t num_steps = 200;
//	float delta_rate = desired_motor_rate - MIN_MOTOR_SPEED;
//	float delta_omega = 2 * WHEEL_RADIUS_M * delta_rate / BASELINE_DISTANCE_M;	// Change in angular vel of rover
//	float rate_step = delta_rate / num_steps;
//	float ramp_time = fabs(delta_omega) / RATE_RAMP_GRAD;
//	float time_step = ramp_time / num_steps;
//	float drive_speed = MIN_MOTOR_SPEED;
//
//	uint32_t start_time = millis();
//	// Ramp up
//	for (uint8_t i = 0; i < num_steps; i++) {
//		drive_motor_speed(left_motor, dir * drive_speed);
//		drive_motor_speed(right_motor, dir * drive_speed);
//		drive_speed += rate_step;
//		HAL_Delay(time_step * 1000);
//	}
//	uint32_t actual_ramp_time = millis() - start_time;
//	float tangential_distance_travelled = actual_ramp_time * delta_rate * WHEEL_RADIUS_M / 1000;
//	float ramp_angle_turned = tangential_distance_travelled / (BASELINE_DISTANCE_M / 2);
//	turn_angle(angle_rad - 2 * ramp_angle_turned, speed_rps, left_motor, right_motor);
//
//	// Ramp down
//	for (uint8_t i = 0; i < num_steps; i++) {
//		drive_motor_speed(left_motor, dir * drive_speed);
//		drive_motor_speed(right_motor, dir * drive_speed);
//		drive_speed -= rate_step;
//		HAL_Delay(time_step * 1000);
//	}
//	lock_motors(left_motor, right_motor);
//	HAL_Delay(50);
//	disable_motors(right_motor);
//return angle_rad;
//}

float drive_distance(float dist_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor) {
	float dir = 1;
	if(dist_m < 0) {
		dir = -1;
	}
	float motor_rate = speed_metres_per_s / WHEEL_RADIUS_M;
	drive_motor_speed(left_motor, -dir*motor_rate);
	motor_rate = drive_motor_speed(right_motor, dir*motor_rate);
	float run_time = fabs(dist_m) / (motor_rate * (float) WHEEL_RADIUS_M);
	HAL_Delay(run_time * 1000);
	lock_motors(left_motor, right_motor);
	HAL_Delay(50);
	disable_motors(right_motor);
	return dir*speed_metres_per_s * run_time;
}

void drive_forward(float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor) {
	float motor_rate = speed_metres_per_s / WHEEL_RADIUS_M;
	drive_motor_speed(left_motor, -motor_rate);
	motor_rate = drive_motor_speed(right_motor, motor_rate);
}

/*
 * Function: drive_forward_smooth
 * Drives the robot with a smooth ramp upwards in speed
 * Inputs:
 * float speed_metres_per_s: Desired linear speed
 *  motor_t *left_motor: Left motor struct
 *  motor_t *right_motor: Right motor struct
 *  Returns: Distance traveled in metres during ramp up time
 */
float drive_forward_smooth(float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor) {
	float desired_motor_rate = speed_metres_per_s / WHEEL_RADIUS_M;
	if (desired_motor_rate > MAX_MOTOR_SPEED)
		return -1;
// Velocity ramp period
	uint8_t num_steps = 200;
	float delta_rate = desired_motor_rate - MIN_MOTOR_SPEED;
	float rate_step = delta_rate / num_steps;
	float ramp_time = delta_rate * WHEEL_RADIUS_M / VELOCITY_RAMP_GRAD;
	float time_step = ramp_time / num_steps;
	float drive_speed = MIN_MOTOR_SPEED;

	uint32_t start_time = millis();
	for (uint8_t i = 0; i < num_steps; i++) {
		drive_motor_speed(left_motor, -drive_speed);
		drive_motor_speed(right_motor, drive_speed);
		drive_speed += rate_step;
		HAL_Delay(time_step * 1000);
	}
	uint32_t actual_ramp_time = millis() - start_time;
	return actual_ramp_time * delta_rate * WHEEL_RADIUS_M / 1000;
}

void turn_scan(float angle_rad, float speed_rps, motor_t *left_motor, motor_t *right_motor, float set_of_vectors[1000][2], ultra_sonic_t *sensor) {
	float motor_rate = (speed_rps * BASELINE_DISTANCE_M / 2) / WHEEL_RADIUS_M;
	float dir = 1;
	if (angle_rad >= 0) {
		dir = 1;
	} else {
		dir = -1;
	}
	motor_rate = drive_motor_speed(left_motor, dir * motor_rate);
	drive_motor_speed(right_motor, dir * motor_rate);
	speed_rps = 2 * WHEEL_RADIUS_M * motor_rate / BASELINE_DISTANCE_M;

	float run_time = fabs(angle_rad / speed_rps);
	uint32_t num_step = 1000;
	float time_step = run_time / num_step;
	float angle_step = angle_rad / num_step;
	for (int i = 0; i < num_step; i++) {
		set_of_vectors[i][0] = angle_step * i;
		set_of_vectors[i][1] = sensor->distance_read;
		HAL_Delay(time_step * 1000);
	}
	lock_motors(left_motor, right_motor);
	HAL_Delay(50);
	disable_motors(right_motor);
}
