/*
 * Motors.h
 *
 *  Created on: 9 Aug. 2023
 *      Author: thean
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stdint.h"
#include "stdlib.h"
#include <math.h>
#include "physical_parameters.h"
#include "Sensors.h"
#include "stm32f4xx_hal.h"

#define MAX_MOTOR_SPEED 		5.0*2.0*M_PI		// Maximum motor speed in rad/s
#define MIN_MOTOR_SPEED 		0.1*2.0*M_PI
#define VELOCITY_RAMP_GRAD		0.05f				// m/s2
#define RATE_RAMP_GRAD			0.5f				// rad/s2

typedef struct {
	TIM_HandleTypeDef *motor_timer;
	GPIO_TypeDef *dir_pin_port;
	uint16_t dir_pin;
	float rate_rps;				// Motor's speed in rad/s
	GPIO_TypeDef *motor_enable_port;
	uint16_t motor_enable_pin;
} motor_t;

extern uint32_t millis();
extern uint32_t micros();

float drive_motor_speed(motor_t *motor, float speed_rps);
void lock_motors(motor_t *left_motor, motor_t *right_motor);
void disable_motors(motor_t *motor);
void calibrate_spin(float num_rotations, float speed_rps, motor_t *left_motor, motor_t *right_motor);
void calibrate_drive(float dist_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor);
void calibrate_drive_circle(float arc_length, float radius_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor);
float turn_angle(float angle_rad, float speed_rps, motor_t *left_motor, motor_t *right_motor);
float turn_angle_smooth(float angle_rad, float speed_rps, motor_t *left_motor, motor_t *right_motor);
float drive_distance(float dist_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor);
void drive_forward(float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor);
float drive_forward_smooth(float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor);
void turn_scan(float angle_rad, float speed_rps, motor_t *left_motor, motor_t *right_motor, float set_of_vectors[1000][2], ultra_sonic_t* sensor);
#endif /* INC_MOTORS_H_ */
