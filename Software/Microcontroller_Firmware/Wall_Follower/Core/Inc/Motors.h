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
#include "stm32f1xx_hal.h"

#define MAX_MOTOR_SPEED	7*2*M_PI		// Maximum motor speed in rad/s
#define MIN_MOTOR_SPEED 1*2*M_PI

typedef struct {
	TIM_HandleTypeDef* motor_timer;
	GPIO_TypeDef* dir_pin_port;
	uint16_t dir_pin;
	float rate_rps;				// Motor's speed in rad/s
}motor_t;

void drive_motor_speed(motor_t *motor, float speed_rps);
void stop_motor(motor_t *motor);
void calibrate_spin(uint8_t num_rotations, float speed_rps, motor_t *left_motor, motor_t *right_motor);
void calibrate_drive(float dist_m, float speed_metres_per_s, motor_t *left_motor, motor_t *right_motor);

#endif /* INC_MOTORS_H_ */
