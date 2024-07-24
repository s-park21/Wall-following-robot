/*
 * Motor_Control.h
 *
 *  Created on: Aug 3, 2023
 *      Author: Angus McLennan
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include <math.h>
#include <stdint.h>
#include "physical_params.h"


typedef struct {
	float state_vector[3];						// State vector [x, y, theta]'
	float lin_vel;								// Linear speed of robot
	float ang_vel;								// Angular velocity of robot (theta dot)
	float P_matrix[9];							// Estimate covariance matrix (3x3)
	float R_matrix[9];							// Measurement covariance matrix (3x3)
	float left_wheel_covariance;
	float right_wheel_covariance;
	float current_wheel_vel[2];					// Current wheel velocity (ticks/s) [left wheel, right wheel]
	float prev_wheel_vel[2];					// Previous wheel velocity (tick/s) [left wheel, right wheel]
}ekf_t;

void convert_wheel_speeds(ekf_t *ekf, float left_speed, float right_speed);
void ekf_predict(ekf_t *ekf, float left_speed, float right_speed, float dt);
void ekf_calculate_estimate_covariance(ekf_t *ekf, float left_speed, float right_speed, float dt, float* covar);
void matrix_multiply(const float* A, const float* B, uint32_t A_rows, uint32_t A_cols, uint32_t B_rows, uint32_t B_cols, float* output);
void matrix_transpose(const float* A, uint32_t rows, uint32_t cols, float* output);
void matrix_add(const float* A, const float* B, uint32_t rows, uint32_t cols, float* output);

#endif /* INC_MOTOR_CONTROL_H_ */
