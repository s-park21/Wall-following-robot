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
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef* Right_Wheel_Encoder_Tim;
extern TIM_HandleTypeDef* Left_Wheel_Encoder_Tim;
extern TIM_HandleTypeDef* Micros_Timer;
extern TIM_HandleTypeDef* PWM_Timer;
extern uint16_t Micros(TIM_HandleTypeDef* Micros_Timer);

typedef struct {
	float state_vector[3];						// State vector [x, y, theta]'
	float lin_vel;								// Linear speed of robot
	float ang_vel;								// Angular velocity of robot (theta dot)
	float P_matrix[9];							// Estimate covariance matrix (3x3)
	float R_matrix[9];							// Measurement covariance matrix (3x3)
	float left_wheel_covariance;
	float right_wheel_covariance;
	float wheel_ang_vel[2];						// Current wheel velocity (ticks/s) [left wheel, right wheel]
	uint16_t encoder_read_time_us;				// Last time the encoder was read in microseconds
	uint16_t prev_encoder_value[2];				// Last encoder reading value [left wheel, right wheel]
}ekf_t;

typedef struct {
	float kp;
	float ki;
	float kd;
	float prev_left_wheel_error;
	float prev_right_wheel_error;
	float left_wheel_integral_error;
	float right_wheel_integral_error;
	uint16_t pid_update_time_us;				// Last time that the PID loop was run in microseconds
} wheel_pid_t;

void convert_wheel_speeds(ekf_t *ekf, float left_speed, float right_speed);
void ekf_predict(ekf_t *ekf, float left_speed, float right_speed, float dt);
void ekf_calculate_estimate_covariance(ekf_t *ekf, float left_speed, float right_speed, float dt, float* covar);
void ekf_calcualte_state_transition_matrix(ekf_t *ekf, float left_speed, float right_speed, float dt, float *state_transition_matrix);
void matrix_multiply(const float* A, const float* B, uint32_t A_rows, uint32_t A_cols, uint32_t B_rows, uint32_t B_cols, float* output);
void matrix_transpose(const float* A, uint32_t rows, uint32_t cols, float* output);
void matrix_add(const float* A, const float* B, uint32_t rows, uint32_t cols, float* output);
void wheel_speed_pid_update(wheel_pid_t *pid, ekf_t *ekf, float left_wheel_speed, float right_wheel_speed);
#endif /* INC_MOTOR_CONTROL_H_ */
