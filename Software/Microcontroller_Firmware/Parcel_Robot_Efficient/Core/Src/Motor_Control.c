/*
 * Motor_Control.c
 *
 *  Created on: Aug 3, 2023
 *      Author: Angus McLennan
 */

#include "Motor_Control.h"

/*
 * convert_wheel_speeds()
 * float left_speed: Ticks per second from right wheel encoder
 * float right_speed: Ticks per second of left wheel encoder
 * ekf_t* ekf: Kalman Filter struct containing state values
 * returns: void
 */
void convert_wheel_speeds(ekf_t *ekf, float left_speed, float right_speed) {
	// Convert to m/s
	float left_speed_m = left_speed * CONVERT_TICK_TO_MPS;
	float right_speed_m = right_speed * CONVERT_TICK_TO_MPS;

	// Compute the linear and angular velocity
	ekf->lin_vel = (left_speed_m + right_speed_m) / 2.0;
	ekf->ang_vel = (right_speed_m - left_speed_m) / ROBOT_WHEEL_SPACING;
}

/*
 * ekf_predict()
 * float left_speed: Ticks per second from right wheel encoder
 * float right_speed: Ticks per second of left wheel encoder
 * ekf_t* ekf: Kalman Filter struct containing state values
 * float dt: Time since last update in seconds
 * returns: void
 */
void ekf_predict(ekf_t *ekf, float left_speed, float right_speed, float dt) {
	/* Generate state transition matrix */
	float F_matrix[9]; // State transition matrix (3x3)
	ekf_calcualte_state_transition_matrix(ekf, left_speed, right_speed, dt, F_matrix);

	/* Perform timestep increment of state */
	// Convert wheel speeds ang and lin velocities to update ekf struct
	convert_wheel_speeds(ekf, left_speed, right_speed);

	// Update state with given estimate of current ang and lin velocities
	if (ekf->ang_vel < 0.001) {
		ekf->state_vector[0] += cos(ekf->state_vector[2]) * ekf->lin_vel * dt;
		ekf->state_vector[1] += sin(ekf->state_vector[2]) * ekf->lin_vel * dt;
	}

	else {
		float th = ekf->state_vector[2];
		ekf->state_vector[0] += ekf->lin_vel / ekf->ang_vel * (sin(th + dt * ekf->ang_vel) - sin(th));
		ekf->state_vector[1] += -ekf->lin_vel / ekf->ang_vel * (cos(th + dt * ekf->ang_vel) - cos(th));
		ekf->state_vector[2] += dt * ekf->ang_vel;
	}
	// Constrain theta to be between -pi and pi
	if (ekf->state_vector[2] > M_PI) {
		ekf->state_vector[2] -= 2 * M_PI;
	}
	else if (ekf->state_vector[2] < -M_PI) {
		ekf->state_vector[2] += 2 * M_PI;
	}

	/* Update Estimate Covariance Matrix (P) */
	float Q_matrix[9];
	ekf_calculate_estimate_covariance(ekf, left_speed, right_speed, dt, Q_matrix);
	float temp_buff[9]; // (3x3)
	// temp (3x3) = F(3x3) * P(3x3)
	matrix_multiply(F_matrix, ekf->P_matrix, 3, 3, 3, 3, temp_buff);

	float F_transpose[9]; // (3x3)
	matrix_transpose(F_matrix, 3, 3, F_transpose);

	float temp_buff2[9]; // (3x3)
	// temp2 (3x3) = temp(3x3) * F_transpose
	matrix_multiply(temp_buff, F_transpose, 3, 3, 3, 3, temp_buff2);

	// P (3x3) = temp2 + Q
	matrix_add(temp_buff2, Q_matrix, 3, 3, ekf->P_matrix);
}

/*
 * ekf_update()
 * ekf_t* ekf: Kalman Filter struct containing state values
 * float measurement_vector: Vector of measurement distances from ultrasonic sensors [forward, right, back, left]
 * returns: void
 */
void ekf_update(ekf_t *ekf, float measurement_vector[4]) {

}

/*
 * ekf_calculate_estimate_covariance()
 * float left_speed: Ticks per second from right wheel encoder
 * float right_speed: Ticks per second of left wheel encoder
 * ekf_t* ekf: Kalman Filter struct containing state values
 * float dt: Time since last update in seconds
 * float* state_transition_matrix (output): State transition matrix MUST BE FLOAT (3X3)
 * returns: void
 */
void ekf_calcualte_state_transition_matrix(ekf_t *ekf, float left_speed, float right_speed, float dt, float *state_transition_matrix) {
	float th = ekf->state_vector[2];
	state_transition_matrix[0] = 1.0;
	state_transition_matrix[1] = 0.0;
	state_transition_matrix[2] = 0.0;
	state_transition_matrix[4] = 1.0;
	state_transition_matrix[6] = 0.0;
	state_transition_matrix[7] = 0.0;
	state_transition_matrix[8] = 1.0;
	if (ekf->ang_vel < 0.001) {
		state_transition_matrix[3] = sin(th) * ekf->lin_vel * dt;
		state_transition_matrix[5] = cos(th) * ekf->lin_vel * dt;
	}
	else {
		state_transition_matrix[3] = ekf->lin_vel / ekf->ang_vel * (cos(th + dt * ekf->ang_vel) - cos(th));
		state_transition_matrix[5] = ekf->lin_vel / ekf->ang_vel * (sin(th + dt * ekf->ang_vel) - sin(th));
	}
}

/*
 * ekf_calculate_estimate_covariance()
 * float left_speed: Ticks per second from right wheel encoder
 * float right_speed: Ticks per second of left wheel encoder
 * ekf_t* ekf: Kalman Filter struct containing state values
 * float dt: Time since last update in seconds
 * float* covar (output): Estimate covariance matrix MUST BE FLOAT (3X3)
 * returns: void
 */
void ekf_calculate_estimate_covariance(ekf_t *ekf, float left_speed, float right_speed, float dt, float *covar) {
	// Calcualte derivative of lin_vel, ang_vel w.r.t. left_speed, right_speed
	// Jacobian1 (2x2)
	float Jacobian1[] = { CONVERT_TICK_TO_MPS / 2.0, CONVERT_TICK_TO_MPS / 2.0, -CONVERT_TICK_TO_MPS / ROBOT_WHEEL_SPACING, CONVERT_TICK_TO_MPS / ROBOT_WHEEL_SPACING };
	float Jacobian2[6] = { 0.0 };
	float th1 = ekf->state_vector[2];	// Current theta
	float th2 = th1 + dt * ekf->ang_vel;	// Theta at next time step

	// Jacobian2 (3x2) -> derivative of (x,y,theta) w.r.t. lin_vel and ang_vel
	if (ekf->ang_vel < 0.001) {
		Jacobian2[0] = dt * cos(th1);
		Jacobian2[2] = dt * sin(th1);
	}
	else {
		Jacobian2[0] = (sin(th2) - sin(th1)) / ekf->ang_vel;
		Jacobian2[1] = -ekf->lin_vel / (ekf->ang_vel * ekf->ang_vel) * (sin(th2) - sin(th1)) + ekf->lin_vel / ekf->ang_vel * (dt * cos(th2));
		Jacobian2[2] = (cos(th2) - cos(th1)) / ekf->ang_vel;
		Jacobian2[3] = ekf->lin_vel / (ekf->ang_vel * ekf->ang_vel) * (cos(th2) - cos(th1)) + -ekf->lin_vel / ekf->ang_vel * (dt * sin(th2));
		Jacobian2[4] = dt;
	}
	// Calculate final jacobian by multiplying Jacobian1 by Jacobian2
	float jacobian[6];
	uint32_t j_rows = 3;
	uint32_t j_cols = 2;
	matrix_multiply(Jacobian2, Jacobian1, j_rows, j_cols, 2, 2, jacobian);

	// Extract diagonal elements from jacobian
	float diag[4] = { ekf->left_wheel_covariance, 0, 0, ekf->right_wheel_covariance };
	float temp_buff[6]; // (3x2)
	// temp(3x2) = jacob(3x2) * diag(2x2)
	matrix_multiply(jacobian, diag, 3, 2, 2, 2, temp_buff);
	float jacob_T[6]; // Jacobian matrix transpose (2x3)
	matrix_transpose(jacobian, j_rows, j_cols, jacob_T);
	// covar(3x3) = temp(3x2) * jaboc_transpose(2x3)
	matrix_multiply(temp_buff, jacob_T, 3, 2, 2, 2, covar);
}

/*
 * ekf_calculate_z_hat()
 * ekf_t* ekf: Kalman Filter struct containing state values
 * float z_hat[4] (output): The values that a sensor "should" read if it were in the current state
 * defined in the ekf_t struct.
 * returns: void
 */
void ekf_calculate_z_hat(ekf_t *ekf, float z_hat[4]) {
//	float x = ekf->state_vector[0];
//	float y = ekf->state_vector[1];
//	float th = ekf->state_vector[2];
//	// Calculate which face each of the four sensors intersect
//	// Calculate angles to each vertex of the arena from current robot position
//	float angle_top_left = atan2(ARENA_HEIGHT / 2.0 - y, -ARENA_WIDTH / 2.0 - x);
//	float angle_top_right = atan2(ARENA_HEIGHT / 2.0 - y, ARENA_WIDTH / 2.0 - x);
//	float angle_bottom_left = atan2(-ARENA_HEIGHT / 2.0 - y, -ARENA_WIDTH / 2.0 - x);
//	float angle_bottom_right = atan2(-ARENA_HEIGHT / 2.0 - y, ARENA_WIDTH / 2.0 - x);
//
//	if (th < angle_top_left && th >= angle_top_right) {
//		// Robot is pointing at front face
//		// Calculate intercept points
////
////		// z0
////		float m = tan(th);
////		float y_point = ARENA_HEIGHT / 2.0;
////		float x_point = m * (y_point / 2.0 - y) + x;
////		z_hat[0] = sqrt(pow(x - x_point, 2) + pow(y - y_point));
////
////		// z1
////		m = tan(th + M_PI / 2);
////		x_point = -ARENA_WIDTH / 2.0;
////		y_point = y + m * (x_point - x);
////		z_hat[1] = sqrt(pow(x - x_point, 2) + pow(y - y_point));
////
////		// z2
////		m = tan(th + M_PI);
////		y_point = -ARENA_HEIGHT / 2.0;
////		x_point = m * (y_point / 2.0 - y) + x;
////		z_hat[2] = sqrt(pow(x - x_point, 2) + pow(y - y_point));
////
////		// z3
////		m = tan(th + 3 * M_PI / 2);
////		x_point = ARENA_WIDTH / 2.0;
////		y_point = y + m * (x_point - x);
////		z_hat[1] = sqrt(pow(x - x_point, 2) + pow(y - y_point));
//	}
//	else if (th < angle_bottom_left && th >= angle_top_left) {
//		// Robot is pointing at left face
//
//	}
//	else if (th < angle_bottom_right && th >= angle_bottom_left) {
//		// Robot is pointing at back face
//	}
//	else {
//		// Robot is pointing at right face

}

/*
 * get_wheel_velocity()
 * ekf_t* ekf: Kalman Filter struct containing state values
 * This functions updates ekf fields current_wheel_vel[] and prev_wheel_vel[]
 * returns: void
 */
void get_wheel_velocity(ekf_t *ekf) {
	// Read left wheel encoder position

}

void matrix_multiply(const float *A, const float *B, uint32_t A_rows, uint32_t A_cols, uint32_t B_rows, uint32_t B_cols, float *output) {
	// Check if input dimensions are correct
	if (A_cols != B_rows) {
//        printf("Error: Matrix dimensions are not compatible for multiplication!\n");
		return;
	}

	// Perform matrix multiplication
	for (uint32_t i = 0; i < A_rows; i++) {
		for (uint32_t j = 0; j < B_cols; j++) {
			float sum = 0;
			for (uint32_t k = 0; k < A_cols; k++) {
				sum += A[i * A_cols + k] * B[k * B_cols + j];
			}
			output[i * B_cols + j] = sum;
		}
	}
}

void matrix_transpose(const float *A, uint32_t rows, uint32_t cols, float *output) {
	for (uint32_t i = 0; i < rows; i++) {
		for (uint32_t j = 0; j < cols; j++) {
			output[j * rows + i] = A[i * cols + j];
		}
	}
}

void matrix_add(const float *A, const float *B, uint32_t rows, uint32_t cols, float *output) {
	for (uint32_t i = 0; i < rows; i++) {
		for (uint32_t j = 0; j < cols; j++) {
			output[i * cols + j] = A[i * cols + j] + B[i * cols + j];
		}
	}
}



