/*
 * Control.c
 *
 *  Created on: 9 Aug. 2023
 *      Author: thean
 */

#include "Control.h"

/*
 * Function: wall_follower_update
 * Inputs:
 * wall_follower_t wf: Wall following controller struct
 * float current_dist_mm: Current distance of robot from wall in mm
 * float desired_dist_mm: Desired distance of robot from wall in mm
 * Returns:
 * float: Control signal output - Desired angle (radians) the robot should be from the wall
 */
float wall_follower_update(wall_follower_t wf, float current_dist_mm, float desired_dist_mm) {
	float angle_to_wall = wf.wall_dist_Kp * (current_dist_mm - desired_dist_mm);

	// Clamp the output to within reasonable bounds
	if (angle_to_wall > wf.max_angle_from_wall)
		angle_to_wall = wf.max_angle_from_wall;
	else if (angle_to_wall < -wf.max_angle_from_wall)
		angle_to_wall = -wf.max_angle_from_wall;

	return angle_to_wall;
}

/*
 * Function: robot_angle_update
 * Inputs:
 * wall_follower_t wf: Wall following controller struct
 * float current_angle: Current angle of robot relative to wall in radians
 * float desired_angle: Desired angle fo robot relative to wall in radians
 * float* (Output) left_wheel_drive_multiplier: Speed multiplier for left wheel
 * float* (Output) right_wheel_drive_multiplier: Speed multiplier for right wheel
 * Returns: void
 */
void robot_angle_update(wall_follower_t wf, float current_angle, float desired_angle, float *left_wheel_drive_multiplier, float *right_wheel_drive_multiplier) {
	float wall_angle_error = current_angle - desired_angle;
	wf.wall_angle_accumulated_error += wall_angle_error;
	float controller_output = wf.wall_angle_Kp * wall_angle_error + wf.wall_angle_Ki * wf.wall_angle_accumulated_error;
	*left_wheel_drive_multiplier = pos_tanh(controller_output);
	*right_wheel_drive_multiplier = neg_tanh(controller_output);
}

/*
 * Function: calculate_angle_and_dist_to_wall
 * Inputs:
 * float front_dist_mm: Reading from front distance sensor in mm
 * float back_dist_mm: Reading of back distacne sensor in mm
 * float* (Output) robot_angle: Calculated angle of robot relative to wall in radians
 * float* (Output) dist_to_wall: Calculated distance of robot to wall in mm;
 * Returns: void
 */
void calculate_angle_and_dist_to_wall(float front_dist_mm, float back_dist_mm, float *robot_angle, float *dist_to_wall) {
	*robot_angle = atan2(back_dist_mm - front_dist_mm, BASELINE_DISTANCE_M);
	*dist_to_wall = 0.5 * (front_dist_mm + back_dist_mm) * cos(*robot_angle);
}

/*
 * Function: update_robot_position
 * This function updates the current state of the robot by dead reckoning the angular rate of the motors
 * This function MUST BE CALLED WHENEVER THE MOTOR STATE IS UPDATED
 * Inputs:
 * robot_pos_t* robot_pos: A struct containing the robot state parameters
 * Returns: void
 */
void update_robot_position(robot_pos_t *robot_pos, float dt) {
	float left_motor_rate = robot_pos->left_motor->rate_rps;
	float right_motor_rate = robot_pos->right_motor->rate_rps;
	if (left_motor_rate == right_motor_rate) {
		float linear_velocity = left_motor_rate * WHEEL_RADIUS_M;
		// Robot is driving straight
		// Update x coordinate
		robot_pos->x += linear_velocity * cos(robot_pos->theta) * dt;
		// Update y coordinate
		robot_pos->y += linear_velocity * sin(robot_pos->theta) * dt;
	} else {
		float angular_rate = (right_motor_rate - left_motor_rate) / BASELINE_DISTANCE_M;
		float linear_velocity = (right_motor_rate*WHEEL_RADIUS_M + left_motor_rate*WHEEL_RADIUS_M) / 2;
		float x_dot = linear_velocity * cos(robot_pos->theta);
		float y_dot = linear_velocity * sin(robot_pos->theta);

		robot_pos->x += x_dot * dt;
		robot_pos->y += y_dot * dt;
		robot_pos->theta += angular_rate * dt;
	}
	if(robot_pos->theta > 2*M_PI)
		robot_pos->theta -= 2*M_PI;
}

void steer_to_target(robot_pos_t *robot_pos, float target_angle) {
	const float Kp = 1;
	float angle_error = Kp * (target_angle - robot_pos->theta);

	// Change left motor speed by tanh relationship
	robot_pos->left_motor->rate_rps = robot_pos->drive_speed * pos_tanh(angle_error);

	// Change right motor speed by tanh relationship
	robot_pos->right_motor->rate_rps = robot_pos->drive_speed * neg_tanh(angle_error);
}

float pos_tanh(float x) {
	return 2 / (1 + exp(-2 * x));
}

float neg_tanh(float x) {
	return 2 / (1 + exp(2 * x));
}
