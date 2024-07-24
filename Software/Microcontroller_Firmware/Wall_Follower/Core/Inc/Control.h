/*
 * Control.h
 *
 *  Created on: 9 Aug. 2023
 *      Author: thean
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>
#include <math.h>
#include "Motors.h"
#include "physical_parameters.h"

typedef struct {
	float wall_dist_Kp;
	float max_angle_from_wall;
	float wall_angle_Kp;
	float wall_angle_Ki;
	float wall_angle_accumulated_error;
}wall_follower_t;

typedef struct {
	motor_t* left_motor;
	motor_t* right_motor;
	float x;
	float y;
	float theta;
	float drive_speed;
}robot_pos_t;

float wall_follower_update(wall_follower_t wf, float current_dist_mm, float desired_dist_mm);
void robot_angle_update(wall_follower_t wf, float current_angle, float desired_angle, float* left_wheel_drive_multiplier, float* right_wheel_drive_multiplier);
void calculate_angle_and_dist_to_wall(float front_dist_mm, float back_dist_mm, float* robot_angle, float* dist_to_wall);
void update_robot_position(robot_pos_t* robot_pos, float dt);
void steer_to_target(robot_pos_t *robot_pos, float target_angle);
float pos_tanh(float x);
float neg_tanh(float x);

#endif /* INC_CONTROL_H_ */
