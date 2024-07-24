/*
 * physical_params.h
 *
 *  Created on: Aug 3, 2023
 *      Author: thean
 */

#ifndef INC_PHYSICAL_PARAMS_H_
#define INC_PHYSICAL_PARAMS_H_
// All units are SI (kg, m, s)
// Robot Parameters
#define WHEEL_RADIUS	 			0.025
#define ROBOT_WHEEL_SPACING			0.100				// Distance between the robot's left and right wheels
#define CONVERT_TICK_TO_MPS			0.05				// Scaling factor to convert encoder ticks/s to m/s

// Arena Parameters
#define ARENA_WIDTH					2.0
#define ARENA_HEIGHT				2.0

#endif /* INC_PHYSICAL_PARAMS_H_ */
