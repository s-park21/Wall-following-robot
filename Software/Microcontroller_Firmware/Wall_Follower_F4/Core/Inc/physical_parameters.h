/*
 * physical_parameters.h
 *
 *  Created on: Aug 9, 2023
 *      Author: thean
 */

#ifndef INC_PHYSICAL_PARAMETERS_H_
#define INC_PHYSICAL_PARAMETERS_H_
#include <math.h>

#define WHEEL_RADIUS_M					(28.0f / 1000.0f)
#define WHEEL_CIRCUMFERENCE_M			(2.0f*M_PI*WHEEL_RADIUS_M)
#define BASELINE_DISTANCE_M				(156.0f / 1000.0f)							// Distance between left and right wheels in mm
#define STEPS_PER_REV					200.0f
#define SPEED_OF_SOUND					340.0f										// m/s

#endif /* INC_PHYSICAL_PARAMETERS_H_ */
