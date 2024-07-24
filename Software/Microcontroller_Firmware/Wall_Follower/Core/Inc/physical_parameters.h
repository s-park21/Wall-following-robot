/*
 * physical_parameters.h
 *
 *  Created on: Aug 9, 2023
 *      Author: thean
 */

#ifndef INC_PHYSICAL_PARAMETERS_H_
#define INC_PHYSICAL_PARAMETERS_H_
#include <math.h>

#define WHEEL_RADIUS_M					50 / 1000
#define WHEEL_CIRCUMFERENCE_M			2*M_PI*WHEEL_RADIUS_M
#define BASELINE_DISTANCE_M				150 / 1000								// Distance between left and right wheels in mm
#define STEPS_PER_REV					200
#define SPEED_OF_SOUND					340										// m/s

#endif /* INC_PHYSICAL_PARAMETERS_H_ */
