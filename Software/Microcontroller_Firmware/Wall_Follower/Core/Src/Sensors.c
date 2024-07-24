/*
 * Sensors.c
 *
 *  Created on: Aug 4, 2023
 *      Author: Angus McLennan
 */

#include "Sensors.h"

void trigger_sensor(ultra_sonic_t* sensor) {
	HAL_GPIO_WritePin(sensor->sensor_trig_port, sensor->sensor_trig_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sensor->sensor_trig_port, sensor->sensor_trig_pin, GPIO_PIN_RESET);
}

