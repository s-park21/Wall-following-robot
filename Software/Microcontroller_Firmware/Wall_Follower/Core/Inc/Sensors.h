/*
 * Sensors.h
 *
 *  Created on: Aug 4, 2023
 *      Author: thean
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
#include "stm32f1xx_hal.h"

typedef struct {
	uint16_t sensor_echo_pin;
	GPIO_TypeDef* sensor_echo_port;
	uint16_t sensor_trig_pin;
	GPIO_TypeDef* sensor_trig_port;
	uint32_t start_time;
	float distance_read;
}ultra_sonic_t;

void trigger_sensor(ultra_sonic_t* sensor);

#endif /* INC_SENSORS_H_ */
