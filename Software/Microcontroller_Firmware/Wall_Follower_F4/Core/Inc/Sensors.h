/*
 * Sensors.h
 *
 *  Created on: Aug 4, 2023
 *      Author: thean
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
#include "stm32f4xx_hal.h"

#define MAX_SENSOR_READ 			5.0f
#define DIST_FILTER_ALPHA			0.3f

extern uint32_t millis();
extern uint32_t micros();

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
