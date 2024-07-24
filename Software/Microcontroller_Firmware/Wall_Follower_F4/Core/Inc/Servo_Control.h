/*
 * Servo_Control.h
 *
 *  Created on: 18 Apr. 2023
 *      Author: Angus McLennan
 */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Servo_Timer must be defined by user
extern TIM_HandleTypeDef *Servo_Timer;

typedef enum {
	SERVO_1=1, SERVO_2=2, SERVO_3=3, SERVO_4=4
} servo_id_t;

typedef struct {
	uint32_t TIM_CLK_FREQ; 		// Timer clock reference frequency in Hz
	float SERVO_PWM_FREQ;		// Servo PWM frequency in Hz (from datasheet)
	float min_pulse_ms;			// Min period in MILISECONDS of servos (from datasheet)
	float max_pulse_ms;			// Max period in MILISECONDS of servos (from datasheet)
	uint16_t min_period;
	uint16_t max_period;
	uint8_t servo_range;		// Angle in degrees
}servo_params;

extern servo_params servo_config;

void servo_init(TIM_HandleTypeDef *htim);
void set_servo_position(uint8_t servo_num, TIM_HandleTypeDef *htim, uint16_t position);
uint16_t mapServoAngleToValue(float angle);
bool contrain_servo_angle(float* servo_angle, float servo_max_angle);

#endif /* INC_SERVO_CONTROL_H_ */


