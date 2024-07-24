/*
 * Servo_Control.c
 *
 *  Created on: 18 Apr. 2023
 *      Author: Angus McLennan
 */

#include "Servo_Control.h"

void servo_init(TIM_HandleTypeDef *htim) {
	// Calculate timer prescaler to achieve required frequency
	uint32_t PWM_PSC = servo_config.TIM_CLK_FREQ / (servo_config.SERVO_PWM_FREQ * (htim->Instance->ARR + 1));

	// Recalculate PWM freq given the prescalar calculated (which was rounded)
	servo_config.SERVO_PWM_FREQ = servo_config.TIM_CLK_FREQ / ((htim->Instance->ARR + 1) * (PWM_PSC + 1));

	// Calculate minimum and maximum periods for servo
	float clock_period_ms = 1000 / servo_config.SERVO_PWM_FREQ;
	servo_config.min_period = (servo_config.min_pulse_ms / clock_period_ms) * htim->Instance->ARR;
	servo_config.max_period = (servo_config.max_pulse_ms / clock_period_ms) * htim->Instance->ARR;

	htim->Init.Prescaler = PWM_PSC;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(htim);
	HAL_TIM_PWM_Init(htim);
}

/*
 * Function to set servo to position
 * Inputs:
 * servo_num:	Number between 1 and 4 to represent the servo to be controlled
 * htim:		HAL timer defined from configuration
 * position:	Number between 0-65535 representing position range from 0 to SERVO_RANGE in degrees
 * Returns: void
 */
void set_servo_position(uint8_t servo_num, TIM_HandleTypeDef *htim, uint16_t position) {
	uint32_t timer_channel = 0;
	HAL_StatusTypeDef res;

	switch (servo_num) {
	case 1:
		timer_channel = TIM_CHANNEL_1;  // Set timer_channel to channel 1 of the timer
		float fractional_position = (float) position / 65535.0;
		uint16_t mapped_pos = fractional_position * (float) (servo_config.max_period - servo_config.min_period) + servo_config.min_period;
		// Convert mapped position to miliseconds
		//	float pulse_width_ms = ((float)mapped_pos/(float)htim->Instance->ARR) * 1000/servo_config.SERVO_PWM_FREQ;
		htim->Instance->CCR1 = mapped_pos;
		res = HAL_TIM_PWM_Start(htim, timer_channel);
		break;
	case 2:
		timer_channel = TIM_CHANNEL_2;  // Set timer_channel to channel 2 of the timer
		fractional_position = (float) position / 65535.0;
		mapped_pos = fractional_position * (float) (servo_config.max_period - servo_config.min_period) + servo_config.min_period;
		// Convert mapped position to miliseconds
		//	float pulse_width_ms = ((float)mapped_pos/(float)htim->Instance->ARR) * 1000/servo_config.SERVO_PWM_FREQ;
		htim->Instance->CCR2 = mapped_pos;
		res = HAL_TIM_PWM_Start(htim, timer_channel);
		break;
	case 3:
		timer_channel = TIM_CHANNEL_3;  // Set timer_channel to channel 3 of the timer
		fractional_position = (float) position / 65535.0;
		mapped_pos = fractional_position * (float) (servo_config.max_period - servo_config.min_period) + servo_config.min_period;
		// Convert mapped position to miliseconds
		//	float pulse_width_ms = ((float)mapped_pos/(float)htim->Instance->ARR) * 1000/servo_config.SERVO_PWM_FREQ;
		htim->Instance->CCR3 = mapped_pos;
		res = HAL_TIM_PWM_Start(htim, timer_channel);
		break;
	case 4:
		timer_channel = TIM_CHANNEL_4;  // Set timer_channel to channel 4 of the timer
		fractional_position = (float) position / 65535.0;
		mapped_pos = fractional_position * (float) (servo_config.max_period - servo_config.min_period) + servo_config.min_period;
		// Convert mapped position to miliseconds
		//	float pulse_width_ms = ((float)mapped_pos/(float)htim->Instance->ARR) * 1000/servo_config.SERVO_PWM_FREQ;
		htim->Instance->CCR4 = mapped_pos;
		res = HAL_TIM_PWM_Start(htim, timer_channel);
		break;
	default:
		// Invalid servo number, do nothing
		return;
	}
	// Configure PWM output on the specified timer channel to the desired position
}

uint16_t mapServoAngleToValue(float angle) {
	return (uint16_t) (((angle + servo_config.servo_range / 2) * 65535UL) / servo_config.servo_range);
}


/*
 * Function to constrain servo angle
 * Inputs:
 * servo_angle: Angle that must be constrained
 * servo_max_angle:	The maxmimum angle that the servos can reach
 * Returns: A boolean -> 1 if saturated, 0 if not saturated
 */
bool contrain_servo_angle(float* servo_angle, float servo_max_angle) {
	if (*servo_angle > servo_max_angle) {
		*servo_angle = servo_max_angle;
		return true;
	} else if (*servo_angle < -servo_max_angle) {
		*servo_angle = -servo_max_angle;
		return true;
	} else
		return false;
}
