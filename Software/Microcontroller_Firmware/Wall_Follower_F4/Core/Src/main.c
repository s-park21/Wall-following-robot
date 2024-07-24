/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "physical_parameters.h"
#include "Motors.h"
#include "Control.h"
#include "Sensors.h"
#include "debug.h"
#include "Servo_Control.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
ultra_sonic_t front_sensor = { .sensor_echo_pin = Sensor_A_Echo_Pin, .sensor_echo_port = Sensor_A_Echo_GPIO_Port, .sensor_trig_pin = Sensor_A_Trig_Pin, .sensor_trig_port = Sensor_A_Trig_GPIO_Port };
ultra_sonic_t top_left_sensor = { .sensor_echo_pin = Sensor_B_Echo_Pin, .sensor_echo_port = Sensor_B_Echo_GPIO_Port, .sensor_trig_pin = Sensor_B_Trig_Pin, .sensor_trig_port = Sensor_B_Trig_GPIO_Port };
ultra_sonic_t bottom_left_sensor = { .sensor_echo_pin = Sensor_C_Echo_Pin, .sensor_echo_port = Sensor_C_Echo_GPIO_Port, .sensor_trig_pin = Sensor_B_Trig_Pin, .sensor_trig_port = Sensor_C_Trig_GPIO_Port };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef *Right_Motor_Timer = &htim2;
TIM_HandleTypeDef *Left_Motor_Timer = &htim4;
TIM_HandleTypeDef *Servo_Timer = &htim5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t micros() {
	return DWT->CYCCNT / 72;
}

uint32_t millis() {
	return DWT->CYCCNT / 72000;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t time_diff = 0;
	switch (GPIO_Pin) {
	case Sensor_A_Echo_Pin:
		if (front_sensor.start_time == 0) {
			front_sensor.start_time = micros();		// Time in uS
		} else {
			time_diff = micros() - front_sensor.start_time;
			float distance_read = time_diff / 1E6 * SPEED_OF_SOUND / 2;
			if (distance_read <= MAX_SENSOR_READ) {
				front_sensor.distance_read = distance_read * DIST_FILTER_ALPHA + front_sensor.distance_read * (1 - DIST_FILTER_ALPHA);
			}
			front_sensor.start_time = 0;
		}
		break;
	case Sensor_B_Echo_Pin:
		if (top_left_sensor.start_time == 0) {
			top_left_sensor.start_time = micros();		// Time in uS
		} else {
			time_diff = micros() - top_left_sensor.start_time;
			float distance_read = time_diff / 1E6 * SPEED_OF_SOUND / 2;
			if (distance_read <= MAX_SENSOR_READ) {
				top_left_sensor.distance_read = distance_read * DIST_FILTER_ALPHA + top_left_sensor.distance_read * (1 - DIST_FILTER_ALPHA);
			}
			top_left_sensor.start_time = 0;
		}
		break;
	case Sensor_C_Echo_Pin:
		if (bottom_left_sensor.start_time == 0) {
			bottom_left_sensor.start_time = micros();		// Time in uS
		} else {
			time_diff = micros() - bottom_left_sensor.start_time;
			float distance_read = time_diff / 1E6 * SPEED_OF_SOUND / 2;
			if (distance_read <= MAX_SENSOR_READ) {
				bottom_left_sensor.distance_read = distance_read * DIST_FILTER_ALPHA + bottom_left_sensor.distance_read * (1 - DIST_FILTER_ALPHA);
			}
			bottom_left_sensor.start_time = 0;
		}
		break;
		break;
	case Button1_Pin:
		break;
	case Button2_Pin:
		break;
	case Button3_Pin:
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim = &htim9) {
		trigger_sensor(&front_sensor);
	}
}

uint32_t DWT_Delay_Init(void) {
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	/* Enable  clock cycle counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;

	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	/* Check if clock cycle counter has started */
	if (DWT->CYCCNT) {
		return 0; /*clock cycle counter started*/
	} else {
		return 1; /*clock cycle counter not started*/
	}
}

motor_t left_motor;
motor_t right_motor;
robot_pos_t robot_pos;
//servo_params servo_config = { .SERVO_PWM_FREQ = 60, .TIM_CLK_FREQ = 72E6, .max_pulse_ms = 2.4, .min_pulse_ms = 0.6, .servo_range = 180 }; // Config for HiTech HS-55 Servo
servo_params servo_config = { .SERVO_PWM_FREQ = 60, .TIM_CLK_FREQ = 72E6, .max_pulse_ms = 2, .min_pulse_ms = 1, .servo_range = 180 };	// Config for esc

uint32_t prev_time_ms = 0;

enum debug_level dbg_level;
enum debug_level dbg = DBG;

// Target position to reach
float target_vector[3][2] = { { 0.9, 0.8 }, { 0.3, 0.8 } };
uint8_t waypoint_idx = 0;
float target_dist_threshold = 0.1;

float drive_speed = 0.05; // 0.05

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	left_motor.motor_timer = Left_Motor_Timer;
	left_motor.dir_pin_port = Left_Motor_Dir_GPIO_Port;
	left_motor.dir_pin = Left_Motor_Dir_Pin;
	left_motor.motor_enable_port = Motor_Enable_GPIO_Port;
	left_motor.motor_enable_pin = Motor_Enable_Pin;
	left_motor.rate_rps = 0;

	right_motor.motor_timer = Right_Motor_Timer;
	right_motor.dir_pin_port = Right_Motor_Dir_GPIO_Port;
	right_motor.dir_pin = Right_Motor_Dir_Pin;
	right_motor.motor_enable_port = Motor_Enable_GPIO_Port;
	right_motor.motor_enable_pin = Motor_Enable_Pin;
	right_motor.rate_rps = 0;

	robot_pos.left_motor = &left_motor;
	robot_pos.right_motor = &right_motor;
	// Inital position
	robot_pos.x = 0.30;
	robot_pos.y = 0.20;
	// Initial angle
	robot_pos.theta = M_PI / 4;
	robot_pos.drive_speed = drive_speed;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM4_Init();
	MX_USB_DEVICE_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */
	// Disable motors
	HAL_GPIO_WritePin(left_motor.motor_enable_port, left_motor.motor_enable_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(right_motor.motor_enable_port, right_motor.motor_enable_pin, GPIO_PIN_RESET);

	// Initialise DWT timer
	uint32_t res = DWT_Delay_Init();

	servo_init(Servo_Timer);
	HAL_Delay(3000);
	HAL_TIM_Base_Start_IT(&htim9);

//	HAL_Delay(1000);
//	uint16_t write = mapServoAngleToValue(0);
//	set_servo_position(SERVO_1, Servo_Timer, write);
//	HAL_Delay(5000);
//
//	for (float angle = -90; angle < 90; angle++) {
//		write = mapServoAngleToValue(angle);
//		set_servo_position(SERVO_1, Servo_Timer, write);
//		HAL_Delay(100);
//	}
//	turn_angle(2*M_PI, drive_speed, &left_motor, &right_motor);
//	drive_distance(0.45, drive_speed, &left_motor, &right_motor);

//	float set_of_vectors[1000][2];
//	turn_scan(M_PI, drive_speed, &left_motor, &right_motor, set_of_vectors, &front_sensor);
//	for(int i=0;i<1000;i++) {
//		char print_stat[64];
//		size_t sz = snprintf(print_stat, sizeof(print_stat), "%f,%f\r\n", set_of_vectors[i][0],set_of_vectors[i][1]);
//		debug_print(print_stat, sz, dbg);
//	}
//	while (1) {
//		char print_stat[64];
//		size_t sz = snprintf(print_stat, sizeof(print_stat), "%f\r\n", front_sensor.distance_read);
//		debug_print(print_stat, sz, dbg);
//		HAL_Delay(100);
//	}

	float angle_turned;
	float dist_driven;
	float turn_angle_val;
	float dist_to_target;
	float angle_to_target;
	/* Drive to first target */
	float target_x = target_vector[0][0];
	float target_y = target_vector[0][1];
	dist_to_target = sqrt(pow(target_x - robot_pos.x, 2) + pow(target_y - robot_pos.y, 2));
	angle_to_target = atan2(target_y - robot_pos.y, target_x - robot_pos.x);
	turn_angle_val = angle_to_target - robot_pos.theta;
	if (turn_angle_val > M_PI)
		turn_angle_val -= 2 * M_PI;
	else if (turn_angle_val < -M_PI)
		turn_angle_val += 2 * M_PI;
	if (fabs(turn_angle_val) > 0.01) {
		angle_turned = turn_angle(turn_angle_val, drive_speed, &left_motor, &right_motor);
		robot_pos.theta += angle_turned;
	}
	float ramp_dist = drive_forward_smooth(drive_speed, &left_motor, &right_motor);
	float run_time = (dist_to_target - ramp_dist) / drive_speed;
	uint16_t start_time = millis();
	while ((millis() - start_time) < run_time * 1000) {
		char print_string[64];
		size_t sz = snprintf(print_string, sizeof(print_string), "%d, %f\r\n", millis() - start_time, run_time * 1000);
		debug_print(print_string, sz, dbg);
		HAL_Delay(50);
	}
	lock_motors(&left_motor, &right_motor);
	HAL_Delay(50);
	disable_motors(&right_motor);
	robot_pos.x += dist_to_target * cos(robot_pos.theta);
	robot_pos.y += dist_to_target * sin(robot_pos.theta);

	char print_string[64];
	size_t sz = snprintf(print_string, sizeof(print_string), "robot pose [%f, %f, %f]\r\n", robot_pos.x, robot_pos.y, robot_pos.theta);
	debug_print(print_string, sz, dbg);
	HAL_Delay(10000);

	/* Drive to second target */
	set_servo_position(SERVO_1, Servo_Timer, mapServoAngleToValue(0));
	target_x = target_vector[1][0];
	target_y = target_vector[1][1];
	dist_to_target = sqrt(pow(target_x - robot_pos.x, 2) + pow(target_y - robot_pos.y, 2));
	angle_to_target = atan2(target_y - robot_pos.y, target_x - robot_pos.x);
	turn_angle_val = angle_to_target - robot_pos.theta;
	if (turn_angle_val > M_PI)
		turn_angle_val -= 2 * M_PI;
	else if (turn_angle_val < -M_PI)
		turn_angle_val += 2 * M_PI;
	if (fabs(turn_angle_val) > 0.01) {
		angle_turned = turn_angle(turn_angle_val, drive_speed, &left_motor, &right_motor);
		robot_pos.theta += angle_turned;
	}
	ramp_dist = drive_forward_smooth(drive_speed, &left_motor, &right_motor);
	run_time = (dist_to_target - ramp_dist) / drive_speed;
	start_time = millis();
	while ((millis() - start_time) < run_time * 1000) {
		sz = snprintf(print_string, sizeof(print_string), "%d, %f\r\n", millis() - start_time, run_time * 1000);
		debug_print(print_string, sz, dbg);
		HAL_Delay(50);
		if (front_sensor.distance_read < OBSTACLE_DIST_THRESHOLD) {
			// Toggle indicator LED ON
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			// Stop
			lock_motors(&left_motor, &right_motor);
			HAL_Delay(50);
			disable_motors(&right_motor);
			float obstical_detect_time = millis();
			float distance_driven_to_obj = drive_speed * (obstical_detect_time - start_time) / 1000;
			run_time = 0;
			// Drive around obstacle
			// Reverse drive 0.1m
			float reverse_dist = -0.1;
			dist_driven = drive_distance(reverse_dist, drive_speed, &left_motor, &right_motor);
			robot_pos.x += dist_driven * cos(robot_pos.theta);
			robot_pos.y += dist_driven * sin(robot_pos.theta);

			// Toggle indicator LED ON
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

			// Turn left 90 degrees
			angle_turned = turn_angle(M_PI / 2, drive_speed, &left_motor, &right_motor);
			robot_pos.theta += angle_turned;

			// Drive forward to avoid target
			float avoid_dist = 0.4;
			dist_driven = drive_distance(avoid_dist, drive_speed, &left_motor, &right_motor);
			robot_pos.x += dist_driven * cos(robot_pos.theta);
			robot_pos.y += dist_driven * sin(robot_pos.theta);

			// Turn right 90 degrees
			angle_turned = turn_angle(-M_PI / 2, drive_speed, &left_motor, &right_motor);
			robot_pos.theta += angle_turned;

			// Drive forward to align with target
			float drive_dist = dist_to_target - ramp_dist - distance_driven_to_obj - reverse_dist + 0.05;
			dist_driven = drive_distance(drive_dist, drive_speed, &left_motor, &right_motor);

			// Turn right
			angle_turned = turn_angle(-M_PI / 2, drive_speed, &left_motor, &right_motor);
			robot_pos.theta += angle_turned;

			// Drive to target
			drive_distance(avoid_dist, drive_speed, &left_motor, &right_motor);
		}
	}
	lock_motors(&left_motor, &right_motor);
	HAL_Delay(50);
	disable_motors(&right_motor);

	robot_pos.x += dist_driven * cos(robot_pos.theta);
	robot_pos.y += dist_driven * sin(robot_pos.theta);

	sz = snprintf(print_string, sizeof(print_string), "robot pose [%f, %f, %f]\r\n", robot_pos.x, robot_pos.y, robot_pos.theta);
	debug_print(print_string, sz, dbg);
	for (int i = 0; i < 10; i++) {
		set_servo_position(SERVO_1, Servo_Timer, mapServoAngleToValue(-90));
		HAL_Delay(300);
		set_servo_position(SERVO_1, Servo_Timer, mapServoAngleToValue(90));
		HAL_Delay(300);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//	servo1_pos = mapServoAngleToValue(i++);
//	set_servo_position(SERVO_1, Servo_Timer, servo1_pos);
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//	HAL_Delay(100);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 65535;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 20 - 1;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Right_Motor_Dir_Pin | Left_Motor_Dir_Pin | Motor_Enable_Pin | Sensor_C_Trig_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Sensor_A_Trig_Pin | Sensor_B_Trig_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Button1_Pin Button1A4_Pin Button2_Pin */
	GPIO_InitStruct.Pin = Button1_Pin | Button1A4_Pin | Button2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Button3_Pin */
	GPIO_InitStruct.Pin = Button3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Right_Motor_Dir_Pin Left_Motor_Dir_Pin Motor_Enable_Pin Sensor_C_Trig_Pin */
	GPIO_InitStruct.Pin = Right_Motor_Dir_Pin | Left_Motor_Dir_Pin | Motor_Enable_Pin | Sensor_C_Trig_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Sensor_A_Trig_Pin Sensor_B_Trig_Pin */
	GPIO_InitStruct.Pin = Sensor_A_Trig_Pin | Sensor_B_Trig_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Sensor_A_Echo_Pin */
	GPIO_InitStruct.Pin = Sensor_A_Echo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Sensor_A_Echo_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Sensor_B_Echo_Pin Sensor_C_Echo_Pin */
	GPIO_InitStruct.Pin = Sensor_B_Echo_Pin | Sensor_C_Echo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
