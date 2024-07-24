/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SERVO_Pin GPIO_PIN_0
#define SERVO_GPIO_Port GPIOA
#define Button1_Pin GPIO_PIN_2
#define Button1_GPIO_Port GPIOA
#define Button1_EXTI_IRQn EXTI2_IRQn
#define Button1A4_Pin GPIO_PIN_4
#define Button1A4_GPIO_Port GPIOA
#define Button1A4_EXTI_IRQn EXTI4_IRQn
#define Right_Motor_PWM_Pin GPIO_PIN_5
#define Right_Motor_PWM_GPIO_Port GPIOA
#define Button2_Pin GPIO_PIN_6
#define Button2_GPIO_Port GPIOA
#define Button2_EXTI_IRQn EXTI9_5_IRQn
#define Button3_Pin GPIO_PIN_1
#define Button3_GPIO_Port GPIOB
#define Button3_EXTI_IRQn EXTI1_IRQn
#define Right_Motor_Dir_Pin GPIO_PIN_2
#define Right_Motor_Dir_GPIO_Port GPIOB
#define Left_Motor_Dir_Pin GPIO_PIN_12
#define Left_Motor_Dir_GPIO_Port GPIOB
#define Motor_Enable_Pin GPIO_PIN_13
#define Motor_Enable_GPIO_Port GPIOB
#define Sensor_A_Trig_Pin GPIO_PIN_8
#define Sensor_A_Trig_GPIO_Port GPIOA
#define Sensor_A_Echo_Pin GPIO_PIN_9
#define Sensor_A_Echo_GPIO_Port GPIOA
#define Sensor_A_Echo_EXTI_IRQn EXTI9_5_IRQn
#define Sensor_B_Trig_Pin GPIO_PIN_15
#define Sensor_B_Trig_GPIO_Port GPIOA
#define Sensor_B_Echo_Pin GPIO_PIN_3
#define Sensor_B_Echo_GPIO_Port GPIOB
#define Sensor_B_Echo_EXTI_IRQn EXTI3_IRQn
#define Left_Motor_PWM_Pin GPIO_PIN_6
#define Left_Motor_PWM_GPIO_Port GPIOB
#define Sensor_C_Echo_Pin GPIO_PIN_8
#define Sensor_C_Echo_GPIO_Port GPIOB
#define Sensor_C_Echo_EXTI_IRQn EXTI9_5_IRQn
#define Sensor_C_Trig_Pin GPIO_PIN_9
#define Sensor_C_Trig_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
