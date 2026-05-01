/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define CONTR1_Pin GPIO_PIN_6
#define CONTR1_GPIO_Port GPIOA
#define CONTR2_Pin GPIO_PIN_7
#define CONTR2_GPIO_Port GPIOA
#define CONTR3_Pin GPIO_PIN_0
#define CONTR3_GPIO_Port GPIOB
#define CONTR4_Pin GPIO_PIN_1
#define CONTR4_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_9
#define MOTOR1_GPIO_Port GPIOE
#define MOTOR2_Pin GPIO_PIN_11
#define MOTOR2_GPIO_Port GPIOE
#define CONTR5_Pin GPIO_PIN_12
#define CONTR5_GPIO_Port GPIOD
#define CONTR6_Pin GPIO_PIN_13
#define CONTR6_GPIO_Port GPIOD
#define CONTR7_Pin GPIO_PIN_14
#define CONTR7_GPIO_Port GPIOD
#define MOTORL_Pin GPIO_PIN_6
#define MOTORL_GPIO_Port GPIOC
#define MOTORR_Pin GPIO_PIN_7
#define MOTORR_GPIO_Port GPIOC
#define lidar_Pin GPIO_PIN_8
#define lidar_GPIO_Port GPIOC
#define Ariel_TOF_Pin GPIO_PIN_9
#define Ariel_TOF_GPIO_Port GPIOC
#define TOF3_Pin GPIO_PIN_8
#define TOF3_GPIO_Port GPIOA
#define TOF2_Pin GPIO_PIN_11
#define TOF2_GPIO_Port GPIOA
#define TOF1_Pin GPIO_PIN_12
#define TOF1_GPIO_Port GPIOA
#define MOTOR1_FG_Pin GPIO_PIN_15
#define MOTOR1_FG_GPIO_Port GPIOA
#define But_2_Pin GPIO_PIN_1
#define But_2_GPIO_Port GPIOD
#define MOTOR2_FG_Pin GPIO_PIN_3
#define MOTOR2_FG_GPIO_Port GPIOB
#define MOTOR1_DIR_Pin GPIO_PIN_0
#define MOTOR1_DIR_GPIO_Port GPIOE
#define MOTOR2_DIR_Pin GPIO_PIN_1
#define MOTOR2_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
