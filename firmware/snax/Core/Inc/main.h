/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define IR_SENSOR_Pin GPIO_PIN_0
#define IR_SENSOR_GPIO_Port GPIOF
#define DEBUG_LED_Pin GPIO_PIN_0
#define DEBUG_LED_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_2
#define M3_DIR_GPIO_Port GPIOA
#define THRW_PWM_Pin GPIO_PIN_3
#define THRW_PWM_GPIO_Port GPIOA
#define SERVO2_PWM_Pin GPIO_PIN_6
#define SERVO2_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_7
#define M3_PWM_GPIO_Port GPIOA
#define M2_DIR_Pin GPIO_PIN_0
#define M2_DIR_GPIO_Port GPIOB
#define M2_PWM_Pin GPIO_PIN_8
#define M2_PWM_GPIO_Port GPIOA
#define M1_PWM_Pin GPIO_PIN_9
#define M1_PWM_GPIO_Port GPIOA
#define M1_DIR_Pin GPIO_PIN_10
#define M1_DIR_GPIO_Port GPIOA
#define ENC1_M1_Pin GPIO_PIN_15
#define ENC1_M1_GPIO_Port GPIOA
#define ENC2_M1_Pin GPIO_PIN_3
#define ENC2_M1_GPIO_Port GPIOB
#define ENC1_M2_Pin GPIO_PIN_4
#define ENC1_M2_GPIO_Port GPIOB
#define ENC2_M2_Pin GPIO_PIN_5
#define ENC2_M2_GPIO_Port GPIOB
#define ENC1_M3_Pin GPIO_PIN_6
#define ENC1_M3_GPIO_Port GPIOB
#define ENC2_M3_Pin GPIO_PIN_7
#define ENC2_M3_GPIO_Port GPIOB
#define SERVO1_PWM_Pin GPIO_PIN_8
#define SERVO1_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
