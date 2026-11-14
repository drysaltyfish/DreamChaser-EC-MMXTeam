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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
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
#define MOTORB_ENC_B_Pin GPIO_PIN_5
#define MOTORB_ENC_B_GPIO_Port GPIOA
#define MOTORC_ENC_B_Pin GPIO_PIN_6
#define MOTORC_ENC_B_GPIO_Port GPIOA
#define MOTORC_ENC_A_Pin GPIO_PIN_7
#define MOTORC_ENC_A_GPIO_Port GPIOA
#define MOTORA_ENC_A_Pin GPIO_PIN_9
#define MOTORA_ENC_A_GPIO_Port GPIOE
#define MOTORA_ENC_B_Pin GPIO_PIN_11
#define MOTORA_ENC_B_GPIO_Port GPIOE
#define MOTORD_ENC_B_Pin GPIO_PIN_12
#define MOTORD_ENC_B_GPIO_Port GPIOD
#define MOTORD_ENC_A_Pin GPIO_PIN_13
#define MOTORD_ENC_A_GPIO_Port GPIOD
#define MOTORD_IN2_Pin GPIO_PIN_14
#define MOTORD_IN2_GPIO_Port GPIOD
#define MOTORC_IN2_Pin GPIO_PIN_15
#define MOTORC_IN2_GPIO_Port GPIOD
#define MOTORD_IN1_Pin GPIO_PIN_6
#define MOTORD_IN1_GPIO_Port GPIOC
#define MOTORC_IN1_Pin GPIO_PIN_7
#define MOTORC_IN1_GPIO_Port GPIOC
#define MOTORB_IN1_Pin GPIO_PIN_8
#define MOTORB_IN1_GPIO_Port GPIOC
#define MOTORA_IN1_Pin GPIO_PIN_9
#define MOTORA_IN1_GPIO_Port GPIOC
#define MOTORB_IN2_Pin GPIO_PIN_8
#define MOTORB_IN2_GPIO_Port GPIOA
#define MOTORA_IN2_Pin GPIO_PIN_9
#define MOTORA_IN2_GPIO_Port GPIOA
#define MOTORB_ENC_A_Pin GPIO_PIN_3
#define MOTORB_ENC_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
