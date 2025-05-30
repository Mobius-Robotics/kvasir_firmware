/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile uint8_t error_flashes;
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
#define ElevatorFinecorsa_Pin GPIO_PIN_2
#define ElevatorFinecorsa_GPIO_Port GPIOB
#define Finecorsa1_Pin GPIO_PIN_12
#define Finecorsa1_GPIO_Port GPIOB
#define Finecorsa2_Pin GPIO_PIN_13
#define Finecorsa2_GPIO_Port GPIOB
#define Emergency_Pin GPIO_PIN_14
#define Emergency_GPIO_Port GPIOB
#define Emergency_EXTI_IRQn EXTI14_IRQn
#define Cordicella_Pin GPIO_PIN_15
#define Cordicella_GPIO_Port GPIOB
#define Cordicella_EXTI_IRQn EXTI15_IRQn
#define ElevatorStep_Pin GPIO_PIN_8
#define ElevatorStep_GPIO_Port GPIOA
#define ElevatorDir_Pin GPIO_PIN_10
#define ElevatorDir_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
