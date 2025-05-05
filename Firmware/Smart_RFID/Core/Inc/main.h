/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern SPI_HandleTypeDef hspi2;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define SIM_RST_Pin GPIO_PIN_11
#define SIM_RST_GPIO_Port GPIOE
#define RC522_Rst_Pin GPIO_PIN_10
#define RC522_Rst_GPIO_Port GPIOB
#define RC522_CS_Pin GPIO_PIN_11
#define RC522_CS_GPIO_Port GPIOB
#define Key1_Pin GPIO_PIN_12
#define Key1_GPIO_Port GPIOD
#define Key2_Pin GPIO_PIN_13
#define Key2_GPIO_Port GPIOD
#define KpadRow1_Pin GPIO_PIN_14
#define KpadRow1_GPIO_Port GPIOD
#define KpadRow2_Pin GPIO_PIN_15
#define KpadRow2_GPIO_Port GPIOD
#define KpadRow3_Pin GPIO_PIN_2
#define KpadRow3_GPIO_Port GPIOG
#define KpadRow4_Pin GPIO_PIN_3
#define KpadRow4_GPIO_Port GPIOG
#define KpadCol1_Pin GPIO_PIN_4
#define KpadCol1_GPIO_Port GPIOG
#define KpadCol2_Pin GPIO_PIN_5
#define KpadCol2_GPIO_Port GPIOG
#define KpadCol3_Pin GPIO_PIN_6
#define KpadCol3_GPIO_Port GPIOG
#define KpadCol4_Pin GPIO_PIN_7
#define KpadCol4_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOC
#define DipSw5_Pin GPIO_PIN_7
#define DipSw5_GPIO_Port GPIOC
#define DipSw4_Pin GPIO_PIN_8
#define DipSw4_GPIO_Port GPIOC
#define DipSw3_Pin GPIO_PIN_9
#define DipSw3_GPIO_Port GPIOC
#define DipSw2_Pin GPIO_PIN_8
#define DipSw2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
