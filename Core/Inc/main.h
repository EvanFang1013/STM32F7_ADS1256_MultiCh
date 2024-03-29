/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETH_SCLK_Pin GPIO_PIN_2
#define ETH_SCLK_GPIO_Port GPIOE
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOE
#define ETH_MISO_Pin GPIO_PIN_5
#define ETH_MISO_GPIO_Port GPIOE
#define ETH_MOSI_Pin GPIO_PIN_6
#define ETH_MOSI_GPIO_Port GPIOE
#define ADC_DRDY_Pin GPIO_PIN_3
#define ADC_DRDY_GPIO_Port GPIOA
#define ADC_DRDY_EXTI_IRQn EXTI3_IRQn
#define ADC_CS_Pin GPIO_PIN_4
#define ADC_CS_GPIO_Port GPIOA
#define ADC_SCLK_Pin GPIO_PIN_5
#define ADC_SCLK_GPIO_Port GPIOA
#define ADC_MISO_Pin GPIO_PIN_6
#define ADC_MISO_GPIO_Port GPIOA
#define ADC_MOSI_Pin GPIO_PIN_7
#define ADC_MOSI_GPIO_Port GPIOA
#define ETH_DRDY_Pin GPIO_PIN_1
#define ETH_DRDY_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
