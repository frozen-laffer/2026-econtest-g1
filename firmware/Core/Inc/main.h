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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RELAY1_AC_DC_Pin GPIO_PIN_13
#define RELAY1_AC_DC_GPIO_Port GPIOC
#define ENC2_SW_Pin GPIO_PIN_2
#define ENC2_SW_GPIO_Port GPIOC
#define PGA1_EN_Pin GPIO_PIN_3
#define PGA1_EN_GPIO_Port GPIOC
#define RELAY2_ATT_Pin GPIO_PIN_2
#define RELAY2_ATT_GPIO_Port GPIOA
#define RELAY2_AC_DC_Pin GPIO_PIN_3
#define RELAY2_AC_DC_GPIO_Port GPIOA
#define PGA2_EN_Pin GPIO_PIN_4
#define PGA2_EN_GPIO_Port GPIOA
#define RELAY3_ATT_Pin GPIO_PIN_5
#define RELAY3_ATT_GPIO_Port GPIOA
#define RELAY3_AC_DC_Pin GPIO_PIN_0
#define RELAY3_AC_DC_GPIO_Port GPIOB
#define PGA3_EN_Pin GPIO_PIN_1
#define PGA3_EN_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOB
#define LCD_LED_Pin GPIO_PIN_11
#define LCD_LED_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define T_RST_Pin GPIO_PIN_7
#define T_RST_GPIO_Port GPIOC
#define ENC1_SW_Pin GPIO_PIN_10
#define ENC1_SW_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define W5500_RST_Pin GPIO_PIN_2
#define W5500_RST_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB
#define W5500_INT_Pin GPIO_PIN_7
#define W5500_INT_GPIO_Port GPIOB
#define RELAY1_ATT_Pin GPIO_PIN_9
#define RELAY1_ATT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
