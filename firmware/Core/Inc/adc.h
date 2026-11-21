/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc5;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC5_Init(void);

/* USER CODE BEGIN Prototypes */

/* 调试用：把三路 ADC 在「差分」与「单端」之间切换。
 * on=1 时读到的是引脚绝对电压（0..4095 对应 0..VREF），用于判断
 * 模拟前端到底有没有把信号送上来、以及两条腿各自的直流电平。 */
void ADC_ReconfigSingleEnded(uint8_t on);

/* [INTER2] 双 ADC 硬件交替（ADC1 master + ADC2 slave, ADC_DUALMODE_INTERL）
 * on=1 进入交替模式；on=0 恢复独立模式（其余采集路径依赖独立模式） */
int  ADC_DualInterleavedConfig(uint8_t on);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

