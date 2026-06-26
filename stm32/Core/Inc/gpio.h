/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)  //KEY1����
#define KEY2        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)  //KEY2����
#define KEY3        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12) //KEY3����


#define KEY1_PRES 1  	//KEY1���º󷵻�ֵ
#define KEY2_PRES	2	//KEY2���º󷵻�ֵ
#define KEY3_PRES	3	//KEY3���º󷵻�ֵ


void KEY_Init(void);  //����IO��ʼ������
uint8_t  KEY_Scan(void); //����ɨ�躯��
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

