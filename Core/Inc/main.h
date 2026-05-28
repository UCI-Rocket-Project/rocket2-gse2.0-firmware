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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* cppExported constants --------------------------------------------------------*/
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
#define MVAS_TIMER htim6
#define MVAS_TIMER_1MS_PSC 7999
#define SOLENOID2_EN_Pin GPIO_PIN_13
#define SOLENOID2_EN_GPIO_Port GPIOC
#define SOLENOID1_EN_Pin GPIO_PIN_14
#define SOLENOID1_EN_GPIO_Port GPIOC
#define SOLENOID0_EN_Pin GPIO_PIN_15
#define SOLENOID0_EN_GPIO_Port GPIOC
#define SOLENOID0_FB_Pin GPIO_PIN_0
#define SOLENOID0_FB_GPIO_Port GPIOC
#define SOLENOID1_FB_Pin GPIO_PIN_1
#define SOLENOID1_FB_GPIO_Port GPIOC
#define SOLENOID2_FB_Pin GPIO_PIN_2
#define SOLENOID2_FB_GPIO_Port GPIOC
#define SOLENOID3_FB_Pin GPIO_PIN_3
#define SOLENOID3_FB_GPIO_Port GPIOC
#define SOLENOID4_FB_Pin GPIO_PIN_0
#define SOLENOID4_FB_GPIO_Port GPIOA
#define SOLENOID5_FB_Pin GPIO_PIN_1
#define SOLENOID5_FB_GPIO_Port GPIOA
#define SOLENOID6_FB_Pin GPIO_PIN_2
#define SOLENOID6_FB_GPIO_Port GPIOA
#define SOLENOID7_FB_Pin GPIO_PIN_3
#define SOLENOID7_FB_GPIO_Port GPIOA
#define SOLENOID8_FB_Pin GPIO_PIN_4
#define SOLENOID8_FB_GPIO_Port GPIOA
#define SOLENOID9_FB_Pin GPIO_PIN_5
#define SOLENOID9_FB_GPIO_Port GPIOA
#define SOLENOID10_FB_Pin GPIO_PIN_6
#define SOLENOID10_FB_GPIO_Port GPIOA
#define SOLENOID11_FB_Pin GPIO_PIN_7
#define SOLENOID11_FB_GPIO_Port GPIOA
#define SOLENOID10_EN_Pin GPIO_PIN_4
#define SOLENOID10_EN_GPIO_Port GPIOC
#define SOLENOID11_EN_Pin GPIO_PIN_5
#define SOLENOID11_EN_GPIO_Port GPIOC
#define PWR0_Pin GPIO_PIN_0
#define PWR0_GPIO_Port GPIOB
#define PWR1_Pin GPIO_PIN_1
#define PWR1_GPIO_Port GPIOB
#define ETH_RST_Pin GPIO_PIN_2
#define ETH_RST_GPIO_Port GPIOB
#define ETH_TX_Pin GPIO_PIN_10
#define ETH_TX_GPIO_Port GPIOB
#define ETH_RX_Pin GPIO_PIN_11
#define ETH_RX_GPIO_Port GPIOB
#define ALARM_Pin GPIO_PIN_12
#define ALARM_GPIO_Port GPIOB
#define ETH_RTS_Pin GPIO_PIN_13
#define ETH_RTS_GPIO_Port GPIOB
#define ETH_CTS_Pin GPIO_PIN_14
#define ETH_CTS_GPIO_Port GPIOB
#define ARMED_Pin GPIO_PIN_15
#define ARMED_GPIO_Port GPIOB
#define EMATCH0_FIRE_Pin GPIO_PIN_6
#define EMATCH0_FIRE_GPIO_Port GPIOC
#define EMATCH0_CONT_Pin GPIO_PIN_7
#define EMATCH0_CONT_GPIO_Port GPIOC
#define EMATCH1_FIRE_Pin GPIO_PIN_8
#define EMATCH1_FIRE_GPIO_Port GPIOC
#define EMATCH1_CONT_Pin GPIO_PIN_9
#define EMATCH1_CONT_GPIO_Port GPIOC
#define TC0_CS_Pin GPIO_PIN_8
#define TC0_CS_GPIO_Port GPIOA
#define TC1_CS_Pin GPIO_PIN_10
#define TC1_CS_GPIO_Port GPIOA
#define TC2_CS_Pin GPIO_PIN_15
#define TC2_CS_GPIO_Port GPIOA
#define SOLENOID9_EN_Pin GPIO_PIN_12
#define SOLENOID9_EN_GPIO_Port GPIOC
#define SOLENOID8_EN_Pin GPIO_PIN_2
#define SOLENOID8_EN_GPIO_Port GPIOD
#define SOLENOID7_EN_Pin GPIO_PIN_3
#define SOLENOID7_EN_GPIO_Port GPIOB
#define SOLENOID6_EN_Pin GPIO_PIN_4
#define SOLENOID6_EN_GPIO_Port GPIOB
#define SOLENOID5_EN_Pin GPIO_PIN_5
#define SOLENOID5_EN_GPIO_Port GPIOB
#define EXT_ADC_SCL_Pin GPIO_PIN_6
#define EXT_ADC_SCL_GPIO_Port GPIOB
#define EXT_ADC_SDA_Pin GPIO_PIN_7
#define EXT_ADC_SDA_GPIO_Port GPIOB
#define SOLENOID4_EN_Pin GPIO_PIN_8
#define SOLENOID4_EN_GPIO_Port GPIOB
#define SOLENOID3_EN_Pin GPIO_PIN_9
#define SOLENOID3_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
