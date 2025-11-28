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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint32_t t1Ticks;   // tick when first sensor saw IR
    uint32_t t2Ticks;   // tick when second sensor saw IR
} SliceEvent_t;
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Fruit2_IR1_Pin GPIO_PIN_0
#define Fruit2_IR1_GPIO_Port GPIOC
#define Fruit2_IR2_Pin GPIO_PIN_1
#define Fruit2_IR2_GPIO_Port GPIOC
#define Fruit1_IR2_Pin GPIO_PIN_0
#define Fruit1_IR2_GPIO_Port GPIOA
#define Fruit1_IR1_Pin GPIO_PIN_1
#define Fruit1_IR1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Fruit4_LED_Pin GPIO_PIN_4
#define Fruit4_LED_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define PushButton_Pin GPIO_PIN_0
#define PushButton_GPIO_Port GPIOB
#define Fruit3_IR1_Pin GPIO_PIN_7
#define Fruit3_IR1_GPIO_Port GPIOC
#define Fruit2_LED_Pin GPIO_PIN_8
#define Fruit2_LED_GPIO_Port GPIOA
#define Fruit1_LED_Pin GPIO_PIN_9
#define Fruit1_LED_GPIO_Port GPIOA
#define Fruit3_LED_Pin GPIO_PIN_10
#define Fruit3_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Fruit4_IR1_Pin GPIO_PIN_4
#define Fruit4_IR1_GPIO_Port GPIOB
#define Fruit4_IR2_Pin GPIO_PIN_5
#define Fruit4_IR2_GPIO_Port GPIOB
#define Fruit3_IR2_Pin GPIO_PIN_6
#define Fruit3_IR2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// ---- Fruit 1 (PA9, PA1, PA0) ----
#define FRUIT1_LED_Pin         Fruit1_LED_Pin
#define FRUIT1_LED_GPIO_Port   Fruit1_LED_GPIO_Port

#define FRUIT1_IR1_Pin         Fruit1_IR1_Pin
#define FRUIT1_IR1_GPIO_Port   Fruit1_IR1_GPIO_Port

#define FRUIT1_IR2_Pin         Fruit1_IR2_Pin
#define FRUIT1_IR2_GPIO_Port   Fruit1_IR2_GPIO_Port

// ---- Fruit 2 (PA8, PC0, PC1) ----
#define FRUIT2_LED_Pin         Fruit2_LED_Pin
#define FRUIT2_LED_GPIO_Port   Fruit2_LED_GPIO_Port

#define FRUIT2_IR1_Pin         Fruit2_IR1_Pin
#define FRUIT2_IR1_GPIO_Port   Fruit2_IR1_GPIO_Port

#define FRUIT2_IR2_Pin         Fruit2_IR2_Pin
#define FRUIT2_IR2_GPIO_Port   Fruit2_IR2_GPIO_Port

// ---- Fruit 3 (PC9, PC2, PC3) ----
#define FRUIT3_LED_Pin         GPIO_PIN_10
#define FRUIT3_LED_GPIO_Port   GPIOA

#define FRUIT3_IR1_Pin         GPIO_PIN_7
#define FRUIT3_IR1_GPIO_Port   GPIOC

#define FRUIT3_IR2_Pin         GPIO_PIN_6
#define FRUIT3_IR2_GPIO_Port   GPIOB

// ---- Fruit 4 (PA4, PB4, PB5) ----
#define FRUIT4_LED_Pin         GPIO_PIN_4
#define FRUIT4_LED_GPIO_Port   GPIOA

#define FRUIT4_IR1_Pin         GPIO_PIN_4
#define FRUIT4_IR1_GPIO_Port   GPIOB

#define FRUIT4_IR2_Pin         GPIO_PIN_5
#define FRUIT4_IR2_GPIO_Port   GPIOB

// ---- Button ----
#define START_BUTTON_Pin       GPIO_PIN_0
#define START_BUTTON_GPIO_Port   GPIOB

// Buzzer on TIM3_CH1 (PC6)
extern TIM_HandleTypeDef htim3;
#define BUZZER_TIM             (&htim3)
#define BUZZER_CHANNEL         TIM_CHANNEL_1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
