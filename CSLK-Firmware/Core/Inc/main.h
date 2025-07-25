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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "HX711.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint16_t PWM_MIN;
    uint16_t PWM_MAX;
    bool HX_CH;
} conf;

typedef struct {
    uint8_t delimiter;
    float32_t height;
    float32_t error;
    float32_t setpoint;
    float32_t pv;
    uint8_t end;
} process;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern char RXbuffer[1024];
extern char TXbuffer[1024];

extern conf Config;
extern bool DebugMode;

extern process PVs;

extern arm_pid_instance_f32 PID;
extern bool AntiWindup;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Set_PWM(uint16_t pwm);
void Set_PWM_percent(float32_t percent);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define HX_Data_Pin GPIO_PIN_6
#define HX_Data_GPIO_Port GPIOA
#define HX_Clk_Pin GPIO_PIN_7
#define HX_Clk_GPIO_Port GPIOA
#define DEBUG_Pin GPIO_PIN_12
#define DEBUG_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_8
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_9
#define IN2_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_10
#define ENA_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define Ts 0.0005
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
