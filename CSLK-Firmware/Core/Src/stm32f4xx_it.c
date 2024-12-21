/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */
  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  if (RXbuffer[0] != 0x00){
      char *loc;
      char *oper;

      loc = strstr(RXbuffer, "HELP");
      if (loc != NULL){
          strncpy(TXbuffer,
                  "DEBUG {0,1}\t:\tSet debug mode\r\n"
                  "\t\t--PWM--\r\n"
                  "PWM d\t\t:\tSet PWM CCR to d\r\n"
                  "PWM% d.f\t:\tSet PWM as a percentage d.f%\r\n"
                  "PWMMIN d\t:\tSet the minimum saturation range for PWM to d\r\n"
                  "PWMMAX d\t:\tSet the maximum saturation range for PWM to d\r\n"
                  "\t\t--LoadCell--\r\n"
                  "TARE\t\t:\tTare the loadcell weight\r\n"
                  "CHANNEL {0,1}\t:\tChange between channel CHA and CHB\r\n"
                  "CHGAIN {0,1}\t:\tChange between low and high channel (CHA) gain\r\n"
                  "SCALE d.f\t:\tSet the scale of the current channel to d.f\r\n"
                  "MEASURE\t:\tGet the current value\r\n",
                  1024);
          CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
          memset(RXbuffer, 0x00, 1024);
          return;
      }

      loc = strstr(RXbuffer, "DEBUG ");
      if (loc != NULL){
          oper = loc + 6;
          DebugMode = (bool)atoi(oper);
          snprintf(TXbuffer, 1024, "Debug Mode: %s\r\n", DebugMode ? "on" : "off");
          CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
          memset(RXbuffer, 0x00, 1024);
          return;
      }

      if (DebugMode){
          loc = strstr(RXbuffer, "PWM ");
          if (loc != NULL){
              oper = loc + 4;
              uint16_t pwm;
              pwm = atoi(oper);
              snprintf(TXbuffer, 1024, "Setting PWM to: %d saturated to (%d, %d)\r\n", pwm, Config.PWM_MIN, Config.PWM_MAX);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              Set_PWM(pwm);
              return;
          }
          loc = strstr(RXbuffer, "PWM% ");
          if (loc != NULL){
              oper = loc + 5;
              float32_t pwm;
              pwm = atof(oper);
              snprintf(TXbuffer, 1024, "Setting PWM to: %.2f %%\r\n", pwm);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              Set_PWM_percent(pwm);
              return;
          }
          loc = strstr(RXbuffer, "PWMMIN ");
          if (loc != NULL) {
              oper = loc + 7;
              uint16_t minpwm;
              minpwm = atoi(oper);
              snprintf(TXbuffer, 1024, "Setting PWM Minimum to: %d\r\n", minpwm);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              Config.PWM_MIN = minpwm;
              if (TIM1->CCR1 > minpwm){
                  Set_PWM(minpwm);
              }
              return;
          }
          loc = strstr(RXbuffer, "PWMMAX ");
          if (loc != NULL) {
              oper = loc + 7;
              uint16_t maxpwm;
              maxpwm = atoi(oper);
              snprintf(TXbuffer, 1024, "Setting PWM Maximum to: %d\r\n", maxpwm);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              Config.PWM_MAX = maxpwm;
              if (TIM1->CCR1 > maxpwm){
                  Set_PWM(maxpwm);
              }
              return;
          }
          loc = strstr(RXbuffer, "TARE");
          if (loc != NULL) {
              snprintf(TXbuffer, 1024, "Tare both channels\r\n");
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              tare_all(&loadcell1, 10);
              return;
          }
          loc = strstr(RXbuffer, "CHANNEL ");
          if (loc != NULL) {
              oper = loc + 8;
              bool ch = atoi(oper);
              snprintf(TXbuffer, 1024, "Switched to CH%s\r\n", ch ? "B": "A");
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              Config.HX_CH = ch;
              return;
          }
          loc = strstr(RXbuffer, "CHGAIN ");
          if (loc != NULL) {
              oper = loc + 7;
              bool gain = atoi(oper);
              snprintf(TXbuffer, 1024, "CHA gain set to %s\r\n", gain ? "low": "high");
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              if (gain){
                  set_gain(&loadcell1, 128, 32);
              }
              else{
                  set_gain(&loadcell1, 64, 32);
              }
              return;
          }
          loc = strstr(RXbuffer, "SCALE ");
          if (loc != NULL) {
              oper = loc + 6;
              float32_t scale = atof(oper);
              snprintf(TXbuffer, 1024, "CH%s scale set to %.4f\r\n", Config.HX_CH ? "B": "A", scale);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              set_scale(&loadcell1, Config.HX_CH, scale);
              return;
          }
          loc = strstr(RXbuffer, "MEASURE");
          if (loc != NULL) {
              float32_t meas;
              meas = get_weight(&loadcell1, 10, Config.HX_CH);
              snprintf(TXbuffer, 1024, "Measured: %.4f\r\n", meas);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
              return;
          }
      }

  }
  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
