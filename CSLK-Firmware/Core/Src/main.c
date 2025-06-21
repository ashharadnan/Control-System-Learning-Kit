/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char RXbuffer[1024];
char TXbuffer[1024];

conf Config = {1300, 4199, CHANNEL_A};
bool DebugMode = true;

hx711_t loadcell1;

process PVs;

arm_pid_instance_f32 PID;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //Loadcell initialisation
    hx711_init(&loadcell1, HX_Clk_GPIO_Port, HX_Clk_Pin, HX_Data_GPIO_Port, HX_Data_Pin);
    set_gain(&loadcell1, 128, 32);
    set_scales(&loadcell1, 26780, 1);
    tare_all(&loadcell1, 10);

    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);

    PID.Kp = 1;
    PID.Ki = 0;
    PID.Kd = 0;
    arm_pid_init_f32(&PID, 1);

    PVs.delimiter = '?';
    PVs.end = '\n';

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float32_t val;
  while (1)
  {
      if (RXbuffer[0] != 0x00){
          char *loc;
          char *oper;

          loc = strstr(RXbuffer, "HELP");
          if (loc != NULL){
              strncpy(TXbuffer,
                      "SP d.f\t\t:\tSet the setpoint to d.f\r\n"
                      "DEBUG {0,1}\t:\tSet debug mode\r\n"
                      "THE FOLLOWING WILL ONLY WORK IN DEBUG MODE!\r\n"
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
                      "MEASURE\t:\tGet the current value\r\n"
                      "\t\t--Control--\r\n"
                      "KP d.f\t\t:\tSet the Proportional Gain to d.f\r\n"
                      "KI d.f\t\t:\tSet the Proportional Gain to d.f\r\n",
                      1024);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
          }

          loc = strstr(RXbuffer, "SP ");
          if (loc != NULL){
              oper = loc + 3;
              PVs.setpoint = atof(oper);
              snprintf(TXbuffer, 1024, "Setpoint: %.4f\r\n", PVs.setpoint);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
          }

          loc = strstr(RXbuffer, "DEBUG ");
          if (loc != NULL){
              oper = loc + 6;
              DebugMode = (bool)atoi(oper);
              snprintf(TXbuffer, 1024, "Debug Mode: %s\r\n", DebugMode ? "on" : "off");
              if (!DebugMode) arm_pid_reset_f32(&PID);
              CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
              memset(RXbuffer, 0x00, 1024);
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
              }
              loc = strstr(RXbuffer, "TARE");
              if (loc != NULL) {
                  snprintf(TXbuffer, 1024, "Tare both channels\r\n");
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
                  tare_all(&loadcell1, 10);
              }
              loc = strstr(RXbuffer, "CHANNEL ");
              if (loc != NULL) {
                  oper = loc + 8;
                  bool ch = atoi(oper);
                  snprintf(TXbuffer, 1024, "Switched to CH%s\r\n", ch ? "B": "A");
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
                  Config.HX_CH = ch;
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
              }
              loc = strstr(RXbuffer, "SCALE ");
              if (loc != NULL) {
                  oper = loc + 6;
                  float32_t scale = atof(oper);
                  snprintf(TXbuffer, 1024, "Scale set to %.4f\r\n", Config.HX_CH ? "B": "A", scale);
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
                  set_scales(&loadcell1, scale, scale);
              }
              loc = strstr(RXbuffer, "MEASURE");
              if (loc != NULL) {
                  float32_t meas;
                  meas = get_weight(&loadcell1, 10, Config.HX_CH);
                  snprintf(TXbuffer, 1024, "Measured: %.4f\r\n", meas);
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
              }
              loc = strstr(RXbuffer, "KP ");
              if (loc != NULL) {
                  oper = loc + 3;
                  PID.Kp = atof(oper);
                  arm_pid_init_f32(&PID, 1);
                  snprintf(TXbuffer, 1024, "Kp changed to: %.4f\r\n", PID.Kp);
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
              }
              loc = strstr(RXbuffer, "KI ");
              if (loc != NULL) {
                  oper = loc + 3;
                  PID.Ki = atof(oper) * Ts;
                  arm_pid_init_f32(&PID, 1);
                  snprintf(TXbuffer, 1024, "Ki changed to: %.4f\r\n", PID.Ki);
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
              }
              loc = strstr(RXbuffer, "KD ");
              if (loc != NULL) {
                  oper = loc + 3;
                  PID.Kd = atof(oper) / Ts;
                  arm_pid_init_f32(&PID, 1);
                  snprintf(TXbuffer, 1024, "Kd changed to: %.4f\r\n", PID.Kd);
                  CDC_Transmit_FS(TXbuffer, strlen(TXbuffer));
                  memset(RXbuffer, 0x00, 1024);
              }
          }

      }
      PVs.height = get_weight(&loadcell1, 5, Config.HX_CH);
      HAL_Delay(0);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 34999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HX_Clk_Pin|IN2_Pin|ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HX_Data_Pin */
  GPIO_InitStruct.Pin = HX_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HX_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HX_Clk_Pin */
  GPIO_InitStruct.Pin = HX_Clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HX_Clk_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_Pin */
  GPIO_InitStruct.Pin = DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin ENA_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Set_PWM(uint16_t pwm){
    if (pwm > Config.PWM_MAX){
        TIM1->CCR1 = Config.PWM_MAX;
    }
    else if(pwm <= Config.PWM_MIN){
        TIM1->CCR1 = 0;
    }
    else{
        TIM1->CCR1 = pwm;
    }

}

void Set_PWM_percent(float32_t percent){
    PVs.pv = percent;
    percent = (float32_t)Config.PWM_MIN + (float32_t)(Config.PWM_MAX - Config.PWM_MIN) * (percent / 100.0);
    Set_PWM((uint16_t) percent);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
