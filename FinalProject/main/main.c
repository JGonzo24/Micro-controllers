/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Timer.h"
#include <string.h>
#include "LCD.h"
#include <stdio.h>




/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Rotary Encoder Pins
#define ENCODER_CLK_Pin GPIO_PIN_0
#define ENCODER_CLK_GPIO_Port GPIOA
#define ENCODER_DT_Pin GPIO_PIN_1
#define ENCODER_DT_GPIO_Port GPIOA

// Button on PB8
#define BUTTON1_Pin GPIO_PIN_8
#define BUTTON1_GPIO_Port GPIOB

// Button on PB9 ("Starting Timer")
#define BUTTON2_Pin GPIO_PIN_9
#define BUTTON2_GPIO_Port GPIOB

// Button on PA5 ("Timer Paused")
#define BUTTON3_Pin GPIO_PIN_5
#define BUTTON3_GPIO_Port GPIOA

// Button on PA6 ("Buzzer")
#define Button4_Pin GPIO_PIN_6
#define Button4_GPIO_Port GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t lastCLKState;
volatile uint8_t rotationFlag = 0; // 1 for CW 2 for CWW


volatile uint8_t buttonPressedFlag1 = 0;  // For PB8
volatile uint8_t buttonPressedFlag2 = 0;  // For PB9
volatile uint8_t buttonPressedFlag3 = 0;  // For PA5

static uint8_t lastButtonState1 = GPIO_PIN_SET;
static uint8_t lastButtonState2 = GPIO_PIN_SET;
static uint8_t lastButtonState3 = GPIO_PIN_SET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    TIMER2_HANDLE();

    // Rotary encoder check (same as before)
    uint8_t currentCLK = HAL_GPIO_ReadPin(ENCODER_CLK_GPIO_Port, ENCODER_CLK_Pin);
    uint8_t currentDT  = HAL_GPIO_ReadPin(ENCODER_DT_GPIO_Port, ENCODER_DT_Pin);

    if (currentCLK == GPIO_PIN_RESET && lastCLKState == GPIO_PIN_SET) {
        if (currentDT != currentCLK) {
            rotationFlag = 1;  // CW
        } else {
            rotationFlag = 2;  // CCW
        }
    }
    lastCLKState = currentCLK;

    // Button checks
    uint8_t buttonState1 = HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin);
    if (buttonState1 == GPIO_PIN_RESET && lastButtonState1 == GPIO_PIN_SET) {
        buttonPressedFlag1 = 1;
    }
    lastButtonState1 = buttonState1;

    uint8_t buttonState2 = HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin);
    if (buttonState2 == GPIO_PIN_RESET && lastButtonState2 == GPIO_PIN_SET) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"PB9 pressed!\n", strlen("PB9 pressed!\n"), HAL_MAX_DELAY);

        buttonPressedFlag2 = 1;
    }
    lastButtonState2 = buttonState2;

    uint8_t buttonState3 = HAL_GPIO_ReadPin(BUTTON3_GPIO_Port, BUTTON3_Pin);
    if (buttonState3 == GPIO_PIN_RESET && lastButtonState3 == GPIO_PIN_SET) {
        buttonPressedFlag3 = 1;
    }
    lastButtonState3 = buttonState3;

}

typedef enum {
    CONFIG_HOURS,
    CONFIG_MINUTES,
    CONFIG_SECONDS
} ConfigMode;

volatile ConfigMode configState = CONFIG_HOURS;
volatile uint8_t rotarySwitchPressed = 0;  // Set by ISR on rotary switch press
volatile uint8_t configModeActive = 1;  // Start in configuration mode

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





TimeStruct countdownTimer = {0,0,10};
volatile uint8_t updateDisplayFlag = 0;
volatile uint8_t timerRunning = 0;  // 1 = running, 0 = paused
volatile uint8_t timerDoneFlag = 0;



void Buzzer_On(void) {
    HAL_GPIO_WritePin(Button4_GPIO_Port, Button4_Pin, GPIO_PIN_SET);  // Turn on
}

void Buzzer_Off(void) {
    HAL_GPIO_WritePin(Button4_GPIO_Port, Button4_Pin, GPIO_PIN_RESET);  // Turn off
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    lastCLKState = HAL_GPIO_ReadPin(ENCODER_CLK_GPIO_Port, ENCODER_CLK_Pin);
    HAL_TIM_Base_Start_IT(&htim2);

    char buffer[16];
    LcdInit();
    LcdClear();
    LcdGoto(0, 0);
    LcdPutS("Configure Timer using the black knob!");
    HAL_Delay(5000);
    LcdClear();
    LcdPutS("Black: Start");
    LcdGoto(1, 0);
    LcdPutS("Red: Stop");
    HAL_Delay(3000);
    LcdClear();

    updateDisplayFlag = 1;

    while (1) {
        // CONFIGURATION MODE OR TIMER PAUSED
        if (configModeActive || !timerRunning) {
            // Rotary switch pressed - change setting
            if (rotarySwitchPressed) {
                configState = (configState + 1) % 3;
                rotarySwitchPressed = 0;
                updateDisplayFlag = 1;
            }

            // Rotary encoder turned
            if (rotationFlag == 1) {
                if (configState == CONFIG_HOURS && countdownTimer.hours < 23) countdownTimer.hours++;
                else if (configState == CONFIG_MINUTES && countdownTimer.minutes < 59) countdownTimer.minutes++;
                else if (configState == CONFIG_SECONDS && countdownTimer.seconds < 59) countdownTimer.seconds++;
                rotationFlag = 0;
                updateDisplayFlag = 1;
            } else if (rotationFlag == 2) {
                if (configState == CONFIG_HOURS && countdownTimer.hours > 0) countdownTimer.hours--;
                else if (configState == CONFIG_MINUTES && countdownTimer.minutes > 0) countdownTimer.minutes--;
                else if (configState == CONFIG_SECONDS && countdownTimer.seconds > 0) countdownTimer.seconds--;
                rotationFlag = 0;
                updateDisplayFlag = 1;
            }

            // LCD Update
            if (updateDisplayFlag) {
                updateDisplayFlag = 0;
                sprintf(buffer, "%02u:%02u:%02u", countdownTimer.hours, countdownTimer.minutes, countdownTimer.seconds);
                LcdClear();
                LcdGoto(0, 0);
                LcdPutS(buffer);

                // Show "Timer Paused" if paused but not in config mode
                if (!timerRunning && !configModeActive) {
                    LcdGoto(1, 0);
                    LcdPutS("Timer Paused");
                }
                // Otherwise show config state (Hrs/Min/Sec)
                else if (configModeActive) {
                    LcdGoto(1, 0);
                    if (configState == CONFIG_HOURS) LcdPutS("Set: Hrs");
                    else if (configState == CONFIG_MINUTES) LcdPutS("Set: Min");
                    else if (configState == CONFIG_SECONDS) LcdPutS("Set: Sec");
                }
            }

            // Start or Resume Timer
            if (buttonPressedFlag2) {
                if (countdownTimer.hours > 0 || countdownTimer.minutes > 0 || countdownTimer.seconds > 0) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)"Starting/Resuming Timer\n", strlen("Starting/Resuming Timer\n"), HAL_MAX_DELAY);
                    timerRunning = 1;
                    configModeActive = 0;  // Exit config mode if needed
                } else {
                    LcdClear();
                    LcdPutS("Set time first!");
                    HAL_Delay(2000);
                    updateDisplayFlag = 1;
                }
                buttonPressedFlag2 = 0;
            }
        }

        // TIMER RUNNING MODE
        else {
            if (updateDisplayFlag) {
                updateDisplayFlag = 0;
                sprintf(buffer, "%02u:%02u:%02u", countdownTimer.hours, countdownTimer.minutes, countdownTimer.seconds);
                LcdClear();
                LcdGoto(0, 0);
                LcdPutS(buffer);
            }

            // Pause Timer
            if (buttonPressedFlag3) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Timer Paused\n", strlen("Timer Paused\n"), HAL_MAX_DELAY);
                timerRunning = 0;
                buttonPressedFlag3 = 0;
            }

            // Optional: Other Button Press (PB8)
            if (buttonPressedFlag1) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Button PB8 Pressed!\n", strlen("Button PB8 Pressed!\n"), HAL_MAX_DELAY);
                buttonPressedFlag1 = 0;
            }
        }

        // Rotary Switch Detection (press)
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET) {
            rotarySwitchPressed = 1;
            HAL_Delay(200);  // Debounce
        }

        // Timer Done Handling
        if (timerDoneFlag) {
            LcdClear();
            LcdGoto(0, 0);
            LcdPutS("Timer Done!");
            LcdGoto(1, 0);
            LcdPutS("Returning to Setup");
            for (int i = 0; i < 3; i++) {
                Buzzer_On(); HAL_Delay(300); Buzzer_Off(); HAL_Delay(300);
            }
            timerDoneFlag = 0;
            HAL_Delay(3000);
            updateDisplayFlag = 1;
        }
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
