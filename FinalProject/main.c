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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include <stdio.h>
#include "ws2812_demos.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} Color;
typedef struct {
	uint8_t position;
	Color color;
} LED_RING;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_CLK_Pin GPIO_PIN_0
#define ENCODER_CLK_GPIO_Port GPIOA
#define ENCODER_DT_Pin GPIO_PIN_1
#define ENCODER_DT_GPIO_Port GPIOA

// Button on PB8
#define ENCODER_BUTTON_GPIO_PIN GPIO_PIN_8
#define ENCODER_BUTTON_PORT GPIOB

// Colors
#define COLOR_RED   (Color){255, 0, 0}
#define COLOR_GREEN (Color){0, 255, 0}
#define COLOR_BLUE  (Color){0, 0, 255}
#define COLOR_WHITE (Color){255, 255, 255}
#define COLOR_OFF   (Color){0, 0, 0}/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
ws2812_handleTypeDef ws2812; // The WS2812 panel handler
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t lastCLKState;
volatile uint8_t rotationFlag = 0; // 1 for CW 2 for CWW
static uint8_t lastEncoderButtonState = GPIO_PIN_SET;
volatile uint8_t encoderButtonPressed = 0;  // For PB8

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void setLedColor(ws2812_handleTypeDef *ws2812, uint16_t led, Color color);
/* USER CODE BEGIN PFP */

int _write(int fd, char *ptr, int len) {
	HAL_StatusTypeDef hstatus;

	if (fd == 1 || fd == 2) {
		hstatus = HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,
		HAL_MAX_DELAY);
		if (hstatus == HAL_OK)
			return len;
		else
			return -1;
	}
	return -1;
}
// Done sending first half of the DMA buffer - this can now safely be updated
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) {
		ws2812_update_buffer(&ws2812, &ws2812.dma_buffer[0]);
	}

}

// Done sending the second half of the DMA buffer - this can now be safely updated
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) {
		ws2812_update_buffer(&ws2812, &ws2812.dma_buffer[BUFFER_SIZE]);
	}

}

// On interrupt ever 1 ms, check for the rotations and also button presses
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// First get the CLK and DATA from the encoder
	int8_t currentCLK = HAL_GPIO_ReadPin(ENCODER_CLK_GPIO_Port,
			ENCODER_CLK_Pin);
	uint8_t currentDT = HAL_GPIO_ReadPin(ENCODER_DT_GPIO_Port, ENCODER_DT_Pin);

	if (currentCLK == GPIO_PIN_RESET && lastCLKState == GPIO_PIN_SET) {
		if (currentDT != currentCLK) {
			rotationFlag = 1;  // CW
		} else {
			rotationFlag = 2;  // CCW
		}
	}
	lastCLKState = currentCLK;

	// Button checks
	uint8_t encoderButtonState = HAL_GPIO_ReadPin(ENCODER_BUTTON_PORT,
			ENCODER_BUTTON_GPIO_PIN);
	if (encoderButtonState == GPIO_PIN_RESET
			&& lastEncoderButtonState == GPIO_PIN_SET) {
		encoderButtonPressed = 1;
	}
	lastEncoderButtonState = encoderButtonState;
}

void setLedColor(ws2812_handleTypeDef *ws2812, uint16_t led, Color color) {
	setLedValues(ws2812, led, color.r, color.g, color.b);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	HAL_TIM_Base_Start_IT(&htim2);

	/* USER CODE BEGIN 2 */
	printf("Hello world!\n");

	ws2812_init(&ws2812, &htim4, TIM_CHANNEL_1, 24);

	ws2812_demos_set(&ws2812, 1);

	// Set the first LED to red
	/* USER CODE END 2 */
	LED_RING ledRing1;
	ledRing1.color = COLOR_BLUE;
	ledRing1.position = 0;

	LED_RING ledRing2;
	ledRing2.color = COLOR_BLUE;
	ledRing2.position = 0;

	// Ring 2 (next 12 LEDs)
	for (int i = 0; i < 12; i++) {
	    setLedColor(&ws2812, i + 12, COLOR_RED);  // For Ring 2 LEDs
	}
//	for (ledRing1.position = 0; ledRing1.position < 12; ledRing1.position++) {
//		zeroLedValues(&ws2812);  // Clear previous LEDs
//		setLedColor(&ws2812, ledRing1.position, COLOR_BLUE);
//		HAL_Delay(300);  // Wait 500ms
//	}
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		if (rotationFlag == 1) {
			if (ledRing1.position < 11) {
				ledRing1.position += 1;
			}
			printf("CW\n");
//			zeroLedValues(&ws2812);
			setLedColor(&ws2812, ledRing1.position, COLOR_BLUE);
			rotationFlag = 0;
		} else if (rotationFlag == 2) {
			if (ledRing1.position > 0) {
				setLedColor(&ws2812, ledRing1.position, COLOR_OFF);
				ledRing1.position -= 1;
			}
			printf("CCW\n");
//			zeroLedValues(&ws2812);
			setLedColor(&ws2812, ledRing1.position, COLOR_BLUE);
			rotationFlag = 0;
		}

		if (encoderButtonPressed) {
			printf("Encoder Button was pressed!\n");
			encoderButtonPressed = 0;
		}
	}
	/* USER CODE END WHILE */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 79;
	htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = LED_CNT;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA1 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB3 PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
