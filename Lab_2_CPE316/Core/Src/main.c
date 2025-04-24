/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    : main.c
 * @brief   : Main program body
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

#define Number_of_Keys 12
#define Number_of_Cols 3
#define PA0 0x0001
#define PA1 0x0002
#define PA4 0x0010
#define PB0 0x0001
#define PC1 0x0002
#define PC0 0x0001
#define PA10 0x0400
#define KeyDetect 0x0001
#define KeyLow2High 0x0002

typedef struct
{
    unsigned short sKeyRead;
    unsigned short sKeyReadTempPos;
    unsigned short sKeySend;
    unsigned short sKeyCol;
    char KeyLetter;
    unsigned short sKeyCommand;
} Key_Contorl_struct_t;

typedef enum KeyName
{
    ONE_command,
    FOUR_command,
    SEVEN_command,
    STAR_command,
    TWO_command,
    FIVE_command,
    EIGHT_command,
    ZERO_command,
    THREE_command,
    SIX_command,
    NINE_command,
    POUND_command
} KeyName;

void key1(void);
void key2(void);
void key3(void);
void key4(void);
void key5(void);
void key6(void);
void key7(void);
void key8(void);
void key9(void);
void key0(void);
void keyS(void);
void keyP(void);

/******** Structure *******/
Key_Contorl_struct_t sKeyControl[Number_of_Keys] =
{
    {PA10, 0x8, PA4, 0, '1', ONE_command},      // PA10 (read), PA4 (send)
    {PC0,  0x4, PA4, 0, '4', FOUR_command},     // PC0 (read), PA4 (send)
    {PC1,  0x2, PA4, 0, '7', SEVEN_command},    // PC1 (read), PA4 (send)
    {PB0,  0x1, PA4, 0, '*', STAR_command},     // PB0 (read), PA4 (send)
    {PA10, 0x8, PA1, 1, '2', TWO_command},      // PA10 (read), PA1 (send)
    {PC0,  0x4, PA1, 1, '5', FIVE_command},     // PC0 (read), PA1 (send)
    {PC1,  0x2, PA1, 1, '8', EIGHT_command},    // PC1 (read), PA1 (send)
    {PB0,  0x1, PA1, 1, '0', ZERO_command},     // PB0 (read), PA1 (send)
    {PA10, 0x8, PA0, 2, '3', THREE_command},    // PA10 (read), PA0 (send)
    {PC0,  0x4, PA0, 2, '6', SIX_command},      // PC0 (read), PA0 (send)
    {PC1,  0x2, PA0, 2, '9', NINE_command},     // PC1 (read), PA0 (send)
    {PB0,  0x1, PA0, 2, '#', POUND_command}     // PB0 (read), PA0 (send)
};

unsigned short sKeyStatus;
unsigned short sKeyCurrentCol[Number_of_Cols];
unsigned short sKeyDebouncedCol[Number_of_Cols];
unsigned short sKeyIssued;
unsigned short sKeyPreviousCol[Number_of_Cols];
unsigned short sKeyLow2HighCol[Number_of_Cols];

UART_HandleTypeDef huart2;

void Keypadscan(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
    unsigned short sIndex;
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Clear all debounced records, Previous, Low2High
    for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
    {
        sKeyDebouncedCol[sIndex] = 0x0000;
        sKeyPreviousCol[sIndex] = 0x0000;
        sKeyLow2HighCol[sIndex] = 0x0000;
    }

    while (1)
    {
        HAL_Delay(10);
        Keypadscan();
        if ((sKeyStatus & KeyDetect) && (sKeyIssued != 0xFFFF))
        {
            switch (sKeyIssued)
            {
                case ONE_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key1();
                    break;
                }
                case FOUR_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key4();
                    break;
                }
                case SEVEN_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key7();
                    break;
                }
                case STAR_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        keyS();
                    break;
                }
                case TWO_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key2();
                    break;
                }
                case FIVE_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key5();
                    break;
                }
                case EIGHT_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key8();
                    break;
                }
                case ZERO_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key0();
                    break;
                }
                case THREE_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key3();
                    break;
                }
                case SIX_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key6();
                    break;
                }
                case NINE_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        key9();
                    break;
                }
                case POUND_command:
                {
                    if (sKeyStatus & KeyLow2High)
                        keyP();
                    break;
                }
                default:
                    break;
            }
            sKeyStatus &= ~(KeyDetect | KeyLow2High);
            // Clear all Low-2-High and High-2-Low records
            for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
                sKeyLow2HighCol[sIndex] = 0x0000;
        }
    }
}

void Keypadscan()
{
    unsigned short sIndex;
    unsigned short Temp;

    // Clear all key records
    for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
    {
        sKeyCurrentCol[sIndex] = 0x00;
    }

    // Read all 3 column
    for (sIndex = 0; sIndex < Number_of_Keys; sIndex++)
    {
        GPIOA->ODR &= ~(PA4 | PA1 | PA0);
        GPIOA->ODR |= sKeyControl[sIndex].sKeySend;
        HAL_Delay(0.5);

        switch (sKeyControl[sIndex].sKeyCommand)
        {
            case ONE_command:
            case TWO_command:
            case THREE_command:
                if (GPIOA->IDR & sKeyControl[sIndex].sKeyRead)
                    sKeyCurrentCol[sKeyControl[sIndex].sKeyCol] = sKeyControl[sIndex].sKeyReadTempPos;
                break;
            case FOUR_command:
            case FIVE_command:
            case SIX_command:
            case SEVEN_command:
            case EIGHT_command:
            case NINE_command:
                if (GPIOC->IDR & sKeyControl[sIndex].sKeyRead)
                    sKeyCurrentCol[sKeyControl[sIndex].sKeyCol] = sKeyControl[sIndex].sKeyReadTempPos;
                break;
            case STAR_command:
            case ZERO_command:
            case POUND_command:
                if (GPIOB->IDR & sKeyControl[sIndex].sKeyRead)
                    sKeyCurrentCol[sKeyControl[sIndex].sKeyCol] = sKeyControl[sIndex].sKeyReadTempPos;
        }
    }

    // Check if a key is steadily read
    for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
    {
        if ((sKeyCurrentCol[sIndex] == sKeyDebouncedCol[sIndex]) && (sKeyCurrentCol[sIndex] != 0x0000))
            break;
    }

    if (sIndex < Number_of_Cols)
    {
        // Check for push on/ push off (Low To High)
        for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
        {
            Temp = sKeyCurrentCol[sIndex] ^ sKeyPreviousCol[sIndex];
            sKeyLow2HighCol[sIndex] = (sKeyCurrentCol[sIndex] & Temp);
        }

        // Update Previous records
        for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
        {
            sKeyPreviousCol[sIndex] = sKeyCurrentCol[sIndex];
        }

        // Find which key is JUST depressed (Low To High)
        for (sIndex = 0; sIndex < Number_of_Keys; sIndex++)
        {
            if (sKeyLow2HighCol[sKeyControl[sIndex].sKeyCol] & sKeyControl[sIndex].sKeyReadTempPos)
            {
                sKeyIssued = sKeyControl[sIndex].sKeyCommand;
                {
                    sKeyStatus |= (KeyDetect | KeyLow2High);
                    break;
                }
            }
            else
                sKeyIssued = 0xFFFF;
        }
    }
    else
    {
        sKeyStatus &= ~(KeyDetect | KeyLow2High);
        for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
            sKeyPreviousCol[sIndex] = 0;
    }

    // Transfer Current reading to debounced reading
    for (sIndex = 0; sIndex < Number_of_Cols; sIndex++)
    {
        sKeyDebouncedCol[sIndex] = sKeyCurrentCol[sIndex];
        sKeyLow2HighCol[sIndex] = 0;
    }
}

void key1()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_1_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key2()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_2_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key3()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_3_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key4()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_4_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key5()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_5_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key6()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_6_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key7()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_7_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key8()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_8_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key9()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_9_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void key0()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_0_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void keyS()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_S_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void keyP()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"Key_P_ON", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
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

    /** Initializes the CPU, AHB and APB buses clocks */
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC0 PC1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA1 PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief This function is executed in case of error occurrence.
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

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
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
