#include "main.h"
#include "Timer.h"
#include <string.h>
#include <stdbool.h>

#define IODIRA    0x00    // MCP23017 I/O-direction register A
#define IODIRB    0x01    // MCP23017 I/O-direction register B
#define MCP_GPIOA 0x12    // MCP23017 GPIO port A
#define MCP_GPIOB 0x13    // MCP23017 GPIO port B
#define MCP_GPPUA 0x0C    // MCP23017 pull-up register A

#define NS_Green   0x01   // PB0
#define NS_Yellow  0x02   // PB1
#define NS_Red     0x04   // PB2
#define EW_Green   0x08   // PB3
#define EW_Yellow  0x10   // PB4
#define EW_Red     0x20   // PB5

#define Timer_Running 0x01

uint8_t flags = 0x00;       // our “timer running” flag
char    I2C_ADDRESS = 0x40; // 7-bit <<1, LSB=0 for write
static uint8_t SensorIn;    // raw port A read

// FSM state structure
struct State {
    unsigned char    Out;   // bits for port B
    unsigned short   Time;  // ms
    struct State    *Next[4];// next-state pointers by input (0–3)
};
typedef struct State STyp;

// helper pointers for readability
#define gN  &FSM[0]
#define wN  &FSM[1]
#define gE  &FSM[2]
#define wE  &FSM[3]

// your 4-step cycle (3s green, 1s yellow each direction)
STyp FSM[4] = {
    { (NS_Green|EW_Red),    3000, { wN, wN, wN, wN } },
    { (NS_Yellow|EW_Red),   1000, { gE, gE, gE, gE } },
    { (NS_Red|EW_Green),    3000, { wE, wE, wE, wE } },
    { (NS_Red|EW_Yellow),   1000, { gN, gN, gN, gN } }
};

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

// send one register addr + data
void I2CSend(char port, char data) {
    uint8_t buf[2] = { (uint8_t)port, (uint8_t)data };
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, buf, 2, 1000);
}
// send just the register addr (for a subsequent read)
void I2CSendOne(char port) {
    uint8_t buf[1] = { (uint8_t)port };
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, buf, 1, 1000);
}

// TIM2 ISR calls this to decrement your sTimer[]
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        TIMER2_HANDLE();
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // init UART, GPIO, I2C, Timer
    MX_USART2_UART_Init();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    // configure MCP23017
    HAL_I2C_Init(&hi2c1);
    I2CSend(IODIRB,    0x00); // port B = outputs
    I2CSend(MCP_GPPUA, 0x03); // port A bits0-1 pull-up
    I2CSend(IODIRA,    0xFF); // port A = inputs

    HAL_TIM_Base_Start_IT(&htim2);

    // pedestrian flags
    bool PedestrianButtonPressedEW = false;
	bool PedestrianButtonPressedNS = false;
    bool extendedGreenNS          = false;
	bool extendedGreenEW          = false;

    STyp *Pt = gN;    // start at North-Green
    unsigned char Input;

    while (1)
    {
        // ─── 1) STATE ENTRY ───────────────────────────
        if (!(flags & Timer_Running)) {
            // drive LEDs
            I2CSend(MCP_GPIOB, Pt->Out);

            // compute duration (use 32-bit for +2000)
            uint32_t time = Pt->Time;
            // extend next E-W green by 2s if requested
            if (Pt == gE && PedestrianButtonPressedEW && !extendedGreenEW) {
                time += 2000;
                extendedGreenEW = true;
            }
			else if(Pt == gN && PedestrianButtonPressedNS && !extendedGreenNS)
			{
				time += 3000;
				extendedGreenNS = true;
			}
            // start countdown
            sTimer[TRAFFIC_STATE_TIMER] = time;
            flags |= Timer_Running;
        }

        // ─── 2) STATE EXIT (when timer hits 0) ─────────
        if (sTimer[TRAFFIC_STATE_TIMER] == 0) {
            // read port A
            I2CSendOne(MCP_GPIOA);
            HAL_I2C_Master_Receive(&hi2c1,
                                   (I2C_ADDRESS | 0x01),
                                   &SensorIn, 1, 1000);

			// invert bits for active-low inputs
            Input = (~SensorIn) & 0x03;

            // UART on first PA0 press
            if ((Input & 0x01) && !PedestrianButtonPressedEW) {
                PedestrianButtonPressedEW = true;
                const char msg[] = "EW pedestrian request\r\n";
                HAL_UART_Transmit(&huart2,
                                  (uint8_t*)msg,
                                  strlen(msg),
                                  HAL_MAX_DELAY);
            }
			else if ((Input & 0x02) && !PedestrianButtonPressedNS) {
				PedestrianButtonPressedNS = true;
				const char msg[] = "NS pedestrian request\r\n";
				HAL_UART_Transmit(&huart2,
								  (uint8_t*)msg,
								  strlen(msg),
								  HAL_MAX_DELAY);
			}

            // save old state, advance FSM
            STyp *prevState = Pt;
            Pt = Pt->Next[Input];
            flags &= ~Timer_Running;

            // UART when leaving extended E-W green
            if (prevState == gE && extendedGreenEW) {
                const char msg2[] = "Traffic Pattern Back to Normal\r\n";
                HAL_UART_Transmit(&huart2,
                                  (uint8_t*)msg2,
                                  strlen(msg2),
                                  HAL_MAX_DELAY);
                // reset for next cycle
                extendedGreenEW          = false;
                PedestrianButtonPressedEW = false;
            }
			else if (prevState == gN && extendedGreenNS) {
				const char msg2[] = "Traffic Pattern Back to Normal\r\n";
				HAL_UART_Transmit(&huart2,
								  (uint8_t*)msg2,
								  strlen(msg2),
								  HAL_MAX_DELAY);
				// reset for next cycle
				extendedGreenNS          = false;
				PedestrianButtonPressedNS = false;
			}
        }
    }
}

// … rest of your MX_*_Init() and Error_Handler() as before …



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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 19;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
