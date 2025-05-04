

#include "main.h"
#include <string.h>
#include <stdio.h>


//macro for how many keys and columns
#define Number_of_Keys 12
#define Number_of_Cols 3

//macros for which bits each pin is in the registers
#define PA0 0x0001
#define PA1 0x0002
#define PA4 0x0010
#define PB0 0x0001
#define PC1 0x0002
#define PC0 0x0001
#define PA10 0x0400


#define TMR_UP 0x0001

//make a struct to hold the hours mins and sec
typedef struct {
    uint8_t hr;
    uint8_t min;
    uint8_t sec;
} Time;

//will go back to this every time program restarts? clears the values
Time currentTime = {0, 0, 0};

//macro for if the key got detected
#define KeyDetect 0x0001

//to detect the rising edge
//??? why is it this? idk
#define KeyLow2High 0x0002
//will keep track of when to turn it off
#define KeyHigh2Low 0x0004
//keeps track of the key status
#define KeyIsOn 0x0008
#define KeyIsOff 0x0010

//every key gets this struct
typedef struct
{
	unsigned short sKeyRead;
	unsigned short sKeyReadTempPos;
	unsigned short sKeySend;
	unsigned short sKeyCol;
	char KeyLetter;
	unsigned short sKeyCommand;
} Key_Contorl_struct_t;

//helps just name the keys
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

//these will increment
//void key1(void);
//void key2(void);
//void key3(void);

//these will decrement
void key4(void);
void key5(void);
void key6(void);

void print_time();

void key7(void);
void key8(void);
void key9(void);
//void key0(void);
//void keyS(void);
//void keyP(void);

//dont need any of these
//void key1OFF(void);
//void key2OFF(void);
//void key3OFF(void);
//void key4OFF(void);
//void key5OFF(void);
//void key6OFF(void);
//void key7OFF(void);
//void key8OFF(void);
//void key9OFF(void);
//void key0OFF(void);
//void keySOFF(void);
//void keyPOFF(void);

//no header file again so function definitions up here
char *cursor_pos(int, int);

//this is the table
//this keeps track of the keypad and tells you for every key, which is the reading pin and which pin powers it
//also has the keyReadTempPos field, which acts as a bitmask for where the key is on the board
/******** Structure *******/
Key_Contorl_struct_t sKeyControl[Number_of_Keys]
={
{PA10,0x8,PA4,0,'1',ONE_command}, // PA10 (read), PA4 (send)
{PC0,0x4,PA4,0,'4',FOUR_command}, // PC0 (read), PA4 (send)
{PC1,0x2,PA4,0,'7',SEVEN_command}, // PC1 (read), PA4 (send)
{PB0,0x1,PA4,0,'*',STAR_command}, // PB0 (read), PA4 (send)
{PA10,0x8,PA1,1,'2',TWO_command}, // PA10 (read), PA1 (send)
{PC0,0x4,PA1,1,'5',FIVE_command}, // PC0 (read), PA1 (send)
{PC1,0x2,PA1,1,'8',EIGHT_command}, // PC1 (read), PA1 (send)
{PB0,0x1,PA1,1,'0',ZERO_command}, // PB0 (read), PA1 (send)
{PA10,0x8,PA0,2,'3',THREE_command}, // PA10 (read), PA0 (send)
{PC0,0x4,PA0,2,'6',SIX_command}, // PC0 (read), PA0 (send)
{PC1,0x2,PA0,2,'9',NINE_command}, // PC1 (read), PA0 (send)
{PB0,0x1,PA0,2,'#',POUND_command} // PB0 (read), PA0 (send)
};

char CursorHome[] = { 0x1B, '[' , 'H' , 0 };
char ClearScreen[] = { 0x1B, '[', '2' , 'J',0 }; // Clear the screen


unsigned short sKeyStatus;
//this array keeps track of the current key and us updated when each key gets pressed
unsigned short sKeyCurrentCol[Number_of_Cols];
//makes sure that the previous press was the same as the current to check if the signal stays stable and is not zero
unsigned short sKeyDebouncedCol[Number_of_Cols];
unsigned short sKeyIssued;
unsigned short sKeyReleased;
//keeps track of what the previous value was to help detect the rising edge
unsigned short sKeyPreviousCol[Number_of_Cols];
//keeps track of if the key that was just pressed went from low to high, then count it as a valid press
unsigned short sKeyLow2HighCol[Number_of_Cols];
unsigned short sKeyHigh2LowCol[Number_of_Cols];
static uint32_t flags = 0;
static uint32_t TickCount = 0;
static void MX_TIM2_Init(void);

UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

void Keypadscan(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


//the same functions as last time to send data or to send a new line
void UART_send(UART_HandleTypeDef *huart, char buffer[])
{
	HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void UART_send_newline(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
}


int main(void)
{
	unsigned short sIndex;
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	//init the timer
	MX_TIM2_Init();

	//now start it
	HAL_TIM_Base_Start_IT(&htim2);
	//clear the screen and print out the starting print time
	UART_send(&huart2, ClearScreen);
	print_time();

	//clear all the arrays at the start
	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
	{
		sKeyDebouncedCol[sIndex] = 0x0000;
		sKeyPreviousCol[sIndex] = 0x0000;
		sKeyLow2HighCol[sIndex] = 0x0000;
	}
	while (1)
	{
		//if the timer went off every 1 second, update the clock tick
		if (flags & TMR_UP)
		        {
					//everu time the timer is up, print out the timer to terminal and clear the flag again
					print_time();
					flags &= ~TMR_UP;
		        }

		//the scanning happens every 10 ms which is very fast so that no presses get skipped over
		HAL_Delay(10);
		//call the scanning function
		Keypadscan();
		//if the key is detected and is a valid, debounced, then print
		if ((sKeyStatus & KeyDetect) && (sKeyIssued != 0xFFFF))
		{
			//check which key was issued
			switch (sKeyIssued)
			{

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

			default:
			break;
			}
		}
		else if ((sKeyStatus & KeyHigh2Low) && (sKeyReleased != 0xFFFF))
		{
				break;
		}
		//clear the key status at the end
		sKeyStatus &= ~(KeyDetect | KeyLow2High | KeyHigh2Low | KeyIsOn |
		KeyIsOff);

		//clear the low to high and high to low arrays
		for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
		{
			sKeyLow2HighCol[sIndex] = 0x0000;
			sKeyHigh2LowCol[sIndex] = 0x0000;
		}
	}
}

void Keypadscan()
	{
	unsigned short sIndex;
	unsigned short Temp;
	//go through and clear the arrays before scanning or before each time you scan
	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
	{
		sKeyCurrentCol[sIndex] = 0x00;
	}
	//for every key, go through and clear those 3 columns
	for (sIndex=0; sIndex<Number_of_Keys; sIndex++)
	{
		//this has to be done with a bitwise operation rather than setting it equal to zero
		//we cannot assume that any of the other bits are not being used so can't zero them out, have to mask
		//this basically clears the output register for GPIOA
		GPIOA->ODR &=~(PA4 | PA1 | PA0);
		//for whatever key we are on, turn on that bit of its corresponding sending pin
		//do that by ORing it
		GPIOA->ODR |= sKeyControl[sIndex].sKeySend;
		HAL_Delay(10);
		//one two and three are all the keys which have a GPIOA pin to read from so these are grouped together
		switch (sKeyControl[sIndex].sKeyCommand)
		{
			case ONE_command:
			case TWO_command:
			case THREE_command:

			//if we are scanning any of these keys, then check the input register to see if they were pressed
			//read the IDR and AND it with the register it is read from
			//this compares that one specific bit to see if it is equal or not
			//if the one we are scanning is the one in the input register, that is the key that was pressed
			//go through and set the current key to be the bit position of it in the keypad
			if (GPIOA->IDR & sKeyControl[sIndex].sKeyRead){
				sKeyCurrentCol[sKeyControl[sIndex].sKeyCol]=sKeyControl[sIndex].sKeyReadTempPos;
				break;
			}

			//these are the pins that get connected to GPIOC so these get grouped together
			case FOUR_command:
			case FIVE_command:
			case SIX_command:
			case SEVEN_command:
			case EIGHT_command:
			case NINE_command:

			//same thing, but this time for GPIOC
			if (GPIOC->IDR & sKeyControl[sIndex].sKeyRead)
				sKeyCurrentCol[sKeyControl[sIndex].sKeyCol] = sKeyControl[sIndex].sKeyReadTempPos;
				break;

			//last group that is connected to pin GPIOB
			case STAR_command:
			case ZERO_command:
			case POUND_command:

			//does the same thing but for GPIOB
			if (GPIOB->IDR & sKeyControl[sIndex].sKeyRead)
				sKeyCurrentCol[sKeyControl[sIndex].sKeyCol] = sKeyControl[sIndex].sKeyReadTempPos;
		}
	}

	//this part is what checks for the rising edge and the falling edge
	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
	{
		//if the current key is the one that was pressed before and is a steady signal and is not zero
		if ((sKeyCurrentCol[sIndex] == sKeyDebouncedCol[sIndex]) && (sKeyCurrentCol[sIndex] != 0x0000)){
			sKeyStatus |= KeyIsOn;
			break;
		}
		if ((sKeyCurrentCol[sIndex] == sKeyDebouncedCol[sIndex]) && (sKeyCurrentCol[sIndex] == 0x0000) && (sKeyPreviousCol[sIndex]!= 0x0000))
		{
			sKeyStatus |= KeyIsOff;
			break;
		}
	}

	if (sIndex <Number_of_Cols)
	{
		//go through the number of columns
		for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
		{
			//XOR to see if the values have changed
			Temp = sKeyCurrentCol[sIndex] ^ sKeyPreviousCol[sIndex];
			//if they have changed, then if the current is high, and the value changed
			//that means it went from low to high so its a valid rising edge
			//store that it was a rising edge in the low to high array
			sKeyLow2HighCol[sIndex] = (sKeyCurrentCol[sIndex] & Temp);
			sKeyHigh2LowCol[sIndex] = (sKeyPreviousCol[sIndex] & Temp);
		}

		//if they key that was just pressed is the same as the low to high, then it is a valid press
		// Find which key is JUST depressed (Low To High) is JUST released
		for (sIndex=0 ; sIndex<Number_of_Keys; sIndex++)
		{
			//if the key status is valid and the key was on, and if the key went low to high
			if (sKeyStatus & KeyIsOn)
			{
				if (sKeyLow2HighCol[sKeyControl[sIndex].sKeyCol] &
					sKeyControl[sIndex].sKeyReadTempPos)
				{
					//the key issued was the key pressed
					//update the key status
					sKeyIssued = sKeyControl[sIndex].sKeyCommand;
					sKeyStatus |= (KeyDetect | KeyLow2High);
					break;
				}
				else
					sKeyIssued = 0xFFFF;
			}
			//if the key was off and if it was the falling edge
			else if (sKeyStatus & KeyIsOff)
			{
				if (sKeyHigh2LowCol[sKeyControl[sIndex].sKeyCol] &
						sKeyControl[sIndex].sKeyReadTempPos)
				{
					//mark that key as just been released
					sKeyReleased = sKeyControl[sIndex].sKeyCommand;
					//update the key status again
					sKeyStatus |= (KeyDetect | KeyHigh2Low);
					break;
				}
				else
					sKeyReleased = 0xFFFF;
			}
		}
	}
	else
	{
		//clear the key status at the end
		sKeyStatus &= ~(KeyDetect | KeyLow2High | KeyHigh2Low | KeyIsOn | KeyIsOff);
	}
	//go through and update the debounced like before
	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
	{
		if (sKeyCurrentCol[sIndex] == sKeyDebouncedCol[sIndex])
	{
	//update the previous
	sKeyPreviousCol[sIndex] = sKeyCurrentCol[sIndex];
	}
	}
	//move the current reading to the debouced
	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
	{
		sKeyDebouncedCol[sIndex] = sKeyCurrentCol[sIndex];
		sKeyLow2HighCol[sIndex] = 0;
		sKeyHigh2LowCol[sIndex] = 0;
	}
}

//this function takes in a coordinate and writes the ANSI sequence so that it translates to a terminal position
//terminal positioning is done
//0x1B[row;colH is the formatting for these special terminal commands
//0x1B[?25l like this hides the cursor
//0x1B[?25h and this shows the cursor
char *cursor_pos(int row, int col)
{
	static char CurPos[10];
	sprintf(CurPos, "\x1B[%d;%dH", row, col);
	return CurPos;
}

//will always print out the time on the screen
//assuming we are using military time, it will only overflow if over 24
void print_time() {
    char time_buffer[20];
    char period[3];

    if (currentTime.hr < 12){
    	strcpy(period, "AM");
    }
    else{
    	strcpy(period, "PM");
    }

    int hour12 = currentTime.hr;
    //if it hits zero we know we reached midnight
    if(hour12 == 0){
    	hour12 = 12;
    }
    //if the current hour is greater than 12, we have reached PM time and subtract we from that
    else if (hour12 > 12){
    	//take away 12 and that is the PM time
    	hour12 = hour12 - 12;
    }

    sprintf(time_buffer, "%02d:%02d:%02d %s", hour12, currentTime.min, currentTime.sec, period);

    //go back to the start of the line and overwrite it, do not go to the next line
    UART_send(&huart2, "\r");
    UART_send(&huart2, time_buffer);
}


//this gets called every time the timer overflows
//when the timer goes over 1 second, update the seconds part of the clock to the next second
//when that count reaches 59, update the minutes, and when the minutes goes above 59, increment the hours

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 TickCount++;
    if (TickCount == 1000) {  //if it hits 1 second
        TickCount = 0;
        currentTime.sec++;
        //if the seconds reaches a min, set the seconds to zero and update the minutes
        if (currentTime.sec == 60) {
            currentTime.sec = 0;
            currentTime.min++;
            if (currentTime.min == 60) {
                currentTime.min= 0;
                currentTime.hr++;
                if (currentTime.hr == 12) {
                	//if we are at 12pm, the timer is up
                	flags |= TMR_UP;
                }
                if (currentTime.hr == 13){
                	//move to the PM time
                	currentTime.hr = 1;
                }
            }
        }
        flags |= TMR_UP;  //set the flag to let them know the timer has run out
    }
}

//these functions use the UART to write to MOBAXTERM for what key was pressed
// if 1 is pressed, needs to be able to update the hours
void key7()
{
	currentTime.hr++;
	if(currentTime.hr == 24) {
		currentTime.hr = 0;
	}
	print_time();
}

//go up in mins
void key8()
{
	currentTime.min++;
	if(currentTime.min == 60) {
		currentTime.hr++;
		currentTime.min = 0;
	}
	print_time();
}

//go up in seconds
void key9()
{
	currentTime.sec++;
	if(currentTime.sec == 60) {
		currentTime.min++;
		currentTime.sec = 0;
	}
	print_time();
}

//decrement seconds
void key4()
{
	//if the seconds are at 0, and the mins are at zero, then overflow and go back to 23 hours
    if (currentTime.sec == 0)
    {
        if (currentTime.min == 0)
        {
            if (currentTime.hr == 0)
            {
                currentTime.hr = 23;
            }

            else
            {
                currentTime.hr--;
            }
            currentTime.min = 59;
        }
        else
        {
            currentTime.min--;
        }
        currentTime.sec = 59;
    }
    else
    {
        currentTime.sec--;
    }
    print_time();
}

void key5()//go down in mins
{
	//if mins was already at zero, then need to check if the hours are zero
    if (currentTime.min == 0)
    {
    	//if hours are also zero, then go back to previous day, so 23 hours 59 mins
        if (currentTime.hr == 0)
        {
            currentTime.hr = 23;
        }
        //else if the hour is not at zero, just go down to the last hour 59 mins
        else
        {
            currentTime.hr--;
        }
        //make it 59 mins
        currentTime.min = 59;
    }
    //else if the mins wasn't at zero, just go down one min
    else
    {
        currentTime.min--;
    }
    //print out the time
    print_time();
}

void key6()//go down in hours
{
	//same thing as above, if zero, go back to the previous day
    if (currentTime.hr == 0)
    {
        currentTime.hr = 23;
    }
    else
    {
        currentTime.hr--;
    }
    print_time();
}


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
