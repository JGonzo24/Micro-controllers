#include "main.h"
#include "TIMER.h"
#include "LCD.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define Number_of_Keys 12
#define Number_of_Cols  3
#define PA0 0x0001
#define PA1 0x0002
#define PA4 0x0010
#define PB0 0x0001
#define PC1 0x0002
#define PC0 0x0001
#define PA10 0x0400

#define KeyDetect 			0x0001
#define KeyLow2High 		0x0002
#define KeyRepeat			0x0004  // after a key is pressed & held for one second
#define KeyToBeRepeated 	0x0008

#define Key1Pushed			0x0001
#define Key2Pushed			0x0002
#define Key3Pushed			0x0004
#define Key4Pushed			0x0008
#define Key5Pushed			0x0010
#define Key6Pushed			0x0020
#define Key7Pushed			0x0040
#define Key8Pushed			0x0080
#define Key9Pushed			0x0100
#define Key0Pushed			0x0201

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

void key1R(void);
void key2R(void);
void key3R(void);
void key4R(void);
void key5R(void);
void key6R(void);
void key7R(void);
void key8R(void);
void key9R(void);
void key0R(void);
void keySR(void);
void keyPR(void);

/******** Structure *******/
Key_Contorl_struct_t sKeyControl[Number_of_Keys]
={
 {PA10,0x8,PA4,0,'1',ONE_command},     // PA10 (read), PA4 (send)
 {PC0,0x4,PA4,0,'4',FOUR_command},     // PC0 (read), PA4 (send)
 {PC1,0x2,PA4,0,'7',SEVEN_command},    // PC1 (read), PA4 (send)
 {PB0,0x1,PA4,0,'*',STAR_command},     // PB0 (read), PA4 (send)

 {PA10,0x8,PA1,1,'2',TWO_command},     // PA10 (read), PA1 (send)
 {PC0,0x4,PA1,1,'5',FIVE_command},     // PC0 (read), PA1 (send)
 {PC1,0x2,PA1,1,'8',EIGHT_command},    // PC1 (read), PA1 (send)
 {PB0,0x1,PA1,1,'0',ZERO_command},     // PB0 (read), PA1 (send)

 {PA10,0x8,PA0,2,'3',THREE_command},   // PA10 (read), PA0 (send)
 {PC0,0x4,PA0,2,'6',SIX_command},      // PC0 (read), PA0 (send)
 {PC1,0x2,PA0,2,'9',NINE_command},     // PC1 (read), PA0 (send)
 {PB0,0x1,PA0,2,'#',POUND_command}     // PB0 (read), PA0 (send)
};

unsigned short sKeyStatus;
unsigned short sKeyCurrentCol[Number_of_Cols];
unsigned short sKeyDebouncedCol[Number_of_Cols];
unsigned short sKeyIssued;
unsigned short sIndexCopy;
unsigned short sKeyPushedRecord;

unsigned short sKeyPreviousCol[Number_of_Cols];
unsigned short sKeyLow2HighCol[Number_of_Cols];

static char LCD_Num;
char Txt[1];

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;


void Keypadscan(void);
void KeyProcess(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartLineReader(void);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIMER2_HANDLE();
}

#define MAX_PHONE     5
#define PHONE_LEN    11

char  phoneNumber[MAX_PHONE][PHONE_LEN];
int   phoneIndex;
uint8_t receivedChar;
uint8_t receivedBuffer[PHONE_LEN];
size_t bufferIndex;

void StartLineReader(void)
{
    phoneIndex  = 0;
    bufferIndex = 0;
    HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nEnter phone #1: ", 18, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    // 1) if Enter: finish line
    if (receivedChar == '\r' || receivedChar == '\n') {
        // echo newline
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

        // null-terminate at bufferIndex
        receivedBuffer[bufferIndex] = '\0';

        // copy into array (include NUL)
        strncpy(phoneNumber[phoneIndex], (char*)receivedBuffer, PHONE_LEN);
        phoneNumber[phoneIndex][PHONE_LEN-1] = '\0';

        phoneIndex++;

        // reset for next input
        bufferIndex = 0;

        // if we still need more, prompt and re-arm
        if (phoneIndex < MAX_PHONE) {
            char prompt[24];
            int n = snprintf(prompt, sizeof(prompt), "Enter phone #%d: ", phoneIndex+1);
            HAL_UART_Transmit(&huart2, (uint8_t*)prompt, n, HAL_MAX_DELAY);
            HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
        }
        // else: we’re done—don’t re-arm, and you can act on phoneNumber[0..4] in main()
        return;
    }

    // 2) Backspace
    if ((receivedChar == '\b' || receivedChar == 0x7F) && bufferIndex > 0) {
        bufferIndex--;
        HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
        return;
    }

    // 3) Regular character: echo + store (up to PHONE_LEN-1)
    HAL_UART_Transmit(&huart2, &receivedChar, 1, HAL_MAX_DELAY);
    if (bufferIndex < PHONE_LEN-1)
        receivedBuffer[bufferIndex++] = receivedChar;

    // re-arm for next byte
    HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
}

int main(void)
{
  unsigned short sIndex;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  HAL_TIM_Base_Start_IT(&htim2);  // Start timer 2 interrupt


  char* prompt = "\r\n Please enter 5 phone numbers:\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);
  StartLineReader();
  // Clear all debounced records, Previous, Low2High
  for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
  {
	  sKeyDebouncedCol[sIndex] = 0x0000;
	  sKeyPreviousCol[sIndex] = 0x0000;
	  sKeyLow2HighCol[sIndex] = 0x0000;
  }

  LcdInit();

  LcdWriteCmd(0x000C);  // CURSOR OFF
  while (1)
  {
    // Check if need to scan and process keys
	if (sTimer[KEY_SCAN_TIMER] == 0)
    {
      Keypadscan();
      KeyProcess();
 	  sTimer[KEY_SCAN_TIMER] = KEY_SCAN_TIME;
	}

  }
}



void Keypadscan()
{
    unsigned short sIndex;
    unsigned short Temp;

    // Clear all key records
    for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
    {
      sKeyCurrentCol[sIndex] = 0x00;
    }

    // Read all 3 column
    for (sIndex=0; sIndex<Number_of_Keys; sIndex++)
    {
      GPIOA->ODR &=~(PA4 | PA1 | PA0);
      GPIOA->ODR |= sKeyControl[sIndex].sKeySend;
      HAL_Delay(0.5);

      switch (sKeyControl[sIndex].sKeyCommand)
	  {
      	  case ONE_command:
      	  case TWO_command:
      	  case THREE_command:
      		if (GPIOA->IDR & sKeyControl[sIndex].sKeyRead)
      		  sKeyCurrentCol[sKeyControl[sIndex].sKeyCol]= sKeyControl[sIndex].sKeyReadTempPos;
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
    for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
    {
      if ((sKeyCurrentCol[sIndex] == sKeyDebouncedCol[sIndex]) && (sKeyCurrentCol[sIndex] != 0x0000))
        break;
    }

    if (sIndex <Number_of_Cols)
    {
    	// Check for push on/ push off (Low To High)
    	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
    	{
    		Temp = sKeyCurrentCol[sIndex] ^ sKeyPreviousCol[sIndex];
    		sKeyLow2HighCol[sIndex] = (sKeyCurrentCol[sIndex] & Temp);
    	}

    	// Update Previous records
    	for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
    	{
    	    sKeyPreviousCol[sIndex] = sKeyCurrentCol[sIndex];
    	}

      // Find which key is JUST depressed (Low To High) or KeyRepeat detected
       for (sIndex=0 ; sIndex<Number_of_Keys; sIndex++)
       {
         if (sKeyLow2HighCol[sKeyControl[sIndex].sKeyCol] & sKeyControl[sIndex].sKeyReadTempPos)
         {
           sKeyIssued = sKeyControl[sIndex].sKeyCommand;
           sKeyStatus |= (KeyDetect | KeyLow2High);
           sTimer[KEY_WAIT_REPEAT_TIMER] = KEY_WAIT_REPEAT_TIME;
           sKeyStatus |= KeyRepeat;		// a new key comes in, set the repeat flag
           sIndexCopy = sIndex;			// save a copy of sIndex for push & held use
           break;
         }
         else if ((sKeyStatus & KeyRepeat) && (sTimer[KEY_WAIT_REPEAT_TIMER]==0))
         {
           if (sTimer[KEY_REPEAT_TIMER] == 0)
           {
              sKeyIssued = sKeyControl[sIndexCopy].sKeyCommand;
              sKeyStatus |= (KeyDetect | KeyToBeRepeated);
              sTimer[KEY_REPEAT_TIMER] = KEY_REPEAT_TIME;
           }
         }
         else
         sKeyIssued = 0xFFFF;
       }
    }
    else
    {
      sKeyStatus &= ~(KeyDetect | KeyLow2High | KeyToBeRepeated | KeyRepeat);
      sTimer[KEY_REPEAT_TIMER] = 0;  // Reset repeat timer if no key

      for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
       	  sKeyPreviousCol[sIndex] = 0;
    }


    // Transfer Current reading to debounced reading
    for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
    {
      sKeyDebouncedCol[sIndex] = sKeyCurrentCol[sIndex];
      sKeyLow2HighCol[sIndex] = 0;
    }
}


void KeyProcess()
{
	uint16_t sIndex;

	if ((sKeyStatus & KeyDetect) && (sKeyIssued != 0xFFFF))
	{
		switch (sKeyIssued)
	    {
	    	case ONE_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key1();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key1R();

	    		break;
	    	}
	        case FOUR_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key4();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key4R();
	    		break;
	    	}

	        case SEVEN_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key7();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key7R();
	    		break;
	    	}

	        case STAR_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			keyS();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			keySR();
	    		break;
	    	}

	        case TWO_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key2();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key2R();
	    		break;
	    	}

	        case FIVE_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key5();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key5R();
	    		break;
	    	}

	        case EIGHT_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key8();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key8R();
	    		break;
	    	}

	        case ZERO_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key0();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key0R();
	    		break;
	    	}

	        case THREE_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key3();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key3R();
	    		break;
	    	}

	        case SIX_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key6();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key6R();
	    		break;
	    	}

	        case NINE_command:
	    	{
	    		if (sKeyStatus & KeyLow2High)
	    			key9();
	    		else if (sKeyStatus & KeyToBeRepeated)
	    			key9R();
	    		break;
	    	}

	        case POUND_command:
	        {
	        	if (sKeyStatus & KeyLow2High)
	        		keyP();
//	        	else if (sKeyStatus & KeyToBeRepeated)
//	        		keyPR();
	        	break;
	        }

            default:
            	break;
        }

		sKeyStatus &= ~(KeyDetect | KeyLow2High | KeyToBeRepeated);

		// Clear all Low-2-High and High-2-Low records
		for (sIndex=0; sIndex<Number_of_Cols; sIndex++)
		  sKeyLow2HighCol[sIndex] = 0x0000;
	}
}

void key1()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key1Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 1;
		sKeyPushedRecord = Key1Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key2()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key2Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 2;
		sKeyPushedRecord = Key2Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key3()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key3Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 3;
		sKeyPushedRecord = Key3Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key4()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key4Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 4;
		sKeyPushedRecord = Key4Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key5()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key5Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 5;
		sKeyPushedRecord = Key5Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key6()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key6Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 6;
		sKeyPushedRecord = Key6Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key7()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key7Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 7;
		sKeyPushedRecord = Key7Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key8()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key8Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 8;
		sKeyPushedRecord = Key8Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key9()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key9Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 9;
		sKeyPushedRecord = Key9Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void key0()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	if (sKeyPushedRecord & Key0Pushed)
	{
		LCD_Num++;
		LCD_Num %= 10;
	}
	else
	{
		LCD_Num = 0;
		sKeyPushedRecord = Key0Pushed;
	}
	LcdPutCh(LCD_Num+0x30);
}

void keyS()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num = 0x53;			// ASCII for 'S'
	LcdPutCh(LCD_Num);
}


static int displayIndex = 0;
void keyP()
{
	LcdClear();
	LcdGoto(0,0);  	 	 	// Column 0, row 1		// ASCII for 'P'

	if (displayIndex < MAX_PHONE)
	{
		LcdPutS("Phone #");
		LcdPutS(phoneNumber[displayIndex]);
		LcdPutS("\r\n");
		displayIndex++;
	}
	else
	{
		LcdClear();
		LcdGoto(1,0);
		LcdPutS("End of list\r\n");
	}
}

void key1R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key2R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key3R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key4R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key5R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key6R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key7R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key8R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key9R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void key0R()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	LCD_Num %= 10;
	itoa(LCD_Num,Txt,10);  		// Convert to string
	LcdPutCh(Txt[0]);
}

void keySR()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	if ((LCD_Num < 0x41) || (LCD_Num > 0x5A))
		LCD_Num = 0x41;
	LcdPutCh(LCD_Num);
}

void keyPR()
{
	LcdGoto(1,0);  	 	 	// Column 0, row 1
	LCD_Num++;
	if ((LCD_Num < 0x41) || (LCD_Num > 0x5A))
		LCD_Num = 0x41;
	LcdPutCh(LCD_Num);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA8
                           PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
