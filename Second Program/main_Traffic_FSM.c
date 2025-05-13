#include "main.h"

#define IODIRA 0x00			// MCP23017 DIR address
#define IODIRB 0x01			// DIR portB register
#define MCP_GPIOA 0x12		// I/O port A data register
#define MCP_GPIOB 0x13		// I/O port B data register
#define MCP_GPPUA 0x0C		// I/O pull up (input use) register

#define NS_Green 0x01		// Green on North-South direction
#define NS_Yellow 0x02		// Yellow on North-South direction
#define NS_Red	0x04		// Red on North-South direction
#define EW_Green 0x08		// Green on East-West direction
#define EW_Yellow 0x10		// Yellow on East-West direction
#define EW_Red	0x20		// Red on East-West direction

char I2C_ADDRESS = 0x40;
static uint8_t SensorIn;

struct State {
	unsigned char Out;
	unsigned short Time;
	unsigned char Next[4];
};
typedef struct State STyp;

#define gN 0
#define wN 1
#define gE 2
#define wE 3


// Change the FSM for 3 sec green, 1 sec yellow, 4 seconds red, for N-S
// Change the FSM for E-W to 4 sec red, 3 sec green, and 1 sec yellow

// While the other direction is red, the other direction duration is green + yellow duration
// this gives the 4 seconds red for the other direction
STyp FSM[4] = {
		{(NS_Green|EW_Red),3000,{wN,wN,wN,wN}},	// gN -> wN
		{(NS_Yellow|EW_Red),1000,{gE,gE,gE,gE}}, // from wN -> gE
	    {(NS_Red|EW_Green),3000,{wE,wE,wE,wE}}, // from gE -> wE
		{(NS_Red|EW_Yellow),1000,{gN,gN,gN,gN}} // from wE -> gN
};

I2C_HandleTypeDef hi2c1;		// I2C instance handle

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void I2CSend(char , char);
void I2CSendOne(char);


//HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
//                                          uint16_t Size, uint32_t Timeout)

//HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
//	                                         uint16_t Size, uint32_t Timeout)

int main(void)
{
	unsigned char n = gN;
	unsigned char Input;

	HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    HAL_I2C_Init(&hi2c1);           // To start I2C
    I2CSend(IODIRB, 0x00);      	// Make IO port_B, bit 0 to 7 all output

    HAL_I2C_Init(&hi2c1);           // To start I2C
    I2CSend(MCP_GPPUA, 0x03);       // Make IO port_A, bit 0 and 1 pull-up

    HAL_I2C_Init(&hi2c1);           // To start I2C
    I2CSend(IODIRA, 0xFF);      	// Make IO port_A all inputs

    while (1)
    {
    	// with Input and state, output corresponding lights
        // Drive the lights for state n
    	I2CSend(MCP_GPIOB, FSM[n].Out);

        // Wait for the time in state n
    	HAL_Delay(FSM[n].Time);

        // Read the input from portA
    	I2CSendOne(MCP_GPIOA);		// send slave address and port_A register
    	HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDRESS | 0x01), &SensorIn, 1, 1000);	// read port A into SensorIn
    	Input = (~SensorIn & 0x03);

        // not from the input but from the FSM table
    	n = FSM[n].Next[0];
    }
}


void I2CSend(char port, char data)
{
    uint8_t Buf[2];

    Buf[0] = port;
    Buf[1] = data;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, Buf, 2, 1000);
}

void I2CSendOne(char port)
{
    uint8_t Buf[1];

    Buf[0] = port;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, Buf, 1, 1000);
}

