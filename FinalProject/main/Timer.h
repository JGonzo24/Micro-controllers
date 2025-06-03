#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#ifdef _TIMER_C
   #define SCOPE
#else
   #define SCOPE extern
#endif




typedef struct
{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} TimeStruct;



#define NUMBER_OF_TIMERS			1

#define KEY_SCAN_TIMER				0


#define KEY_SCAN_TIME				10


SCOPE unsigned short sTimer[NUMBER_OF_TIMERS];
extern volatile uint8_t configModeActive;
extern volatile uint8_t timerRunning;

SCOPE void TIMER2_HANDLE(void);

#undef SCOPE
#endif /* INC_TIMER_H_ */
