#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#ifdef _TIMER_C
   #define SCOPE
#else
   #define SCOPE extern
#endif

#define NUMBER_OF_TIMERS			1
#define TRAFFIC_STATE_TIMER			0

SCOPE unsigned short sTimer[NUMBER_OF_TIMERS];

SCOPE void TIMER2_HANDLE(void);

#undef SCOPE
#endif /* INC_TIMER_H_ */