#define _TIMER_C

#include "main.h"
#include "Timer.h"
#include "LCD.h"

// Declare external variables from main.c
extern TimeStruct countdownTimer;
extern volatile uint8_t updateDisplayFlag;
extern volatile uint8_t timerDoneFlag;
extern volatile uint8_t configModeActive;

void TIMER2_HANDLE(void) {
    extern volatile uint8_t timerRunning;

    static uint16_t msCounter = 0;
    unsigned short sIndex;

    for (sIndex = 0; sIndex < NUMBER_OF_TIMERS; sIndex++) {
        if (sTimer[sIndex] != 0)
            sTimer[sIndex]--;
    }

    if (timerRunning) {
        msCounter++;
        if (msCounter >= 1000) {
            msCounter = 0;

            if (countdownTimer.hours > 0 || countdownTimer.minutes > 0 || countdownTimer.seconds > 0) {
                if (countdownTimer.seconds > 0) {
                    countdownTimer.seconds--;
                } else if (countdownTimer.minutes > 0) {
                    countdownTimer.minutes--;
                    countdownTimer.seconds = 59;
                } else if (countdownTimer.hours > 0) {
                    countdownTimer.hours--;
                    countdownTimer.minutes = 59;
                    countdownTimer.seconds = 59;
                }
                updateDisplayFlag = 1;
            } else {
                // Timer has finished
                timerRunning = 0;
                configModeActive = 1;
                updateDisplayFlag = 1;
                timerDoneFlag = 1;
            }
        }
    }
}
