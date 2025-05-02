#ifndef CYCLE_CONTROL_H
#define CYCLE_CONTROL_H

#include <Arduino.h>
#include <EEPROM.h>

// Global variables
extern signed long resultCTime;
extern signed long startCTime;

// Timer structure
struct timerVar_t {
    signed long REF; // = (signed long)millis(); // 0 -origin
    signed long ACC; // = 0;
    int PRE;
    bool flagRun; // = 0;
};

// Function declarations
void RuntimeMeasuring();
bool TimerDelayOn(struct timerVar_t &T, int T_PRE);
bool TimerDelayOnRet(struct timerVar_t &T);
void TimerReset(struct timerVar_t &T);
 void MEGA_WDT();

// External variables (ensure they exist elsewhere)
extern bool wdt_enable_flagRun;
extern bool Mega_SW_restart_En;
extern int Mower_Running;

#endif // CYCLE_CONTROL_H
