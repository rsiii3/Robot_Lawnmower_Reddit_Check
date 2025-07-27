#ifndef EEPROM_SETTINGS_H
#define EEPROM_SETTINGS_H

#include <Arduino.h>  // Ensures compatibility with Arduino functions
#include <EEPROM.h>   // Required for EEPROM read/write

// Function declarations
void Load_EEPROM_Saved_Data();
void Clear_EEPROM_Data();

// Any global variables or constants should be declared here
//extern int Alarm_1_Hour;
//extern int Alarm_1_Minute;
//extern int Alarm_1_ON;
//extern int Alarm_1_Action;

#endif // EEPROM_SETTINGS_H
