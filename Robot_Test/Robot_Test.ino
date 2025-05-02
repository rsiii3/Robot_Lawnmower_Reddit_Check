
// ReP_AL Lawn Mower & ReP_AL Aerator Arduino Code
// Please make sure you have installed all the library files to the Arduino libraries folder
// You will need to unzip all the libraries from the GitHuB site.
// Instructions on how to do this are available on my webpage
// www.repalmakershop.com

// Thanks to Radek Veselý for help with the code speed up, improved serial communication
// and additonal features like the webserver and e-stop button option.

// check Read_Serial1_Nano()
// check Running_Test_for_Boundary_Wire()

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// YOU NEED TO SET UP THE ROBOT TYPE / BOARD TYPE to ensure pins are correctly assigned

// Robot Type
# define ROBOT_MOWER
//# define ROBOT_AERATOR

// Uncomment the correct Board Type
#define BOARD_MEGA
//#define BOARD_DUE

// Uncomment if using an LCD & Keypad     leave commented out to activate TFT.
// Uncomment if using the stop button over interrupt pin
//#define STOP_BTN
#define NODELAY_BACKWARD      // Uncomment if using backward running without delay() function
//#define WEBSERVER             // Uncomment if using WebServer - show mower data on web page
//#define WDT                   // Uncomment if using hardware watchdog timer - if MEGA freeze then restart itself to avoid still running

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

//Libraries for Perimeter Wire Receiver
#include <Arduino.h>
#include <Wire.h>
#include "drivers.h"
#include "adcman.h"
#include "perimeter.h"
#include <mavlink.h>
#include <SoftwareSerial.h>

SoftwareSerial Pixhawk_Serial(A10, A11);  // RX, TX

#if defined(BOARD_MEGA)
#include <EEPROM.h>
#endif

#if defined(BOARD_DUE)
#include <DueFlashStorage.h>
DueFlashStorage dueFlashStorage;
#endif

#include "SerialCom_non_blocking.h"
SerialCom SerialCom1 (Serial1, 40);
SerialCom SerialCom2 (Serial2, 80);
SerialCom SerialCom3 (Serial3, 40);



//Libraries for Real Time Clock
#include <stdio.h>
#include <DS1302.h>
#define DS3231_I2C_ADDRESS 0x68
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}


#if defined(LCD_KEYPAD)
  //Libraries for ic2 Liquid Crystal
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Enter the i2C address here lcd(_x__  e.g. lcd(0x27
  bool LCD_Screen_Keypad_Menu = 1;
  bool TFT_Screen_Menu        = 0;
  #else

bool LCD_Screen_Keypad_Menu = 0;
bool TFT_Screen_Menu        = 1;
#endif

//Libraries for the Mowing Calendar Function
#include <TimeLib.h>
#include <TimeAlarms.h>
AlarmId id;

//Compass Setup
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass;

#if defined(WDT) and defined(BOARD_MEGA)
  #include <avr/wdt.h>
  bool wdt_enable_flagRun = false;
  bool Mega_SW_restart_En = false;
#endif // -(WDT)-

//Perimeter Wire Pins
#define pinPerimeterLeft 47        // perimeter pin normally A5
#define pinPerimeterRight A4       // leave open
#define pinLED LED_BUILTIN

//GPS Fence Signal Pin
#define GPS_Fence_Signal_Pin A7
#define GPS_Lock_Pin A6

//Real Time Clock Pins
const int kCePin   = 29;  // RST
const int kIoPin   = 30;  // DAT
const int kSclkPin = 31;  // CLK

DS1302 rtc(kCePin, kIoPin, kSclkPin);

//Sonar Setup for Front Sonars 1-3
#define echoPin3 34   //S3               
#define trigPin3 35
#define echoPin2 36   //S2 
#define trigPin2 37
#define echoPin1 38   //S1
#define trigPin1 39


//Bumper Microswitches
 #define Microswitch_1  46               // Define Pin 47 on the MEGA to detect the microswitch  (Bumper RH or Lift Top End Stop)
 #define Microswitch_2  47               // Define Pin 46 on the MEGA to detect the microswitch  (Bumper LH or Lift Top End Stop)


#if defined(LCD_KEYPAD)
//Membrane Switch
#define Start_Key 50 //connect wire 1 to pin 2
#define Plus_Key 51  //connect wire 2 to pin 3
#define Minus_Key 52 //connect wire 3 to pin 4
#define Stop_Key 53  //connect wire 4 to pin 5
#endif

#if defined(ROBOT_MOWER)
  //Mower Pin Setup for the wheel Motor Bridge Controller
  //Motor A
  #define ENAPin 13                // EN Pins need a digital pin with PWM
  #define IN1Pin 12                // IN Pins dont need digital PWM
  #define IN2Pin 11
  //Motor B
          #if defined(STOP_BTN)
          #define ENBPin 13                // EN Pins need a digital pin with PWM
          #define IN3Pin 12                // IN Pins dont need digital PWM
          #define IN4Pin 11
          bool Stop_Button_Activated = false;  // true when STOP_BTN activated
          #else
  #define ENBPin 2                // EN Pins need a digital pin with PWM
  #define IN3Pin 4                // IN Pins dont need digital PWM
  #define IN4Pin 3
  #endif
  //Motor Blades
  bool BladeMotorState = 0;
  #define RPWM 9
  //#define L_EN 9
  #define R_EN 10
  #define Relay_Blades_Brake_Resistor 25 // 1 = brake active
  #define StopBtn 2              // Emergency stop button - immediate stop using interrupt

  #define PIXHAWK_LH_PWM 11    // MEGA D11
  #define PIXHAWK_RH_PWM 12    // MEGA D12

  int Robot_Type                  = 1;

#endif



//Relay Switch
#define Relay_Motors 24         // be careful that you really use PIN24.  The order is sometimes labelled
                                // so it looks like 24 is actually 22.

//Compass Level
#define X 3
#define Y 7
#define Z 5

// Tilt Sensors
#define Tilt_Angle A8           // measures the angle of the mower
#define Tilt_Orientation A9     // measures if the mower is upside down



//Global Variables

  //Perimeter Variables
  Perimeter perimeter;
  unsigned long nextTime = 0;
  int counter = 0;
  boolean inside = true;
  int Wire_Detected;

  int Loop_Cycle_Mowing = 0;             // was byte before.

  int Empty = 0;
  int Data1;
  int Data2;
  int Data3;
  int Data4;
  int Data5;
  int Data6;
  int Data7;
  
  //Sonar Variables
  long duration1 = 0;                     // Time required for the sonar ping to be recieved by the echo pin.
  long duration2 = 0;
  long duration3 = 0;

  int distance1 = 999;                    // Distance caculated  by the Sonar
  int distance2 = 999;
  int distance3 = 999;

  int distance_blockage;     

  int Sonar_Hit_1_Total;
  int Sonar_Hit_2_Total;
  int Sonar_Hit_3_Total;
  bool Sonar_Hit_Any_Total;
  bool Sonar_Hit_1 = 0;
  bool Sonar_Hit_2 = 0;
  bool Sonar_Hit_3 = 0;
  bool Sonar_Hit   = 0;
  bool Skip_Sonar_Turn = 0;

  int Sonar_1_Error;
  int Sonar_2_Error;
  int Sonar_3_Error;  

//  // Bumper Variables
//  bool Bump_Frnt_LH;
//  bool Bump_Frnt_RH;
//  bool Bumper;

  //Mower Status Variables
  bool Mower_Docked;
  bool Mower_Parked;
  int  Mower_Running;
  bool Mower_Parked_Low_Batt;
  int  Mower_Error;
  bool Manual_Mode;
  bool Mower_Setup_Mode;
  int  Robot_Status_Value;
  int  Mower_Error_Value;
  bool Exiting_Dock;

  bool Mower_PIXHAWK;
  bool PIXHAWK_Armed;
  int PIXHAWK_Mode;
  
  int Mower_RunBack; // when mower is go backward - at "1" due to Wire/GPS/Wheel Amp,  at "2" due to Sonar
  //bool Mower_RunBackSonar; // at "1" when mower is go backward
#if defined(NODELAY_BACKWARD)
  int Manouver_Turn_Around_Phase = 0; // used in function Manouver_Turn_Around() to control its flow
  int Manouver_Turn_Around_Sonar_Phase = 0; // used in function Manouver_Turn_Around_sonar() to control its flow
#endif

  // Mower Running Data
  int Sonar_Status;
  //int Data_Sent_Wire;       // Counter
//  int Bumper_Status;
  int Loops;
  int Compass_Steering_Status;
  int GYRO_Steering_Status;

  // Aerator Running Data
  int Drill_Status;


  //Membrane Key Variables
  byte  Start_Key_X;
  byte  Plus_Key_X;
  byte  Minus_Key_X; 
  byte  Stop_Key_X;  bool  Menu_Complete_Alarms;

  bool  Menu_Complete_Settings;
  bool  Menu_Complete_Sensors;
  bool  Menu_Complete_Motion;
  bool  Menu_Complete_NAVI;
  bool  Menu_Complete_Tracking;  
  bool  Menu_Complete;
  bool  Menu_NodeMCU_Complete;
  byte  Menu_Mode_Selection;
  int   Menu_View;
  int   Mow_Time_Set;
  int   Max_Options_Timing;
  int   Max_Options_Docked;
  int   Max_Options_Parked;
  int   Max_Options_Settings;
  int   Max_Options_Test;
  int   Max_Options_Alarms;
  int   Max_Options_Sensors;
  int   Max_Options_Motion;
  int   Max_Options_Tracking;
  int   Max_Options_NAVI;
  int   Max_Options_BETA;

  //Serial Communication
  float Volts;
  int   VoltsTX;
  float Volts_Last;
  int   Zero_Volts;
  float Amps;
  float VoltageAmp;
  int   RawValueAmp;
  int   RawValueVolt;
  int   Rain_Detected;
  int   Rain_Hit_Detected = 0;
  int   Charging;
  int WheelAmpsTX; // RVES added
  int Compass_Heading_DegreesTX; // RVES added

  //Simulation
  int   Fake_All_Settings;
  int   Fake_Loops;
  int   Fake_Wire;
  bool  Fake_WheelAmp;
 

  
  //float Battery_Voltage_Last;
  float Amps_Last;
  int   Volts_Outside_Reading;
  byte  OK_Nano_Data_Volt_Received; 
  byte  OK_Nano_Data_Charge_Received;
  byte  Charge_Hits = 0;
  byte  Docked_Hits = 0;
  bool  Charge_Detected_MEGA = 0;
  int   VoltsTX_Last;
  
  //Mow Calendar Variables
  byte Alarm_Hour_Now;
  byte Time_Second;
  byte Time_Minute;
  byte Time_Hour;
  byte Time_Date;
  byte Time_Month;
  int Time_Year;
  byte Time_Day;
  bool Alarm_Timed_Mow_ON = 0;
  byte Alarm_Timed_Mow_Hour;                         // Mowing Hour Number 3
  byte Alarm_Timed_Mow_Minute;                       // Alarm minute 3

  int Alarm_1_Saved_EEPROM;
  int Alarm_2_Saved_EEPROM;
  int Alarm_3_Saved_EEPROM;

  String dayAsString(const Time::Day day) {
    switch (day) {
      case Time::kSunday: return "Sunday";
      case Time::kMonday: return "Monday";
      case Time::kTuesday: return "Tuesday";
      case Time::kWednesday: return "Wednesday";
      case Time::kThursday: return "Thursday";
      case Time::kFriday: return "Friday";
      case Time::kSaturday: return "Saturday";
      }
  return "(unknown day)";
  }

  //Perimeter Wire Tracking
  int I; 
  int Track_Wire_Itterations;
  bool Outside_Wire;
  byte Exit_Zone;
  int MAG_Now;
  int MAG_OUT_Stop;
  int MAG_IN_Stop;
  int MAG_TURN;
  int MAG_Average_Start;
  int MAG_Last;
  byte Outside_Wire_Count = 0;
  int Tracking_Wire = 0;
  bool Wire_ON_Printed;
  int Wire_Off;
  int Wire_Refind_Tries = 0;

  int Tracking_Turn_Left;
  int Tracking_Turn_Right;
  bool Mower_Track_To_Charge;
  bool Mower_Track_To_Exit;

  bool Abort_Wire_Find;
  bool No_Wire_Found_Fwd;
  bool No_Wire_Found_Bck;
  int  Wire_Find_Attempt = 0; 

  int  PWM_Right;
  int  PWM_Left;
  int  MAG_Goal;
  int  MAG_Error;
  int  MAG_Start;
  int  Full_Speed_Achieved = 0;
  byte PWM_Blade_Speed_Min;
  byte PWM_Blade_Speed_Max;
  bool Blade_Override = 0;
  bool Blade_flagRun = false;

  //Compass Variables
  float   Compass_Heading_Degrees;
  float   Heading;
  int     Home_Error;
  bool    Compass_Heading_Locked = 0;
  float   Heading_Lock;
  int     Heading_Upper_Limit_Compass;
  int     Heading_Lower_Limit_Compass;
  float   Compass_Target;
  int     Compass_Leg = 0;
  int     Turn_Adjust = 0;
  int     error = 0;
  float   Calb_XAxis;
  float   Calb_YAxis;
  float   Calb_ZAxis;
  int     Tilt_X;
  int     Tilt_Y;
  int     Tilt_Z;
  float   X_Tilt;
  float   Y_Tilt;
  float   Z_Tilt;
  int     Compass_Detected;
  Vector  norm;

  //GY-521 GYRO replace with MPU-6050
  bool    GYRO_Heading_Locked = 0;
  int     GYRO_Angle;
  const int MPU_addr=0x69;
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ, Temp;
  int GYRO_Angle_X;
  int GYRO_Angle_Y;
  int GYRO_Angle_Z;

  char tmp_str[7]; // temporary variable used in convert function

  // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  char* convert_int16_to_str(int16_t i) { 
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
  }

  int minVal=265;
  int maxVal=402;

  // Pattern Mow
  int Spiral_Mow = 1;
  int Linking_Section;
  int Leg = 1;
  float Compass_Last;
  bool Wire_Found;  

  // GPS
  bool GPS_Inside_Fence = 1;
  int  GPS_Fence_Signal;
  int  GPS_Lock_Signal;
  bool GPS_Lock_OK;

  // PIXHAWK
  float PWM_Arduino_LH;
  float PWM_Arduino_RH;
  int PIXHAWK_PWM_Value_LH = 0;
  int PIXHAWK_PWM_Value_RH = 0;
  int cycles;

  int Pixhawk_Motor_Test_Initiate = 0;
  bool Calibration_Done  = 0;
  int Motor_C_PWM_Max             = 1472;                       // Max PWM Value of Motor C in Test
  int Motor_C_PWM_Min             = 1169;                       // Min PWM Value of Motor C in Test
  int Motor_D_PWM_Max             = 1472;                       // Max PWM Value of Motor D in Test
  int Motor_D_PWM_Min             = 1169;                       // Min PWM Value of Motor D in Test
  
  volatile int prev_time = 0;

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

// Mavlink_msg_command_long_pack() function
uint8_t   Target_System         = 1; 
uint8_t   Target_Component      = 0;
uint16_t  CMD_LONG_command      = 0;
uint8_t   CMD_LONG_confirmation = 0;
float     CMD_LONG_param1       = 0;
float     CMD_LONG_param2       = 0;
float     CMD_LONG_param3       = 0;
float     CMD_LONG_param4       = 0;
float     CMD_LONG_param5       = 0;
float     CMD_LONG_param6       = 0;
float     CMD_LONG_param7       = 0;


int       System_ID          = 1;                          ///< ID 20 for this airplane. 1 PX, 255 ground station
int       Component_ID       = 0;  
int       _GCSCOMPID         = 158;                        ///< The component sending the message
int       Type               = MAV_TYPE_GROUND_ROVER;      ///< This system is an airplane / fixed wing
uint8_t   System_Type        = MAV_TYPE_GENERIC;
uint8_t   Autopilot_Type     = MAV_AUTOPILOT_INVALID;
uint8_t   System_Mode        = MAV_MODE_PREFLIGHT;         ///< Booting up
uint32_t  Custom_Mode        = 0;                          ///< Custom mode, can be defined by user/adopter
uint8_t   System_State       = MAV_STATE_STANDBY;          ///< System ready for flight

uint8_t Base_Mode = 1;

 
// Utility Functions
#define ToDeg(x) ( (x) * 57.2957795131)   // * 180 pi
#define ToRad(x) ( (x) * 0.01745329252)

  //Wire Track Printer
  int PrintInMax;
  int PrintInMid;
  int PrintInMin;
  int PrintOutMin;
  int PrintOutMid;
  int PrintOutMax;
  int PrintMAG_Now;

  // Tilt Sensor
  int Tilt_Angle_Sensed = 1;          // reads the Tilt Angle sensor
  int Tilt_Orientation_Sensed;    // reads the Tilt Orientation sensor

  //WIFI Variables
  float val_WIFI;

  // TFT
  int  TFT_Menu_Command; 
  bool Menu_Complete_TFT;
  bool Mower_Is_Docking;
  bool Turn_To_Home;
  bool Find_Wire_Track;
  bool Go_To_Charging_Station;

  // Wheel Amp Sensors
  int    RawWheelAmp;
  int    Wheel_Blocked;
  float  WheelAmps;
  int    Wheel_Blocked_Count;
  bool   Wheel_Blocked_Status; // NODELAY_BACKWARD
  byte   Wheel_Status_Value;   // to send wheel state to NodeMCU


  // Drill Amp Sensors
  int    RawDrillAmp;
  int    Drill_Blocked;
  float  DrillAmps;
  int    Drill_Blocked_Count;

  // Endstop variables
  bool End_Stop_Lower;
  bool End_Stop_Upper;
  bool End_Stop;

  // Drill Values
  bool Drill_ON;
  int  Next_Drill_Target;
  int  max_cycles = 200;
  bool Stop = 0;

  // Serial Command Input
  int Command;
  

 
/***********************************************************************************************

                   SETUP OF MOWER / Aerator

  The following setup parameters will setup the mower for your garden
  Turn on or off the settings to defien how you like the mower to behave.
  
  Settings marked with EEPROM can be adjusted using the mower LCD menu.  Once changes and saved
  the EEPROM settings will override the settings in this menu.  
  
  To clear these settings you need to clear the EEPROM
    
  1 = Turned ON      0 = Turned OFF       Value = Value set for variable.

****************************************************************************************************/

  char Version[16] = "V9.751";

  bool PCB                        = 1;                          // USE Printed Circuit Board Relay

  bool Cutting_Blades_Activate    = 1;     // EEPROM            // Activates the cutting blades and disc in the code
  bool WIFI_Enabled               = 1;     // EEPROM            // Activates the WIFI Fucntions
  bool Perimeter_Wire_Enabled     = 1;     // EEPROM            // Activates use of the perimeter boundary wire
  
  // GPS Settings
  bool GPS_Enabled                 = 0;     // EEPROM            // Activates the GPS Fence Module
  int  GPS_Type                    = 2;     // EEPROM            // 1 = ReP_AL Fence   2 = PIXHAWK   3 = Spare
  bool Run_PIXHAWK_Mission_At_Exit = 0;                          // After Exiting the dock switch to PIXHAWK Auto control



  //Docking Station
  bool Use_Charging_Station       = 1;      //EEPROM            // 1 if you are using the docking/charging station     0 if not
  bool CW_Tracking_To_Charge      = 1;      //EEPROM            // Clock-Wise         tracking around the boundary wire to the charging station
  bool CCW_Tracking_To_Charge     = 0;      //EEPROM            // Counter-Clock-Wise tracking around the boundary wire to the charging station
  bool CW_Tracking_To_Start       = 0;      //EEPROM            // Clock-Wise         tracking around the boundary wire when tracking to the start position
  bool CCW_Tracking_To_Start      = 1;      //EEPORM            // Counter-Clock-Wise tracking around the boundary wire to the charging station
  byte Docked_Filter_Hits         = 1;                          // Number of charge signals to be detected before mower powers off


  // Wire Tracking
  int Track_Wire_Zone_1_Cycles    = 1300;   //EEPROM            // Zone 1 - Number of Itterations the PID function does before the mower exits the wire track
  int Track_Wire_Zone_2_Cycles    = 2200;   //EEPROM            // Zone 2 - Therefore how long the mower is tracking the wire can be set = distance tracked.
  int Max_Tracking_Turn_Right     = 270;    //EEPROM            // The maximum number of turn right commands during wire tracking before a renewed wire find function is called (wheel spins)
  int Max_Tracking_Turn_Left      = 270;    //EEPROM            // a re-find the wire sub-routine is called if this value is reached.
  int Max_Cycle_Wire_Find         = 300;    //EEPROM            // Maximum number of forward tracking cycles in finding wire before the mower restarts a compass turn and wire find.
  int Max_Cycle_Wire_Find_Back    = 50;     //EEPROM            // Maximum number of Backward tracking cycles in finding wire before the mower restarts a compass turn and wire find.  

  //Compass Settings
  int  Compass_Setup_Mode             = 1;                      // 1 to use DFRobot Library   2 to use Manual access code.  3 MechaQMC Library
  bool Compass_Activate               = 1;       //EEPROM       // Turns on the Compass (needs to be 1 to activate further compass features)
  bool Compass_Heading_Hold_Enabled   = 1;       //EEPROM       // Activates the compass heading hold function to keep the mower straight
  int  Home_Wire_Compass_Heading      = 110;     //EEPROM       // Heading the Mower will search for the wire once the mowing is completed.
  float CPower                        = 2;       //EEPROM       // Magnification of heading to PWM - How strong the mower corrects itself in Compass Mowing

  // GYRO Settings
  bool GYRO_Enabled                   = 0;      // EEPROM       // Enable the GYRO - Automatically activates the GYRO heading hold
  float GPower                        = 3;      // EEPROM       // Magnification of heading to PWM - How strong the mower corrects itself in Compass Mowing
  
  // Pattern Mow
  int  Pattern_Mow                = 0;       //EEPROM       // 0 = OFF |  1 = Parallel  | 2 = Sprials | 3 = Wire Mow
    
  // Pattern Spiral
  long int  Max_Cycles_Spirals    = 50000;      // Overrides the Max_Cycles for straight line mowing as the spirals requires more loops to complete
  float     Compass_Mow_Direction = 110;      // Mow Direction of line when pattern mow is activated

  // Pattern Parallel
  int Turn_90_Delay_LH        = 1150;      // EEPROM          // adjust this number so the mower turns 90° Left
  int Turn_90_Delay_RH        = 1250;      // EEPROM          // adjust this number so the mower turns 90° Right
  int Move_to_next_line_delay = 1000;      // distance between lines
  int Line_Length_Cycles      = 25;        // length of the line mowed

    // Wire Perimeter Mow
    


  // Safety Tilt Feature
  int  Angle_Sensor_Enabled           = 0;      //EEPROM       // Prvents Mower from climibing steep hills
  int  Tip_Over_Sensor_Enabled        = 0;      //EEPROM       // Turns mower off if turned over

  //Rain sensor 
  bool Rain_Sensor_Installed          = 1;  //EEPROM            // 1  = Rain sensor installed    0 = no sensor installed.
  int  Rain_Total_Hits_Go_Home        = 15; //EEPROM            // This sensor only makes sense in combination with a mower docking station
                                                                // as the mower is sent there to get out of the rain.
  //Battery Settings
  float Battery_Max               = 12.6;                       // Max battery volts in Volts. 3S = 12.6V
  float Battery_Min               = 11.3;   //EEPROM            // Lower Limit of battery charge before re-charge required.
  byte  Low_Battery_Detected      = 0;                          // Always set to 0
  byte  Low_Battery_Instances_Chg = 14;     //EEPROM            // Instances of low battery detected before a re-charge is called..
  float Amps_Charging_Setting     = 0.5;                        // Set value at which charge is detected in Amps (normally between 0.5 and 0.8)  0.5 = my 330/LAM   0.8 = my LM
  int   Volts_R2_Value            = 7300;                       // Set R2 vaklue in mower settings (usually between 6000 and 7500)  7100 = my 330   7500 = my LM   7000 = my LAM

  //Sonar Modules
  bool Sonar_1_Activate           = 1;      //EEPROM            // Activate (1) Deactivate (0) Sonar 1
  bool Sonar_2_Activate           = 1;      //EEPROM            // Activate (1) Deactivate (0) Sonar 2
  bool Sonar_3_Activate           = 1;      //EEPROM            // Activate (1) Deactivate (0) Sonar 3
  int  Max_Sonar_Hit              = 30;     //EEPROM            // Maximum number of Sonar hits before object is discovered
  long maxdistancesonar           = 20;     //EEPROM            // distance in cm from the mower that the sonar will activate at.
  int Sonar_Max_Error_Shutdown    = 10;

  // Bumper Module
  bool Bumper_Activate_Frnt       = 0;      //EEPROM            // Activates the bumper bar on the front facia - defualt is off.  Enable in the LCD settings menu.

  //Wheel Motors Setup
  bool Ramp_Motor_ON             = 1;
  
  int Max_Cycles_Straight        = 1250;    //EEPROM            // Number of loops the Sketch will run before the mower just turns around anyway. Adjust according to your garden length
  int PWM_MaxSpeed_LH            = 255;     //EEPROM            // Straight line speed LH Wheel (Looking from back of mower)  Will be overidden if saved in EEPROM
  int PWM_MaxSpeed_RH            = 255;     //EEPROM            // Straight line speed RH Wheel - adjust to keep mower tracking straight.  Will be overridden if saved in EEPROM
  bool Wheels_Activate           = 1;       //EEPROM            // Wheels ON/OFF - for testing

  bool MAG_Speed_Adjustment      = 0;   //** Experimental
  int Slow_Speed_MAG             = -900;                        // MAG Value that Slow Speed Kicks In
  int PWM_Slow_Speed_LH          = 160;                         // Straight line speed when the mower is almost over the wire Left Wheel.
  int PWM_Slow_Speed_RH          = 175;                         // Straight line speed when the mower is almost over the wire Right Wheel.
  
  int Mower_Turn_Delay_Min       = 1000;    //EEPROM            // Min Max Turn time of the Mower after it reverses at the wire. 1000 = 1 second
  int Mower_Turn_Delay_Max       = 2500;    //EEPROM            // A random turn time between these numbers is selected by the software
  int Mower_Reverse_Delay        = 1800;    //EEPORM            // Time the mower reverses before making a turn.

  bool Wheel_Amp_Sensor_ON       = 1;                           // Measures the amps in the wheel motor to detect blocked wheels.
  float Max_Wheel_Amps           = 3.5;                         // Maximum amperage allowed in the wheels before a blockage is called.
  int  Wheel_Blocked_Count_Max   = 3;                           // Number of times the wheels blocked are sensed before a reverse action takes place.

      

  //Blade Motor Setup
  //Blade Speed can be modified in the settings menu and will be written to EEPROM
  //The number below will then be overidden
  int PWM_Blade_Speed            = 150;     //EEPROM           // PWM signal sent to the blade motor (speed of blade) new motor works well at 245.

  // Alarm Setup
  bool Set_Time                   = 0;       //EEPROM           // Turn to 1 to set time on RTC (Set time in Time tab Set_Time_On_RTC)  After setting time turn to 0 and reload sketch.
                                                                // Not needed if setting time in the TFT Menu



  // Aerator Drill Setup
  int   Drill_Spacing             = 20;                          // Cycle Loops between each drill cycle
  bool  Drill_Amp_Sensor_ON       = 1;                           // Measures the amps in the drill motor to detect blocked wheels.
  float Max_Drill_Amps            = 5.5;                         // Maximum amperage allowed in the Drill before a blockage is called.
  int   Drill_Blocked_Count_Max   = 3;                           // Number of times the drill is blocked are sensed before a reverse action takes place.

  // Mecanum Wheel Setup
  int   Mecanum_Wheels_Installed  = 2;                           // 2 = 2 front wheel    4 = 2 Front & 2 Rear wheels (Allows sideways traverse)
                                                                 // Only needed for the AERator robot.

  // If the Alarm is changed in settings it will be written to EEPROM and the settings below will be overriden.
  // Action for Alarm 1 is set to exit the dock and mow at this time.
  // To change this action go to "void Activate_Alarms()" 
  bool Alarm_1_ON                 = 0;       //EEPROM            // Activate Alarm 1  (1 = ON 0 = OFF)
  int  Alarm_1_Hour               = 12;      //EEPROM            // Mowing Hour Number 1
  int  Alarm_1_Minute             = 00;      //EEPROM            // Alarm Minute 1
  bool Alarm_1_Repeat             = 0;                           // Repeat the Alarm at the same time
  int  Alarm_1_Action             = 1;       //EEPROM            // Sets the actions to be performed when the alarm is called

  // Action for Alarm 2 can be set in "void Activate_Alarms()" 
  bool Alarm_2_ON                 = 0;       //EEPROM            // Activate Alarm 2 (1 = ON 0 = OFF)
  int  Alarm_2_Hour               = 12;      //EEPROM            // Mowing Hour Number 2
  int  Alarm_2_Minute             = 00;      //EEPROM            // Alarm minute 2
  bool Alarm_2_Repeat             = 0;                           // Repeat the Alarm at the same time
  int  Alarm_2_Action             = 1;       //EEPROM            // Sets the actions to be performed when the alarm is called

  // Action for Alarm 3 can be set in "void Activate_Alarms()" 
  // Go Home Alarm
  bool Alarm_3_ON                 = 0;       //EEPROM            // Activate Alarm 3 (1 = ON 0 = OFF)
  int  Alarm_3_Hour               = 12;      //EEPROM            // Mowing Hour Number 3
  int  Alarm_3_Minute             = 00;      //EEPROM            // Alarm minute 3
  bool Alarm_3_Repeat             = 0;                           // Repeat the Alarm at the same time
  int  Alarm_3_Action             = 1;       //EEPROM            // Sets the actions to be performed when the alarm is called

  byte Alarm_Second               = 5;                            // Seconds


  /* Description of how the below values are displayed in the Serial Monitor Print Out for the wire
     function
     (InMax)                   Wire = 0                 (OutMax)
         |      (InMid)           |           (OutMid)     |
         |--------|--------|------|------|--------|--------|
         |        |        |      |      |        |        |
                        (InMin)       (OutMin)
  */

  // Wire detection Values
    /*Negative Values for In*/                                    // These values are based on the signal received by the wire sensor for my perimeter loop
    int InMin = -200;
    int InMid = -700;
    int InMax = -1500;                                            // the maximum received signal value  the wire
    /*General Setup PID numbers for wire tracking*/
    float P               = 0.08;              //EEPROM           // Multiplication factor to the error measured to the wire center.  if jerky movement when tracking reduce number
    float D               = 10;                                   // Dampening value to avoid the mower snaking on the wire.  
    byte Scale            = 36;                                   // Serial Monitor Line Tracking Print Scale

    // Gives the mower extra turning power at sharp turns in the wire during tracking
    bool Boost_Turn           = 1;                                // When the wheel PWM value is below this value the mower slows to make the curve better
                                                                  // For Planetary Motors = 100  220 Small Motors Use = 150
    int Min_Track_PWM         = 110;
    int Hard_Track_Turn_Delay = 10;

  
    // These values set the scale for the wire print out in the serial monitor once tracking
    int OutMin = 150;
    int OutMid = 400;
    int OutMax = 1500;                                            

    int Outside_Wire_Count_Max          = 15;                     // If the mower is outside the wire this many times the mower is stopped
    int Action_On_Over_Wire_Count_Max   = 3;                      // Set 1 to hibernate mower (Power Off and Stop)   Set 2 to refind garden using sonar and wire detect function
                                                                  // 3 to do a refind wire function

    bool Show_TX_Data                   = 0;                      // Show the values recieved from the Nano / ModeMCU in the serial monitor


/************************************************************************************************************/    
    // struct for timer function in general.ino file
    struct timerVar_t
    {
      signed long ACC = 0;
      signed long REF = (signed long)millis(); // 0 -origin
      int PRE;
      bool flagRun = 0;
    };
    timerVar_t T1, T2, T_MTA_ph2, T_MTA_ph4, T_MTA_ph5, T_MTA_ph7, T_MTAS_ph2, T_MTAS_ph4, T_MTAS_ph5, T_MTAS_ph7
  ;

    bool F_EN[33]; // Blocking bit for quick enable/ disable function

/************************************************************************************************************/


void setup() {
  Serial.begin(115200);
  Serial1.begin(57600);                                   // Open Serial port 1 for the nano communication
  SerialCom1.begin ();
  if (WIFI_Enabled == true) { 
    Serial2.begin(57600);         // If WIFI is on open Serial port 2 for the NodeMCU communication
    SerialCom2.begin ();
  }
  Wire.begin();                                           // start the i2c interface
  Serial.println(F(" "));
  Serial.println(F(" "));  
  if (Robot_Type == 1) Serial.print(F("ReP_AL Robot Lawn Mower :"));
//  if (Robot_Type == 2) Serial.print(F("ReP_AL Robot Aerator:"));
  Serial.println(Version);  
  Serial.println(F("==================="));
  Serial.println(F(""));
  Serial.println(F("Starting Mower Setup"));
  Serial.println(F("==================="));

  if (TFT_Screen_Menu == 1) {
    Serial3.begin(57600);
    SerialCom3.begin ();
    Serial.println(F("TFT Screen activated"));
  }
  /*if (LCD_Screen_Keypad_Menu == 1) {
    Serial3.begin(9600);          // 1200 before
    Serial.println(F("LCD Keypad activated"));
  }*/

  Alarm_1_ON = 0;
  Alarm_2_ON = 0;
  Alarm_3_ON = 0;
  Serial.println(F(""));

  //Clear_EERPOM();
  
//  #if defined(BOARD_DUE) 
//    Serial.println("BOARD = DUE");
//    Load_dueFlashStorage_Saved_Data();
//    #endif
 
  #if defined(BOARD_MEGA)
    Serial.println("BOARD = MEGA");
    Load_EEPROM_Saved_Data();
    #endif

  Serial.println(F(""));
  Serial.print(F("Volts R2 Value = "));
  Serial.print(Volts_R2_Value);
  Serial.print(F("  Charging Set to = "));
  Serial.print(Amps_Charging_Setting);
  Serial.println(F(" Amps"));
  
//  // If the Manual set time is switched on
//  if (Set_Time == 1 ) {
//    Serial.println(F("Setting Time"));
//    if (PCB == 0) Manual_Set_Time_On_DS1307();
//    if (PCB == 1) Manual_Set_Time_DS3231();
//    }
  if (PCB == 0) DisplayTime_DS1302();
//  if (PCB == 1) Display_DS3231_Time();
  
  Serial.println(F(""));
  Prepare_Mower_from_Settings();

  #if defined(LCD_KEYPAD)
  Setup_Run_LCD_Intro ();
  Setup_Membrane_Buttons();
  #endif  
  
  Setup_DFRobot_QMC5883_HMC5883L_Compass();                     // USes the DFRobot Library
  Setup_Gyro();
  delay(100);
  Setup_Relays();
  Setup_Tilt_Tip_Safety();
  Setup_Motor_Pins();
  Setup_ADCMan();
  Setup_Check_Pattern_Mow();
//  if (Bumper_Activate_Frnt == true) Setup_Microswitches();
  if ((GPS_Enabled == 1) && (GPS_Type == 2)) Setup_PIXHAWK();

  for(int i=0; i<33; i++) { F_EN[i] = true; } // for CT debug
  }

void loop() {

Get_Current_Time_Print_On_Serial_Monitor();
if (Mower_PIXHAWK == 0) Check_Serial_Input();
if (Mower_PIXHAWK == 1) Check_Serial_Input_PIXHAWK();
#if defined(STOP_BTN)
Stop_Button_Action();
#endif

if (TFT_Screen_Menu == 1)                                 Check_TFT_Serial_Input();   // Check the TFT Serial for any input command.
if ((TFT_Screen_Menu == 1) && (TFT_Menu_Command > 1))     Activate_TFT_Menu();        // If TFT Menu has requested an input, TX or RX that input.


// Read the Serial Ports for Data
Read_Serial1_Nano();                                                                  // Read the Serial data from the nano
Print_Mower_Status();                                                                 // Update the Serial monitor with the current mower status.


// Mower is docked, docking station is enabled and waiting for a command to leave and mow.
if ((Mower_Docked == 1) && (LCD_Screen_Keypad_Menu == 1))         Print_LCD_Volt_Info();                                  // Print the voltage to the LCD screen
if  (Mower_Docked == 1)                                           Check_if_Charging();
if (Mower_Docked == 1)                                            Check_if_Raining_From_Nano ();                          // Checks if the water sensor detects Rain
if ((Mower_Docked == 1) && (LCD_Screen_Keypad_Menu == 1))         Print_LCD_Info_Docked();                                // Print information to the LCD screen
if ((Mower_Docked == 1) && (LCD_Screen_Keypad_Menu == 1))         Print_Time_On_LCD();
if ((Mower_Docked == 1) && (LCD_Screen_Keypad_Menu == 1))         Check_Membrane_Switch_Input_Docked();                   // Check the membrane buttons for any input
if  (Mower_Docked == 1)                                           Running_Test_for_Boundary_Wire();                       // Test is the boundary wire is live
if  (Mower_Docked == 1)                                           Manouver_Dock_The_Mower();
if  (Mower_Docked == 1)                                           Display_Next_Alarm();
if  (Mower_Docked == 1)                                           Activate_Alarms();
if ((Mower_Docked == 1) && (TFT_Screen_Menu == 1))                Send_Mower_Docked_Data();                               // Send Data to TFT Display


// Mower is Parked ready to be started / re-started / or the mower has no docking station enabled.
if ((Mower_Parked == 1) && (LCD_Screen_Keypad_Menu == 1))         Print_LCD_Volt_Info();                                  // Print the voltage to the LCD screen
if (Mower_Parked == 1)                                            Check_if_Charging();
if (Mower_Parked == 1)                                            Check_if_Raining_From_Nano ();                          // Checks if the water sensor detects Rain
if ((Mower_Parked == 1) && (LCD_Screen_Keypad_Menu == 1))         Print_LCD_Info_Parked();                                // Print information to the LCD screen
if ((Mower_Parked == 1) && (LCD_Screen_Keypad_Menu == 1))         Check_Membrane_Switch_Input_Parked();                   // Check the membrane buttons for any input
if ((Mower_Parked == 1) && (GPS_Enabled == 1) && (GPS_Type == 1)) Check_GPS_In_Out();
if (Mower_Parked == 1)                                            Running_Test_for_Boundary_Wire();
if (Mower_Parked == 1)                                            Manouver_Park_The_Mower();
if ((Mower_Parked == 1) && (TFT_Screen_Menu == 1))                Send_Mower_Docked_Data();                               // Send Data to TFT Display

// Mower is in Setup Mode.
//if (Mower_Setup_Mode == 1)                                      Manouver_Setup_Mode();
if ((Mower_Setup_Mode == 1) && (TFT_Screen_Menu == 1))            Send_Mower_Setup_Data();                                // Display Setup on TFT Screen

// Mower is Parked with Low Battery needing Manual charging
if ((Mower_Parked_Low_Batt == 1) && (LCD_Screen_Keypad_Menu == 1)) Print_LCD_Volt_Info();                                  // Print the battery voltage
if ((Mower_Parked_Low_Batt == 1) && (LCD_Screen_Keypad_Menu == 1)) Print_Recharge_LCD();                                   // Print re-charge on the LCD screen
if ((Mower_Parked_Low_Batt == 1) && (LCD_Screen_Keypad_Menu == 1)) Check_Membrane_Switch_Input_Parked();

// Lost mower is put into standby mode
if ((Mower_Error == 1)  && (LCD_Screen_Keypad_Menu == 1))         Print_Mower_Error();                     // Safety mode incase the mower is lostor in an error state
if ((Mower_Error == 1)  && (LCD_Screen_Keypad_Menu == 1))         Check_Membrane_Switch_Input_Parked();
if ((Mower_Error == 1)  && (TFT_Screen_Menu == 1))                Send_Mower_Error_Data();                               // Send Data to TFT Display


// Mower is running cutting the grass.
if ((Mower_Running == 1) && (LCD_Screen_Keypad_Menu == 1))                                                                Print_LCD_Volt_Info();              // Print the voltage to the LCD screen
if  (Mower_Running == 1)                                                                                                  Process_Volt_Information();         // Take action based on the voltage readings
if  (Mower_Running == 1 && Mower_RunBack == 0)                                                                            Check_if_Raining_From_Nano();       // Test the rain sensor for rain. If detected sends the mower home
if ((Mower_Running == 1) && (LCD_Screen_Keypad_Menu == 1))                                                                Check_Membrane_Keys_Running();      // Check to see if the mower needs to be stopped via keypad
if  (Mower_Running == 1 && Mower_RunBack == 0)                                                                            Check_Timed_Mow();                  // Check to see if the time to go home has come.
if  (Mower_Running == 1 && Mower_RunBack == 0)                                                                            Running_Test_for_Boundary_Wire();   // Test is the boundary wire is live
if  (Mower_Running == 1)                                                                                                  Check_Tilt_Tip_Angle();             // Tests to see if the mower is overturned.
if ((Mower_Running == 1) && (Wheel_Amp_Sensor_ON == 1)/* && Mower_RunBack == 0*/)                                         Check_Wheel_Amps();                 // Tests to see if the wheels are blocked.
if ((Mower_Running == 1) && (Wire_Detected == 1) && Mower_RunBack == 0)                                                   Check_Wire_In_Out();                // Test if the mower is in or out of the wire fence.
if ((Mower_Running == 1) && (GPS_Enabled == 1) && (GPS_Type == 1) && Mower_RunBack == 0)                                  Check_GPS_In_Out();                 // Test is the GPS Fence has been crossed
if ((Mower_Running == 1) && (Wire_Detected == 1) && (Outside_Wire == 0) && Mower_RunBack == 0)                            Check_Sonar_Sensors();              // If the mower is  the boundary wire check the sonars for obsticles and prints to the LCD
if ((Mower_Running == 1) && (Wire_Detected == 1) && (Outside_Wire == 0) && (Sonar_Hit == 0) && Mower_RunBack == 0)        Manouver_Mow_Aerate_The_Grass();    // Inputs to the wheel motors / blade motors according to surroundings
// if ((Mower_Running == 1) && (Wire_Detected == 1) && (Outside_Wire == 0) && (Sonar_Hit == 0) && Mower_RunBack == 0)        Check_Bumper();                     // If the mower is  the boundary wire check the Bumper for activation

#if not defined(NODELAY_BACKWARD)
  if ((Mower_Running == 1) && (Wire_Detected == 1) && ((Outside_Wire == 1) // || (Bumper == 1)) && (Loop_Cycle_Mowing > 0))  Manouver_Turn_Around();             // If the bumper is activated or the mower is outside the boundary wire turn around
#else
  if ((Mower_Running==1)&&(Wire_Detected==1)&&((Outside_Wire==1) ||(Wheel_Blocked==4))&&(Loop_Cycle_Mowing>0)) Manouver_Turn_Around(); // ||(Bumper==1)  // If the bumper is activated or the mower is outside the boundary wire turn around
#endif // -(NODELAY_BACKWARD)-

if ((Mower_Running == 1) && (GPS_Enabled == 1) && (GPS_Type == 1) && (GPS_Inside_Fence == 0))                             Manouver_Turn_Around();             // If the GPS Fence is activated Turn Around
if ((Mower_Running == 1) && (Wire_Detected == 1) && (Outside_Wire == 0) && (Sonar_Hit == 1))                              Manouver_Turn_Around_Sonar();       // If sonar hit is detected and mower is  the wire, manouver around obsticle

#if defined(NODELAY_BACKWARD)
if  (Mower_Running == 1 && Mower_RunBack == 1)                                                                            Manouver_Turn_Around_non_blocking();
if  (Mower_Running == 1 && Mower_RunBack == 2)                                                                            Manouver_Turn_Around_Sonar_non_blocking();
#endif // -(NODELAY_BACKWARD)-

if (Mower_Running == 1)                                                                                                   Send_Mower_Running_Data();


//Mower in PIXHAWK Mode
if (Mower_PIXHAWK == 1)                                                                                                   Check_PIXHAWK();       
if ((Mower_PIXHAWK == 1) && (PIXHAWK_Armed == 1))                                                                         Check_PIXHAWK_PWM();              // Checks the motor PWM if the PIXHAWK is armed



// WIFI Commands from NodeMCU to and from the Blynk APP
if (Manual_Mode == 1)                             Receive_WIFI_Manual_Commands();
if (Manual_Mode == 1)                             Print_LCD_Info_Manual();
if (Manual_Mode == 0)                             Get_WIFI_Commands();                                   // TX and RX data from NodeMCU


RuntimeMeasuring(); // loop cycle time measuring

Serial.println();
//Check_Sonar_Sensors();
MEGA_WDT(); // watchdog timer function
}  // End Loop


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// AmpsVolts
// The voltage is transfered to the MEGA from the Nano via the serial com.
// The vlaues from the Nano are the raw sensor values.
// The Amps and volts are then calculated via equations


void Process_Volt_Information()   {
//  Logic for how the battery readings should be handled
    
    if (Volts < Battery_Min) {
      Low_Battery_Detected = (Low_Battery_Detected + 1);
      Serial.print(F("VLow:"));
      Serial.print(Low_Battery_Detected);
      Serial.print(F("|"));
      if (Low_Battery_Detected > Low_Battery_Instances_Chg) {
         Serial.println(F(""));
         Serial.println(F("Low Battery Detected"));
         if (Use_Charging_Station == 1) {
          // need to add a low batt code... to TFT
          Manouver_Go_To_Charging_Station();                       // Stops the mowing and sends the mower back to the charging station via the permieter wire
          }
         if (Use_Charging_Station == 0) Manouver_Park_The_Mower_Low_Batt();                      // Parks the mower with a low battery warning
         }
      }

    if (Volts >= Battery_Min) {
      Serial.print(F("VLow:"));
      Serial.print(Low_Battery_Detected);
      Serial.print(F("|"));
      Low_Battery_Detected = 0;
      }
}



// checks to see if the mower is on the charging station
void Check_if_Charging() {
    if (Charging == 4)  {                            // If the value recieved is equal to 1 or 0 as expected then print the value to the serial monitor
        Serial.print(F("Charging:"));
        Serial.print(Charging);
        Serial.print(F("|"));
        Charge_Detected_MEGA = 1;
        Print_Charging_LCD();
        Serial.print(F("MEGA = 1|"));
        }
    if (Charging == 0)  {                            // If the value recieved is equal to 1 or 0 as expected then print the value to the serial monitor
        Serial.print(F("Charging:"));
        Serial.print(Charging);
        Serial.print(F("|"));
        Charge_Detected_MEGA = 0;
        Print_Charging_LCD();
        }
      if ((Charging != 4) && (Charging !=0)) {
        Serial.print(F("Charging:"));
        Serial.print(Charging);
        Serial.print(F("|"));
        Charge_Detected_MEGA = 0;
        Print_Charging_LCD();
        }
}

void Check_if_Docked() {
  if (Charge_Detected_MEGA == 1) {                                    // if Amps are between this there is a charge detected.  Amps above 4 are discounted as a miscommunication
        Motor_Action_Stop_Motors();    
        Serial.println(F(""));
        Serial.println(F("Charging Current detected"));
        Serial.println(F(""));
        Serial.println(F("----------------"));
        Serial.println(F("| Mower Docked |"));
        Serial.println(F("----------------"));
        Serial.println(F(""));
        
        #if defined(LCD_KEYPAD)
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Docked in"));
        lcd.setCursor(0, 1);
        lcd.print(F("Charging Station"));                                // Prints info to LCD display 
        #endif
        
        delay(2000);
        Mower_Docked = 1;
        // Update the TFT Screen
        if (TFT_Screen_Menu == 1) {
            //Send_Mower_Docking_Data();
            Turn_To_Home = 0;
            Find_Wire_Track = 0;
            Go_To_Charging_Station = 0;
            }
        Manouver_Dock_The_Mower();                                    // Shuts down the Mower ready for charging and mowing again.
        
        #if defined(LCD_KEYPAD)
        lcd.clear();
        #endif
        
        }
   Serial.println(F(""));
}
        
void Calculate_Volt_Amp_Charge() {
  if (Show_TX_Data == 1) {
        Serial.print(F("Amp:"));
        Serial.print(RawValueAmp);
        Serial.print(F("|"));
        Serial.print(F("Volt:"));
        Serial.print(RawValueVolt);
        Serial.print(F("|"));
        Serial.print(F("Rain:"));
        Serial.print(Rain_Detected);
        Serial.print(F("|"));
        if (Wheel_Amp_Sensor_ON == 1) {
            Serial.print(F("WBlock:"));
            Serial.print(Wheel_Blocked);
            Serial.print(F("|"));
            }
        }

  // Calculate Amps from NANO RX Data
   int mVperAmp = 185;
   int ACSoffset = 2500; 
   double VoltageAmp = 0;
   double Amps_Now = 0;
   VoltageAmp = (RawValueAmp / 1024.0) * 5000; // Gets you mV
   Amps_Now =  ((VoltageAmp - ACSoffset) / mVperAmp);
   Amps = Amps_Now;
   Serial.print(F("A:"));    
   Serial.print(Amps);
   Serial.print(F("|"));

  // Calculate Voltage from NANO RX Data
  if (RawValueVolt > 100)  {
    float vout = 0.0;
    float R1 = 30000;                                   
 
    // Set R2 vaklue in mower settings (usually between 6000 and 7500)
    float R2 = Volts_R2_Value;         
    vout = (RawValueVolt * 5.0) / 1024.0; // see text
    Volts = vout / (R2/(R1+R2));
    
    // can be enbaled from the serial command for testing only
    Volts_Last = Volts;
    Zero_Volts = 0;
  }

  else {
    Volts = Volts_Last;
    Zero_Volts = Zero_Volts + 1;
    if (Zero_Volts > 5) Volts = 0; 
  }

  if (Fake_All_Settings == 1) Execute_Fake_It();

  Serial.print(F("V:"));    
  Serial.print(Volts);
  Serial.print(F("|"));

  if (Amps < Amps_Charging_Setting) Charging = 0;
  if (Amps >= Amps_Charging_Setting) Charging = 4;
  Serial.print("Charging = ");  
  Serial.print(Charging);
  Serial.print("|");
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Apply_Setup
// Functions to setup the mower and enable sensors
// Most of these functions are performed in the intial
// void setup phase of the sketch

void Print_Mower_Status() {
  if (Mower_PIXHAWK == 0) {
    //Serial.print("Mower Stat....");
    if (Mower_Docked == 1) Serial.print(F("Docked:1|"));
    if (Mower_Parked == 1) Serial.print(F("Parked:1|"));
    if (Mower_Running == 1) Serial.print(F("Running:1|"));
    if (Manual_Mode == 1) Serial.print(F("Manual Mode:1|"));
    if (Mower_Setup_Mode == 1) Serial.print(F("Setup Mode:1|"));
    if (Mower_Parked_Low_Batt == 1) Serial.print(F("Park_Low_Batt:1|"));
    if (Mower_Error == 1) Serial.print(F("Mower Error:1|"));
  }
}

void Setup_Tilt_Tip_Safety() {
  if (Angle_Sensor_Enabled == 1)     pinMode(Tilt_Angle, INPUT);//define Data input pin input pin
  if (Tip_Over_Sensor_Enabled == 1)  pinMode(Tilt_Orientation, INPUT);//define Data input pin input
}

void Prepare_Mower_from_Settings() {
  if (Use_Charging_Station == 1) {
     Mower_Docked  = 1;
     Mower_Parked  = 0;
     Mower_Running = 0;
  }
  if (Use_Charging_Station == 0) {
    Mower_Docked  = 0;
    Mower_Parked  = 1;
    Mower_Running = 0;
  }
}

void Setup_DFRobot_QMC5883_HMC5883L_Compass() {
  if (Compass_Activate == 1) {
    /*Setup Compass
    *************************************************************************/
    #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print(F("Compass  "));
      lcd.setCursor(0, 1);
      lcd.print(F("Setup"));
    #endif
  
    int Compass_Attempt = 0;
    int Compass_Found = 0;
  
    //HMC5883 Compass
    while ((!compass.begin()) && (Compass_Attempt <= 5)){
      Serial.println(F("No HMC or QMC5883 sensor found, check wiring!"));
      delay(500);
      Compass_Attempt = Compass_Attempt + 1;
    }
    if (compass.isHMC()) {
      Serial.println(F("Initialize DF Robot HMC5883 Compass"));
      Compass_Detected = 1;                        // HMC5883
      
      #if defined(LCD_KEYPAD)
      lcd.setCursor(6,0);
      lcd.print(F(": HMC5883"));
      delay(500);
      #endif
  
      //compass.setRange(HMC5883L_RANGE_1_3GA);
      //compass.setMeasurementMode(HMC5883L_CONTINOUS);
      //compass.setDataRate(HMC5883L_DATARATE_15HZ);
      //compass.setSamples(HMC5883L_SAMPLES_8);
      //Compass_Found = 1;
    }    
    //QMC5883 Compass
    else if (compass.isQMC()) {
      Serial.println(F("Initialising DF Robot QMC5883 Compass"));
      Compass_Detected = 2;                        // HMC5883
      
      #if defined(LCD_KEYPAD)
      lcd.setCursor(6,0);
      lcd.print(F(": QMC5883"));
      delay(500);
      #endif
      
      compass.setRange(QMC5883_RANGE_2GA);
      compass.setMeasurementMode(QMC5883_CONTINOUS); 
      compass.setDataRate(QMC5883_DATARATE_50HZ);
      compass.setSamples(QMC5883_SAMPLES_8);
    }  
    // Escape the loop if no compass is found but compass is activated in the settings
    if ((Compass_Attempt > 5) && (Compass_Found == 0)) {
      Serial.println("No Valid Compass Found");
      Compass_Activate = 0;
      Serial.println("Compass Deactivated");
      delay(3000);
    }
        
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1); 
    lcd.print(F("Done!             "));
    delay(500);
    lcd.clear();
    #endif
        
    //Wire.endTransmission(); 
  }       
  if (Compass_Activate == 0) {
    Serial.println(F("Compass Switched off - Select 1 in setup to switch on.")); 
  }    
}

void Setup_Gyro() {
  if (GYRO_Enabled == 1) {
    Serial.println("GYRO GY-521 Activated");
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);  
  }
}

void Setup_Relays() {
  Serial.println(F("Setup Relays"));
  pinMode(Relay_Motors, OUTPUT);
  delay(5);
  Turn_Off_Relay();
  delay(5);
}

void Setup_Motor_Pins() {
  Serial.println(F(""));
  Serial.print(F("Setup Motor Pins:"));
  
  #if defined(ROBOT_MOWER)
  Serial.println(F("Mower:"));
  //  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(Relay_Blades_Brake_Resistor, OUTPUT);
  pinMode(StopBtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(StopBtn), Stop_Button_Pressed, FALLING);

  pinMode(ENAPin, OUTPUT);
  pinMode(ENBPin, OUTPUT);
  pinMode(IN1Pin, OUTPUT);
  pinMode(IN2Pin, OUTPUT);
  pinMode(IN3Pin, OUTPUT);
  pinMode(IN4Pin, OUTPUT);

  pinMode(PIXHAWK_LH_PWM, INPUT);
  pinMode(PIXHAWK_RH_PWM, INPUT);
  #endif
}

void  Turn_On_Relay() {
  Serial.print(F("Relay:ON|"));
  if (PCB == 0) digitalWrite(Relay_Motors, HIGH);   // NOT Inverted                         // Turn on the relay for the 12V main battery power
  if (PCB == 1) digitalWrite(Relay_Motors, LOW);  // NOT Inverted 
}

void  Turn_Off_Relay() {
  Serial.print(F("Relay:Off|"));
  if (PCB == 0) (digitalWrite(Relay_Motors, HIGH));  // NOT Inverted                        // Turn off the relay for the 12V main battery power
  if (PCB == 1) (digitalWrite(Relay_Motors, LOW));  // NOT Inverted      
}

void Setup_Membrane_Buttons() {
  #if defined(LCD_KEYPAD)
  Serial.println(F("Setup Membrane Keys"));
  pinMode(Start_Key, INPUT_PULLUP);            // set pin as input
  pinMode(Plus_Key, INPUT_PULLUP);            // set pin as input
  pinMode(Minus_Key, INPUT_PULLUP);            // set pin as input
  pinMode(Stop_Key, INPUT_PULLUP);            // set pin as input  
  #endif
}

void Setup_Microswitches() {
  Serial.println(F("Setup Microswitches"));
  pinMode(Microswitch_1, INPUT_PULLUP); 
  pinMode(Microswitch_2, INPUT_PULLUP); 
}

void Setup_ADCMan() {
  Serial.println(F("Setting up ADCMAN"));
  if (Perimeter_Wire_Enabled == 1) {
    // Wire Sensor
    Serial.println(F("Setting up Wire Sensor"));
    Serial.println(F(""));
    ADCMan.init();
    ADCMan.setCapture(pinPerimeterLeft, 1, 0);
    perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);
    perimeter.useDifferentialPerimeterSignal = true; 
    perimeter.speedTest(); 
  }
  if ((GPS_Enabled == 1) && (GPS_Type == 2)) {
    // GPS Fence Sensor 
    Serial.println(F("Setting up PIXHAWK PWM Signals"));
    pinMode(PIXHAWK_LH_PWM, INPUT);
    pinMode(PIXHAWK_RH_PWM, INPUT);
  }
  ADCMan.run();
}

void Setup_Check_Pattern_Mow() {
  #if defined(LCD_KEYPAD)
    if (Pattern_Mow == 1) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Pattern Mow");
      lcd.setCursor(0,1);
      lcd.print("Parallel");
      delay(1000);
      lcd.clear();
      }
    if (Pattern_Mow == 2) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Pattern");
      lcd.setCursor(0,1);
      lcd.print("Spirals");
      delay(1000);
      lcd.clear();
      }
  #endif
}

void Setup_PIXHAWK() {
  Serial.println(F("PIXHAWK SETUP..."));
  Pixhawk_Serial.begin(57600);
  Serial.println("MAVLink starting.");  
  Check_PIXHAWK();         // Check PIXHAWK Packages
  Command_long_Disarm();   // Disarm Pixhawk
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Compass
// COMPASS FUNCTIONS
//*****************************************************************************

/* Calculates the compass heading as heading & degrees of the onboard compass */
void Get_Compass_Reading() {
  // If the Compass is activated
  if (Compass_Activate == 1) {
    // Displays a star on the LCD to show compass is being used.
    #if defined(LCD_KEYPAD)
      lcd.setCursor(7, 0);
      lcd.print("*");
    #endif
    
    Serial.print(F("Comp°:"));
    Vector norm = compass.readNormalize();
    Heading = atan2(norm.YAxis, norm.XAxis);   
    // Set declination angle. Find your location declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative,
    float Declination_Angle = (2.0 + (19.0 / 60.0)) / (180 / PI);   // Bad Krozingen is 2° 19'
    Heading += Declination_Angle;
    if (Heading < 0) {                                              // Correct for heading < 0deg and heading > 360deg
      Heading += 2 * PI;
    }
    if (Heading > 2 * PI) {
      Heading -= 2 * PI;
    }
    Compass_Heading_Degrees = Heading * 180 / M_PI;                 // Convert to degrees
    Serial.print(Compass_Heading_Degrees);
    Serial.print(F("|"));
    //delay(5);
    // Clear * to show compass reading is OK and no compass crash          
    #if defined(LCD_KEYPAD)
      lcd.setCursor(7, 0);
      lcd.print(" ");
    #endif
    //delay(100);
  }
}

// Turns the Mower to the correct orientation for the optimum home wire track
// Avoiding tracking around the whole wire to get back to the docking station
void Compass_Turn_Mower_To_Home_Direction() {
  //Stop the motors.
  Motor_Action_Stop_Motors();
  delay(2000);
  Print_LCD_Compass_Home();
  delay(1000);
  Compass_Target = Home_Wire_Compass_Heading;
  
  #if defined(LCD_KEYPAD)
    lcd.clear();
  #endif
  
  // Reverse the mower a little
  SetPins_ToGoBackwards();
  Motor_Action_Go_Full_Speed();
  delay(800);
  Motor_Action_Stop_Motors();  
  Get_Compass_Reading();
  SetPins_ToTurnLeft(); 
  // This spins the mower a little to ensure a true compass reading is being read (calibration).
  SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
  Motor_Action_Turn_Speed();                                       // Sets the speed of the turning motion
  delay(500);    
  Motor_Action_Stop_Motors();
  Get_Compass_Reading();
  SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
  delay(100);
  Motor_Action_Turn_Speed();                                       // Sets the speed of the turning motion
  delay(2000);  
  Get_Compass_Reading();
  delay(500);
  
  #if defined(LCD_KEYPAD)
    lcd.clear();
    lcd.print("Compass Set");
  #endif

  Motor_Action_Stop_Motors();
  Get_Compass_Reading();
  delay(2000);
  Turn_To_Compass_Heading();                                  // Uses the compass to orientate the mower towards home.
}

void Turn_To_Compass_Heading() {
  bool Made_Turn = 0;
  bool Spin_Left = 0;
  bool Spin_Right = 0;
  Serial.println(F(""));
  Serial.println(F("- - - - - - - - -"));
  Serial.println(F("Compass Home 2 Activated"));
  delay(1000);
  
  // Print info to LCD
  #if defined(LCD_KEYPAD)
    lcd.setCursor(0,0);
    lcd.print(F("Target: "));
    lcd.print(Compass_Target);
  #endif
  
  // Print to Serial Monitor
  Serial.print(F("Compass Target : "));
  Serial.println(Compass_Target);   
  
  // Sets the boundaries for the error of compass accurcy required.
  Heading_Lower_Limit_Compass = Compass_Target - 5;
  Heading_Upper_Limit_Compass = Compass_Target + 5;
  
  int Attemps_Compass_Turn = 0;
  float Compass_Last = 0;
  int Bad_Reading = 0;
  
  // Sends the mower spinning towards the traget compass heading
  if (Compass_Target < 180) {
    Serial.println(F("Compass Target < 180 logic used"));
    float Limit = Compass_Target + 180;
    Serial.print(F("Left Turn Limit : "));
    Serial.print(Compass_Target);
    Serial.print(F(" to "));
    Serial.println(Limit);
    Serial.print(F("Right Turn Limit : "));
    Serial.print(Limit);
    Serial.print(F(" to 360° & 0 to "));
    Serial.println(Compass_Target);
    Get_Compass_Reading();
    delay(500);
    Get_Compass_Reading();
    delay(500);
    while ( ((Compass_Heading_Degrees < Heading_Lower_Limit_Compass) || (Compass_Heading_Degrees > Heading_Upper_Limit_Compass)) && ((Attemps_Compass_Turn < 40) && (Bad_Reading < 5)))  { 
      delay(200);
      Get_Compass_Reading();
      
      // Double check if the compass reading retunred is plausable.
      if (Attemps_Compass_Turn > 5) {
        if (Compass_Heading_Degrees - Compass_Last > 50) {
          Serial.print(F("Bad Compass Reading "));
          Serial.print(F("Compass Heading Degrees = "));
          Serial.print(Compass_Heading_Degrees);
          Serial.print(F("  Compass Last = "));
          Serial.print(Compass_Last);
          Serial.print(F("  Degrees - Last = "));
          Serial.println(Compass_Heading_Degrees - Compass_Last);
          
          Bad_Reading = Bad_Reading + 1;
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15,1);
          lcd.print(F("x"));
          #endif
          Get_Compass_Reading();
          delay(100);
        }
        else {
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15,1);
          lcd.print(F("o"));  
          #endif
        }
      }
      Compass_Last = Compass_Heading_Degrees;    // sotres the last good compass reading
      delay(100);
      Serial.print(F("Compass Heading : "));
      Serial.print(Compass_Heading_Degrees);
      Serial.print(F("|"));   
      if ( (Compass_Heading_Degrees  > Compass_Target) && (Compass_Heading_Degrees < Limit) )  {
        SetPins_ToTurnLeft(); 
        Serial.print(F("Spin Left"));
        Serial.print(F("|"));  
        delay(100);
        if (Made_Turn == 0) Motor_Action_Turn_Speed();
        Spin_Left = 1;
        if ((Spin_Right == 1) && (Made_Turn == 0)) {
          Made_Turn = 1;
          Motor_Action_Stop_Motors();
          delay(200);
          Spin_Right = 0;
        }
        if (Made_Turn == 1) {
          Motor_Action_Turn_Speed();
          delay(50);
          Motor_Action_Stop_Motors();          
        }   
      }
      if ( ((Compass_Heading_Degrees > Limit) && (Compass_Heading_Degrees < 360)) || ( (Compass_Heading_Degrees > 0) && (Compass_Heading_Degrees < Compass_Target)) ) {
        SetPins_ToTurnRight(); 
        Serial.print(F("Spin Right"));
        Serial.print(F("|")); 
        delay(100);           
        if (Made_Turn == 0) Motor_Action_Turn_Speed();
        Spin_Right = 1;
        if ((Spin_Left == 1) && (Made_Turn == 0)) {
          Made_Turn = 1;
          Motor_Action_Stop_Motors();
          delay(200);
          Spin_Left = 0;
        }
        if (Made_Turn == 1) {
          Motor_Action_Turn_Speed();
          delay(50);
          Motor_Action_Stop_Motors();          
        } 
      }
      Get_Compass_Reading();

      #if defined(LCD_KEYPAD)
        lcd.setCursor(0,1);
        lcd.print(Compass_Heading_Degrees);
      #endif

      Attemps_Compass_Turn = Attemps_Compass_Turn + 1;
      Serial.print(F("Atempts:"));
      Serial.print(Attemps_Compass_Turn);
      Serial.print(F("|"));
      Serial.println(F(""));
    }
  }

// Sends the compass spinning the shortest way to the compass goal
  if (Compass_Target >= 180) {
    Serial.println(F("Compass Target > 180 logic used"));
    float Limit = Compass_Target - 180;
    Serial.print(F("Left Turn Limit : "));
    Serial.print(Compass_Target);
    Serial.print((" to 360° & 0 to "));
    Serial.println(Limit);
    Serial.print(F("Right Turn Limit : "));
    Serial.print(Compass_Target);
    Serial.print(F(" to "));
    Serial.println(Limit);
    while ( ((Compass_Heading_Degrees < Heading_Lower_Limit_Compass) || (Compass_Heading_Degrees > Heading_Upper_Limit_Compass)) && ((Attemps_Compass_Turn < 20) && (Bad_Reading < 5)))  { 
      delay(200);
      Get_Compass_Reading();
      
      // Double check if the compass reading returned is plausable.
      if (Attemps_Compass_Turn > 5) {
        if (Compass_Heading_Degrees - Compass_Last > 50){
          Serial.println(F("Bad Compass Reading"));
          Bad_Reading = Bad_Reading + 1;
          
          #if defined(LCD_KEYPAD)
            lcd.setCursor(15,1);
            lcd.print(F("x"));
          #endif
            
          Get_Compass_Reading();
          delay(100);
        }
        else {
          #if defined(LCD_KEYPAD)
            lcd.setCursor(15,1);
            lcd.print("o"); 
          #endif 
        }
      }
      Compass_Last = Compass_Heading_Degrees;    // sotres the last good compass reading  
      delay(100);
      Serial.print(F("Compass Heading : "));
      Serial.print(Compass_Heading_Degrees);
      Serial.print(F("|"));
      if ( ((Compass_Heading_Degrees > Compass_Target) && (Compass_Heading_Degrees < 360)) || ( (Compass_Heading_Degrees > 0) && (Compass_Heading_Degrees < Limit)) ) {
        SetPins_ToTurnLeft(); 
        Serial.print(F("Spin Left"));
        Serial.print(F("|")); 
        if (Made_Turn == 0) Motor_Action_Turn_Speed();
        Spin_Left = 1;
        if ((Spin_Right == 1) && (Made_Turn == 0)) {
          Made_Turn = 1;
          Motor_Action_Stop_Motors();
          delay(200);
          Spin_Right = 0;
        }
        if (Made_Turn == 1) {
          Motor_Action_Turn_Speed();
          delay(50);
          Motor_Action_Stop_Motors();          
        }
      }
      if ( (Compass_Heading_Degrees  > Limit) && (Compass_Heading_Degrees < Compass_Target) )  {
        SetPins_ToTurnRight(); 
        Serial.print(F("Spin Right"));
        Serial.print(F("|"));  
        delay(100); 
        Spin_Right = 1;
        if (Made_Turn == 0) Motor_Action_Turn_Speed();
        if ((Spin_Left == 1) && (Made_Turn == 0)) {
          Made_Turn = 1;
          Motor_Action_Stop_Motors();
          delay(200);
          Spin_Left = 0;
        }
        if (Made_Turn == 1) {
          Motor_Action_Turn_Speed();
          delay(50);
          Motor_Action_Stop_Motors();          
        }
      }
      Get_Compass_Reading();
      Attemps_Compass_Turn = Attemps_Compass_Turn + 1;
      Serial.print("Atempts:");
      Serial.print(Attemps_Compass_Turn);
      Serial.print("|");
      Serial.println("");
    }   
  }   
  if (Bad_Reading > 5)  {
    #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print(F("Bad Compass Readings"));
      lcd.setCursor(0,1);
      lcd.print(F("Restarting"));
    #endif
    
    delay(1000);
    SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
    Motor_Action_Turn_Speed();                                       // Sets the speed of the turning motion
    delay(100);  
    Get_Compass_Reading();   
    SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
    Motor_Action_Turn_Speed();                                       // Sets the speed of the turning motion
    delay(100);  
    Get_Compass_Reading();   
    SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
    Motor_Action_Turn_Speed();                                       // Sets the speed of the turning motion
    delay(100);  
    Get_Compass_Reading();   
    Turn_To_Compass_Heading();
  }
}

void Display_Compass_Current_Heading_on_LCD() {
  #if defined(LCD_KEYPAD)
    lcd.setCursor(5, 1);
    lcd.print("    ");
    lcd.print(Compass_Heading_Degrees);
    delay(10);
  #endif
}

void Calculate_Compass_Wheel_Compensation() {
  float Compass_Error = Compass_Heading_Degrees - Heading_Lock;        // Calculates the error in compass heading from the saved lock heading
  if (Compass_Error > 180) Compass_Error = Compass_Error * - 1 ;
  if (Compass_Error < -180) Compass_Error = Compass_Error * - 1 ;
  Serial.print(F("C_Err:"));
  Serial.print(Compass_Error);
  Serial.print(F("|")); 
  if (Compass_Error < 0) {                                             // Steer left
    Serial.print(F("SR|"));    
    // With no adjustment of wheel speed according to MAG Level    
    if (MAG_Speed_Adjustment == 0) {
      PWM_Left = PWM_MaxSpeed_LH;         
      PWM_Right = PWM_MaxSpeed_RH + (CPower * Compass_Error);            // Multiply the difference by D to increase the power then subtract from the PWM
      if (PWM_Right < 0) PWM_Right = PWM_MaxSpeed_RH - 50;
    }
    if (MAG_Speed_Adjustment == 1) {
      // MAX MAG Speed Left Hand Wheel
      if (MAG_Now >= Slow_Speed_MAG)  {
        PWM_Left = PWM_MaxSpeed_LH;
        PWM_Right = PWM_MaxSpeed_RH + (CPower * Compass_Error);            // Multiply the difference by D to increase the power then subtract from the PWM
        if (PWM_Right < 0) PWM_Right = PWM_MaxSpeed_RH - 50; 
      }
      if (MAG_Now < Slow_Speed_MAG)   {
        PWM_Left = PWM_Slow_Speed_LH;
        PWM_Right = PWM_Slow_Speed_RH + ((CPower/2) * Compass_Error);            // Multiply the difference by D to increase the power then subtract from the PWM
        if (PWM_Right < 0) PWM_Right = PWM_Slow_Speed_RH - 20; 
      }
    }
    Compass_Steering_Status = 2;
  }  
  if (Compass_Error >= 0) {  
    Serial.print(F("SL|"));
    
    // With no adjustment of wheel speed according to MAG Level
    if (MAG_Speed_Adjustment == 0) {
      PWM_Right = PWM_MaxSpeed_RH; 
      PWM_Left = PWM_MaxSpeed_LH -  (CPower * Compass_Error);            // Multiply the difference by D to increase the power then subtract from the PWM
      if (PWM_Left < 0) PWM_Left = PWM_MaxSpeed_LH - 50;
    }
    if (MAG_Speed_Adjustment == 1) {
      // MAX MAG Speed Right Hand Wheel
      if (MAG_Now >= Slow_Speed_MAG)  {
        PWM_Right = PWM_MaxSpeed_RH; 
        PWM_Left = PWM_MaxSpeed_LH -  (CPower * Compass_Error);            // Multiply the difference by D to increase the power then subtract from the PWM
        if (PWM_Left < 0) PWM_Left = PWM_MaxSpeed_LH - 50;
      }
      if (MAG_Now < Slow_Speed_MAG)   {
        PWM_Right = PWM_Slow_Speed_RH;
        PWM_Left = PWM_Slow_Speed_LH -  ((CPower/2) * Compass_Error);            // Multiply the difference by D to increase the power then subtract from the PWM
        if (PWM_Left < 0) PWM_Left = PWM_Slow_Speed_LH - 20;
      }
    }
    Compass_Steering_Status = 3;
  }
}

void Turn_To_Compass_Heading_2() {

    // Simple Compass Spin Code

    float Error;
    int Spin = 0;
    int Max_Spin = 30;
    int Max_Error = 100;
    
    // Get current Compass Heading  
    Get_Compass_Reading();
    
    
    // Print to Serial Monitor
    Serial.println(F(""));
    Serial.println(F("Turning to Home Direction Phase 1 : "));
    Serial.println(F(""));
    
    Error = Compass_Heading_Degrees - Home_Wire_Compass_Heading;
    Print_Compass_Turn_Info();


   // For a Positive Error Value
   if (Error >= 0) {
    
   while ((Error > Max_Error) && (Spin < Max_Spin))  {
    

        Get_Compass_Reading();
        Error = Compass_Heading_Degrees - Home_Wire_Compass_Heading;
        Print_Compass_Turn_Info();
      
        SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
        Motor_Action_Turn_Speed();        
        Spin = Spin + 1;
        Serial.print(F("  Spin: "));
        Serial.println(Spin);  
        }

   }

   // For a Negative Error Value
   if (Error < 0) {
    
   while ((Error > (Max_Error * -1) ) && (Spin < Max_Spin))  {
    

        Get_Compass_Reading();
        Error = Compass_Heading_Degrees - Home_Wire_Compass_Heading;
        Print_Compass_Turn_Info();
      
        SetPins_ToTurnLeft();                                     // Calls the motor function turn Left
        Motor_Action_Turn_Speed();        
        Spin = Spin + 1;
        Serial.print(F("  Spin: "));
        Serial.println(Spin);  
        }

   }



    // Print to Serial Monitor
    Serial.println(F(""));
    Serial.println(F("Turning to Home Direction Phase 2 : "));
    Serial.println(F(""));
    
    Error = Compass_Heading_Degrees - Home_Wire_Compass_Heading;
    Print_Compass_Turn_Info();


     Max_Error = 20;
    Turn_Adjust = 100;

   // For a Positive Error Value
   if (Error >= 0) {
    
   while ((Error > Max_Error) && (Spin < Max_Spin))  {
    

        Get_Compass_Reading();
        Error = Compass_Heading_Degrees - Home_Wire_Compass_Heading;
        Print_Compass_Turn_Info();
      
        SetPins_ToTurnRight();     

        // Calls the motor function turn Left
        Motor_Action_Turn_Speed();        
        Spin = Spin + 1;
        Serial.print(F("  Spin: "));
        Serial.println(Spin);  
        }

   }

   // For a Negative Error Value
   if (Error < 0) {
    
   while ((Error > (Max_Error * -1)) && (Spin < Max_Spin))  {
    

        Get_Compass_Reading();
        Error = Compass_Heading_Degrees - Home_Wire_Compass_Heading;
        Print_Compass_Turn_Info();
      
        SetPins_ToTurnRight();                                     // Calls the motor function turn Left
        Motor_Action_Turn_Speed();        
        Spin = Spin + 1;
        Serial.print(F("  Spin: "));
        Serial.println(Spin);  
        }

   }

Turn_Adjust = 0;

}

void Print_Compass_Turn_Info() {
        Serial.print(F("Compass Home Target : "));
        Serial.print(Compass_Target);
        Serial.print(F("  Compass Heading : "));
        Serial.print(Compass_Heading_Degrees);        
        Serial.print(F("    Error : "));
        Serial.print(Home_Error);   
        }
        

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Due_Flash
// Experimental - This is ready for a potential future swith to a 
// 32bit Arduino due which will improve the loop speed

// Loads the saved values from dueFlashStorage
// to the settings menuu

void Load_dueFlashStorage_Saved_Data() {

#if defined(BOARD_DUE)

Serial.println("Loading dueFlashStorage Settings");

  int Robot_Type_dueFlashStorage = dueFlashStorage.read(121);
  if (Robot_Type_dueFlashStorage == 1) {
    Robot_Type = dueFlashStorage.read(122); 
    Serial.print(F("FLASH: Robot_Type: "));
    if (Robot_Type == 1) Serial.println(F("FLASH: Robot Type = Mower"));
//    if (Robot_Type == 2) Serial.println(F("FLASH: Robot Type = Aerator"));
  }

  if (Robot_Type_dueFlashStorage != 1) {
    if (Robot_Type == 1) Serial.println(F("Settings: Robot Type = Mower"));
//    if (Robot_Type == 2) Serial.println(F("Settings: Robot Type = Aerator"));
  }


  int PCB_dueFlashStorage = dueFlashStorage.read(119);
  if (PCB_dueFlashStorage == 1) {
    PCB = dueFlashStorage.read(120); 
    Serial.print(F("FLASH: PCB Enabled: "));
      if (PCB == 1) Serial.println(F("Enabled"));
      if (PCB == 0) Serial.println(F("Disabled"));
      }

  if (PCB_dueFlashStorage != 1) {
    Serial.print(F("Settings: PCB Enabled: "));
      if (PCB == 1) Serial.println(F("Enabled"));
      if (PCB == 0) Serial.println(F("Disabled"));
      }

 
  int Alarm_1_Saved_dueFlashStorage = dueFlashStorage.read(1);
  
  if (Alarm_1_Saved_dueFlashStorage == 1) {
    Alarm_1_Hour    = dueFlashStorage.read(2);
    Alarm_1_Minute  = dueFlashStorage.read(3);
    Alarm_1_ON      = dueFlashStorage.read(4);
    Alarm_1_Action  = dueFlashStorage.read(87);

    if (Alarm_1_Hour == 255)   {
      Alarm_1_Hour = 0;
      Alarm_1_ON = 0;
      }
    if (Alarm_1_Minute == 255) {
      Alarm_1_Minute = 0;
      Alarm_1_ON  = 0;
      }
    
    if (Alarm_1_ON == 0)  Serial.println("Alarm 1 : OFF");
 
    if (Alarm_1_ON == 1) {
      Serial.print(F("dueFlashStorage Alarm 1 Active | Time Set:"));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      if (Alarm_1_Minute < 10) Serial.print("0");
      Serial.print(Alarm_1_Minute);
      Serial.print(" | Action :");
      if (Alarm_1_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_1_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_1_Action == 3) Serial.println("Mow the Line");
      if (Alarm_1_Action == 4) Serial.println("Quick Start");
      if (Alarm_1_Action == 5) Serial.println("Custom");
      }
    }
  if (Alarm_1_Saved_dueFlashStorage != 1) {
    if (Alarm_1_ON == 1) {
      Serial.print(F("Settings: Alarm 1 Active | Time Set:"));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      if (Alarm_1_Minute < 10) Serial.print("0");
      Serial.print(Alarm_1_Minute);
      Serial.print(" | Action :");
      if (Alarm_1_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_1_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_1_Action == 3) Serial.println("Mow the Line");
      if (Alarm_1_Action == 4) Serial.println("Quick Start");
      if (Alarm_1_Action == 5) Serial.println("Custom");
      }
  }
    
  int Alarm_2_Saved_dueFlashStorage = dueFlashStorage.read(5);
  
  if (Alarm_2_Saved_dueFlashStorage == 1) {
    Alarm_2_Hour    = dueFlashStorage.read(6);
    Alarm_2_Minute  = dueFlashStorage.read(7);
    Alarm_2_ON      = dueFlashStorage.read(8);
    Alarm_2_Action  = dueFlashStorage.read(88);
    
    if (Alarm_2_Hour == 255)   {
      Alarm_2_Hour = 0;
      Alarm_2_ON = 0;
      }
    if (Alarm_2_Minute == 255) {
      Alarm_2_Minute = 0;
      Alarm_2_ON  = 0;
      }
    
    if (Alarm_2_ON == 0) Serial.println("Alarm 2 : OFF:");
    
    if (Alarm_2_ON == 1) {
      Serial.print(F("Alarm 2 Active | Time Set:"));
      Serial.print(Alarm_2_Hour);
      Serial.print(F(":"));
      if (Alarm_2_Minute < 10) Serial.print("0");
      Serial.print(Alarm_2_Minute);  
      Serial.print(" | Action :");
      if (Alarm_2_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_2_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_2_Action == 3) Serial.println("Mow the Line");
      if (Alarm_2_Action == 4) Serial.println("Quick Start");
      if (Alarm_2_Action == 5) Serial.println("Custom");
      }

    }

  if (Alarm_2_Saved_dueFlashStorage != 1) {
    if (Alarm_2_ON == 1) {
      Serial.print(F("Settings: Alarm 2 Active | Time Set:"));
      Serial.print(Alarm_2_Hour);
      Serial.print(F(":"));
      if (Alarm_2_Minute < 10) Serial.print("0");
      Serial.print(Alarm_2_Minute);
      Serial.print(" | Action :");
      if (Alarm_2_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_2_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_2_Action == 3) Serial.println("Mow the Line");
      if (Alarm_2_Action == 4) Serial.println("Quick Start");
      if (Alarm_2_Action == 5) Serial.println("Custom");
      }
  }


  int Alarm_3_Saved_dueFlashStorage = dueFlashStorage.read(9);
  
  if (Alarm_3_Saved_dueFlashStorage == 1) {
    Alarm_3_Hour    = dueFlashStorage.read(10);
    Alarm_3_Minute  = dueFlashStorage.read(11);
    Alarm_3_ON      = dueFlashStorage.read(12);
    Alarm_3_Action  = dueFlashStorage.read(89);

    if (Alarm_3_Hour == 255)   {
      Alarm_3_Hour = 0;
      Alarm_3_ON = 0;
      }
    if (Alarm_3_Minute == 255) {
      Alarm_3_Minute = 0;
      Alarm_3_ON  = 0;
      }
    
    
    if (Alarm_3_ON == 0) Serial.println("Alarm 3 : OFF: ");
   
    if (Alarm_3_ON == 1) {
      Serial.print(F("Alarm 3 Active | Time Set:"));
      Serial.print(Alarm_3_Hour);
      Serial.print(F(":"));
      if (Alarm_3_Minute < 10) Serial.print("0");
      Serial.print(Alarm_3_Minute);
      Serial.print(" | Action :");
      if (Alarm_3_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_3_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_3_Action == 3) Serial.println("Mow the Line");
      if (Alarm_3_Action == 4) Serial.println("Quick Start");
      if (Alarm_3_Action == 5) Serial.println("Custom");  
      }

    }

  if (Alarm_3_Saved_dueFlashStorage != 1) {
    if (Alarm_3_ON == 1) {
      Serial.print(F("Settings: Alarm 3 Active | Time Set:"));
      Serial.print(Alarm_3_Hour);
      Serial.print(F(":"));
      if (Alarm_3_Minute < 10) Serial.print("0");
      Serial.print(Alarm_3_Minute);
      Serial.print(" | Action :");
      if (Alarm_3_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_3_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_3_Action == 3) Serial.println("Mow the Line");
      if (Alarm_3_Action == 4) Serial.println("Quick Start");
      if (Alarm_3_Action == 5) Serial.println("Custom");
      }
  }
    

  int PWM_LEFT_dueFlashStorage = dueFlashStorage.read(13);
  if (PWM_LEFT_dueFlashStorage == 1) {
    PWM_MaxSpeed_LH = dueFlashStorage.read(14);
    Serial.print(F("FLASH: PWM Wheel Left value : "));
    Serial.println(PWM_MaxSpeed_LH);
  }

  if (PWM_LEFT_dueFlashStorage != 1) {
    Serial.print(F("Settings: PWM Wheel Left value: "));
    Serial.println(PWM_MaxSpeed_LH);
    }

  int PWM_RIGHT_dueFlashStorage = dueFlashStorage.read(15);
  if (PWM_RIGHT_dueFlashStorage == 1) {
    PWM_MaxSpeed_RH = dueFlashStorage.read(16); 
    Serial.print(F("FLASH: PWM Wheel Right value: ")); 
    Serial.println(PWM_MaxSpeed_RH);
  }

  if (PWM_RIGHT_dueFlashStorage != 1) {
    Serial.print(F("Settings: PWM Wheel Right value: ")); 
    Serial.println(PWM_MaxSpeed_RH);
  }

  int PWM_LEFT_Slow_dueFlashStorage = dueFlashStorage.read(94);
  if (PWM_LEFT_Slow_dueFlashStorage == 1) {
    PWM_Slow_Speed_LH = dueFlashStorage.read(95);
    Serial.print(F("FLASH: PWM Wheel Slow Left value: "));
    Serial.println(PWM_Slow_Speed_LH);
  }

  if (PWM_LEFT_Slow_dueFlashStorage != 1) {
    Serial.print(F("Settings: PWM Wheel Slow Left value: "));
    Serial.println(PWM_Slow_Speed_LH);
  }

  int PWM_RIGHT_Slow_dueFlashStorage = dueFlashStorage.read(96);
  if (PWM_RIGHT_Slow_dueFlashStorage == 1) {
    PWM_Slow_Speed_RH = dueFlashStorage.read(97); 
    Serial.print(F("FLASH: PWM Wheel Slow Right value: ")); 
    Serial.println(PWM_Slow_Speed_RH);
  }

  if (PWM_RIGHT_Slow_dueFlashStorage != 1) {
    PWM_Slow_Speed_RH = dueFlashStorage.read(97); 
    Serial.print(F("Settings: PWM Wheel Slow Right value: ")); 
    Serial.println(PWM_Slow_Speed_RH);
  }
  
  int Slow_Speed_MAG_dueFlashStorage = dueFlashStorage.read(98);
  if (Slow_Speed_MAG_dueFlashStorage == 1) {
    Slow_Speed_MAG = dueFlashStorage.read(99); 
    Slow_Speed_MAG = (Slow_Speed_MAG * -1 ) * 10;
    Serial.print(F("FLASH: Slow Speed MAG: ")); 
    Serial.println(Slow_Speed_MAG);
  }

 if (Slow_Speed_MAG_dueFlashStorage == 1) {
    Serial.print(F("Settings: Slow Speed MAG: ")); 
    Serial.println(Slow_Speed_MAG);
  }


  int PWM_BLADE_dueFlashStorage = dueFlashStorage.read(17);
  if (PWM_BLADE_dueFlashStorage == 1) {
    PWM_Blade_Speed = dueFlashStorage.read(18); 
    Serial.print(F("FLASH: PWM Blade value: ")); 
    Serial.println(PWM_Blade_Speed);
  }

  if (PWM_BLADE_dueFlashStorage != 1) {
    Serial.print(F("Settings: PWM Blade value: ")); 
    Serial.println(PWM_Blade_Speed);
  }


  int COMPASS_dueFlashStorage = dueFlashStorage.read(19);
  if (COMPASS_dueFlashStorage == 1) {
    Compass_Activate = dueFlashStorage.read(20);  
    Serial.print(F("FLASH: Compass Activated: "));
    if (Compass_Activate == 0) Serial.println(F("OFF"));
    if (Compass_Activate == 1) Serial.println(F("ON"));
  }

  if (COMPASS_dueFlashStorage != 1) {
    Serial.print(F("Settings: Compass Activated: "));
    if (Compass_Activate == 0) Serial.println(F("OFF"));
    if (Compass_Activate == 1) Serial.println(F("ON"));
  }

  int COMPASS_Setup_Mode_dueFlashStorage = dueFlashStorage.read(113);
  if (COMPASS_Setup_Mode_dueFlashStorage == 1) {
    Compass_Setup_Mode = dueFlashStorage.read(114);  
    Serial.print(F("FLASH: Compass Setup Mode: "));
   if (Compass_Setup_Mode == 1) Serial.println(F("Setup DFRobot_QMC5883 Compass"));
   if (Compass_Setup_Mode == 2) Serial.println(F("Setup QMC5883_Manual Compass"));           
   if (Compass_Setup_Mode == 3) Serial.println(F("Setup QMC5883L Compass"));      
   }

  if (COMPASS_Setup_Mode_dueFlashStorage != 1) {
    Serial.print(F("Settings: Compass Setup Mode: "));
   if (Compass_Setup_Mode == 1) Serial.println(F("Setup DFRobot_QMC5883 Compass"));
   if (Compass_Setup_Mode == 2) Serial.println(F("Setup QMC5883_Manual Compass"));           
   if (Compass_Setup_Mode == 3) Serial.println(F("Setup QMC5883L Compass"));      
   }

  int Compass_Heading_Hold_Enabled_dueFlashStorage = dueFlashStorage.read(59);
  if (Compass_Heading_Hold_Enabled_dueFlashStorage == 1) {
    Compass_Heading_Hold_Enabled = dueFlashStorage.read(60);  
    Serial.print(F("FLASH: Compass Heading Hold: "));
    if (Compass_Heading_Hold_Enabled == 0) Serial.println(F("OFF"));
    if (Compass_Heading_Hold_Enabled == 1) Serial.println(F("ON"));
  }

  if (Compass_Heading_Hold_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: Compass Heading Hold: "));
    if (Compass_Heading_Hold_Enabled == 0) Serial.println(F("OFF"));
    if (Compass_Heading_Hold_Enabled == 1) Serial.println(F("ON"));
  }

  int CPower_dueFlashStorage = dueFlashStorage.read(61);
  if (CPower_dueFlashStorage == 1) {
    CPower = dueFlashStorage.read(62); 
    CPower = CPower / 10; 
    Serial.print(F("FLASH: Compass Power: "));
    Serial.println(CPower);
  }

  if (CPower_dueFlashStorage != 1) {
    CPower = CPower / 10; 
    Serial.print(F("Settings: Compass Power: "));
    Serial.println(CPower);
  }

  int Compass_Home_dueFlashStorage = dueFlashStorage.read(27);
  if (Compass_Home_dueFlashStorage == 1) {
    Home_Wire_Compass_Heading = (dueFlashStorage.read(28) * 10);    // *10 as value can be more than 255. Vaule is therefore stored as a tenth value
    Serial.print(F("FLASH: Compass Home Degrees : ")); 
    Serial.println(Home_Wire_Compass_Heading);
  }

  if (Compass_Home_dueFlashStorage != 1) {
    Serial.print(F("Settings: Compass Home Degrees : ")); 
    Serial.println(Home_Wire_Compass_Heading);
  }

 int Tracking_PID_P_dueFlashStorage = dueFlashStorage.read(21);
  if (Tracking_PID_P_dueFlashStorage == 1) {
    P = dueFlashStorage.read(22); 
    P = P / 100; 
    Serial.print(F("FLASH: Traking PID P: "));
    Serial.println(P);
  }


  if (Tracking_PID_P_dueFlashStorage != 1) {
    Serial.print(F("Settings: Traking PID P: "));
    Serial.println(P);
  }

  int Pattern_Mow_dueFlashStorage = dueFlashStorage.read(23);
  if (Pattern_Mow_dueFlashStorage == 1) {
    Pattern_Mow = dueFlashStorage.read(24);  
    //Pattern_Mow = 1;
    Serial.print(F("FLASH: Pattern Mow Type: "));
    if (Pattern_Mow == 0) Serial.println(F("OFF"));
    if (Pattern_Mow == 1) Serial.println(F("ON Parallel"));
    if (Pattern_Mow == 2) Serial.println(F("ON Spiral"));
  }

  if (Pattern_Mow_dueFlashStorage != 1) {
    Serial.print(F("Settings: Pattern Mow Type: "));
    if (Pattern_Mow == 0) Serial.println(F("OFF"));
    if (Pattern_Mow == 1) Serial.println(F("ON Parallel"));
    if (Pattern_Mow == 2) Serial.println(F("ON Spiral"));
  }


  int Minimum_Volt_dueFlashStorage = dueFlashStorage.read(25);
  if (Minimum_Volt_dueFlashStorage == 1) {
    Battery_Min = dueFlashStorage.read(26); 
    Battery_Min = Battery_Min / 10; 
    Serial.print(F("FLASH: Minimum Battery Voltage: "));
    Serial.println(Battery_Min);
  }

  if (Minimum_Volt_dueFlashStorage != 1) {
    Serial.print(F("Settings: Minimum Battery Voltage: "));
    Serial.println(Battery_Min);
  }


  int Angle_Sensor_Enabled_dueFlashStorage = dueFlashStorage.read(29);
  if (Angle_Sensor_Enabled_dueFlashStorage == 1) {
    Angle_Sensor_Enabled = dueFlashStorage.read(30);  
    Serial.print(F("FLASH: Angle Sensor Enabled: "));
    if (Angle_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Angle_Sensor_Enabled == 1) Serial.println(F("ON"));
  }

  if (Angle_Sensor_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: Angle Sensor Enabled: "));
    if (Angle_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Angle_Sensor_Enabled == 1) Serial.println(F("ON"));
  }


  int Tip_Over_Sensor_Enabled_dueFlashStorage = dueFlashStorage.read(92);
  if (Tip_Over_Sensor_Enabled_dueFlashStorage == 1) {
    Tip_Over_Sensor_Enabled = dueFlashStorage.read(93);  
    Serial.print(F("dueFlashStorage Tip Sensor Enabled:  "));
    if (Tip_Over_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Tip_Over_Sensor_Enabled == 1) Serial.println(F("ON"));
  }

  if (Tip_Over_Sensor_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: Tip Sensor Enabled:  "));
    if (Tip_Over_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Tip_Over_Sensor_Enabled == 1) Serial.println(F("ON"));
  }

  
  int Mower_Turn_Delay_Min_dueFlashStorage = dueFlashStorage.read(31);
  if (Mower_Turn_Delay_Min_dueFlashStorage == 1) {
    Mower_Turn_Delay_Min = dueFlashStorage.read(32);
    Mower_Turn_Delay_Min = Mower_Turn_Delay_Min * 100;
    Serial.print(F("FLASH: Mower Turn Delay Min: "));
    Serial.println(Mower_Turn_Delay_Min);
  }

  if (Mower_Turn_Delay_Min_dueFlashStorage != 1) {
    Serial.print(F("Settings: Mower Turn Delay Min: "));
    Serial.println(Mower_Turn_Delay_Min);
  }


  int Mower_Turn_Delay_Max_dueFlashStorage = dueFlashStorage.read(33);
  if (Mower_Turn_Delay_Max_dueFlashStorage == 1) {
    Mower_Turn_Delay_Max = dueFlashStorage.read(34);
    Mower_Turn_Delay_Max =  Mower_Turn_Delay_Max * 100; 
    Serial.print(F("FLASH: Mower Turn Delay Max: "));
    Serial.println(Mower_Turn_Delay_Max);
  }


  if (Mower_Turn_Delay_Max_dueFlashStorage != 1) {
    Serial.print(F("Settings: Mower Turn Delay Max: "));
    Serial.println(Mower_Turn_Delay_Max);
  }


  int Mower_Reverse_Delay_dueFlashStorage = dueFlashStorage.read(35);
  if (Mower_Reverse_Delay_dueFlashStorage == 1) {
    Mower_Reverse_Delay = dueFlashStorage.read(36);
    Mower_Reverse_Delay = (Mower_Reverse_Delay * 100);
    Serial.print(F("FLASH: Mower Reverse Time Delay/ms: "));
    Serial.println(Mower_Reverse_Delay);
  }

  if (Mower_Reverse_Delay_dueFlashStorage != 1) {
    Serial.print(F("Settings: Mower Reverse Time Delay/ms: "));
    Serial.println(Mower_Reverse_Delay);
  }



  int Track_Wire_Zone_1_Cycles_dueFlashStorage = dueFlashStorage.read(43);
  if (Track_Wire_Zone_1_Cycles_dueFlashStorage == 1) {
    Track_Wire_Zone_1_Cycles = dueFlashStorage.read(44);
    Track_Wire_Zone_1_Cycles = (Track_Wire_Zone_1_Cycles * 100);
    Serial.print(F("FLASH: Zone 1 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_1_Cycles);
  }

  if (Track_Wire_Zone_1_Cycles_dueFlashStorage != 1) {
    Serial.print(F("Settings: Zone 1 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_1_Cycles);
  }

  int Track_Wire_Zone_2_Cycles_dueFlashStorage = dueFlashStorage.read(45);
  if (Track_Wire_Zone_2_Cycles_dueFlashStorage == 1) {
    Track_Wire_Zone_2_Cycles = dueFlashStorage.read(46);
    Track_Wire_Zone_2_Cycles = (Track_Wire_Zone_2_Cycles * 100);
    Serial.print(F("FLASH: Zone 2 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_2_Cycles);
  }

  if (Track_Wire_Zone_2_Cycles_dueFlashStorage != 1) {
    Serial.print(F("Settings: Zone 2 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_2_Cycles);
  }


  int Use_Charging_Station_dueFlashStorage = dueFlashStorage.read(47);
  if (Use_Charging_Station_dueFlashStorage == 1) {
    Use_Charging_Station = dueFlashStorage.read(48);  
    Serial.print(F("FLASH: Charge Station: "));
    if (Use_Charging_Station == 0) Serial.println(F("OFF"));
    if (Use_Charging_Station == 1) Serial.println(F("ON"));
  }

  if (Use_Charging_Station_dueFlashStorage != 1) {
    Serial.print(F("Settings: Charge Station: "));
    if (Use_Charging_Station == 0) Serial.println(F("OFF"));
    if (Use_Charging_Station == 1) Serial.println(F("ON"));
  }


  int CW_Tracking_To_Charge_dueFlashStorage = dueFlashStorage.read(49);
  if (CW_Tracking_To_Charge_dueFlashStorage == 1) {
    CW_Tracking_To_Charge = dueFlashStorage.read(50);  
    Serial.print(F("FLASH: Tracking Direction to Charge : "));
    if (CW_Tracking_To_Charge == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }


  if (CW_Tracking_To_Charge_dueFlashStorage != 1) {
    Serial.print(F("Settings: Tracking Direction to Charge : "));
    if (CW_Tracking_To_Charge == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }

  int CCW_Tracking_To_Charge_dueFlashStorage = dueFlashStorage.read(51);
  if (CCW_Tracking_To_Charge_dueFlashStorage == 1) {
    CCW_Tracking_To_Charge = dueFlashStorage.read(52);  
    Serial.print(F("dueFlashStorage Tracking Direction to Charge : "));
    if (CCW_Tracking_To_Charge == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }

  if (CCW_Tracking_To_Charge_dueFlashStorage != 1) {
    Serial.print(F("Settings: Tracking Direction to Charge : "));
    if (CCW_Tracking_To_Charge == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }


  int CW_Tracking_To_Start_dueFlashStorage = dueFlashStorage.read(53);
  if (CW_Tracking_To_Start_dueFlashStorage == 1) {
    CW_Tracking_To_Start = dueFlashStorage.read(54);  
    Serial.print(F("FLASH: Tracking Direction to Start : "));
    if (CW_Tracking_To_Start == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }

  if (CW_Tracking_To_Start_dueFlashStorage != 1) {
    Serial.print(F("Settings: Tracking Direction to Start : "));
    if (CW_Tracking_To_Start == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }


  int CCW_Tracking_To_Start_dueFlashStorage = dueFlashStorage.read(55);
  if (CCW_Tracking_To_Start_dueFlashStorage == 1) {
    CCW_Tracking_To_Start = dueFlashStorage.read(56);  
    Serial.print(F("EERPOM: Tracking Direction to Start : "));
    if (CCW_Tracking_To_Start == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }

  if (CCW_Tracking_To_Start_dueFlashStorage != 1) {
    Serial.print(F("Settings: Tracking Direction to Start : "));
    if (CCW_Tracking_To_Start == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }

  int Max_Cycles_Straight_dueFlashStorage = dueFlashStorage.read(57);
  if (Max_Cycles_Straight_dueFlashStorage == 1) {
    Max_Cycles_Straight = dueFlashStorage.read(58);
    Max_Cycles_Straight = (Max_Cycles_Straight * 10);
    Serial.print(F("FLASH: Straight Line Cycles before turn: "));
    Serial.println(Max_Cycles_Straight);
  }

  if (Max_Cycles_Straight_dueFlashStorage != 1) {
    Serial.print(F("Settings: Straight Line Cycles before turn: "));
    Serial.println(Max_Cycles_Straight);
  }

  int Sonar_1_Activate_dueFlashStorage = dueFlashStorage.read(37);
  if (Sonar_1_Activate_dueFlashStorage == 1) {
    Sonar_1_Activate = dueFlashStorage.read(38);
    Serial.print(F("FLASH: Sonar 1 Activated: "));
    Serial.println(Sonar_1_Activate);
  }

  if (Sonar_1_Activate_dueFlashStorage != 1) {
    Serial.print(F("Settings: Sonar 1 Activated: "));
    Serial.println(Sonar_1_Activate);
  }

  int Sonar_2_Activate_dueFlashStorage = dueFlashStorage.read(39);
  if (Sonar_2_Activate_dueFlashStorage == 1) {
    Sonar_2_Activate = dueFlashStorage.read(40);
    Serial.print(F("FLASH: Sonar 2 Activated: "));
    Serial.println(Sonar_2_Activate);
  }

  if (Sonar_2_Activate_dueFlashStorage != 1) {
    Serial.print(F("Settings: Sonar 2 Activated: "));
    Serial.println(Sonar_2_Activate);
  }

  int Sonar_3_Activate_dueFlashStorage = dueFlashStorage.read(41);
  if (Sonar_3_Activate_dueFlashStorage == 1) {
    Sonar_3_Activate = dueFlashStorage.read(42);
    Serial.print(F("FLASH: Sonar 3 Activated: "));
    Serial.println(Sonar_3_Activate);
  }

  if (Sonar_3_Activate_dueFlashStorage != 1) {
    Serial.print(F("Settings: Sonar 3 Activated: "));
    Serial.println(Sonar_3_Activate);
  }

  int Max_Sonar_Hit_dueFlashStorage = dueFlashStorage.read(63);
  if (Max_Sonar_Hit_dueFlashStorage == 1) {
    Max_Sonar_Hit = dueFlashStorage.read(64); 
    Serial.print(F("FLASH: Sonar Sensitivity: "));
    Serial.println(Max_Sonar_Hit);
  }

  if (Max_Sonar_Hit_dueFlashStorage != 1) {
    Serial.print(F("Settings: Sonar Sensitivity: "));
    Serial.println(Max_Sonar_Hit);
  }

  int maxdistancesonar_dueFlashStorage = dueFlashStorage.read(65);
  if (maxdistancesonar_dueFlashStorage == 1) {
    maxdistancesonar = dueFlashStorage.read(66); 
    Serial.print(F("FLASH: Sonar Activation Distance: "));
    Serial.println(maxdistancesonar);
  }

  if (maxdistancesonar_dueFlashStorage != 1) {
    Serial.print(F("Settings: Sonar Activation Distance: "));
    Serial.println(maxdistancesonar);
  }

  int Perimeter_Wire_Enabled_dueFlashStorage = dueFlashStorage.read(67);
  if (Perimeter_Wire_Enabled_dueFlashStorage == 1) {
    Perimeter_Wire_Enabled = dueFlashStorage.read(68);  
    Serial.print(F("FLASH: Wire Module ON/OFF: "));
    if (Perimeter_Wire_Enabled == 0) Serial.println(F("OFF"));
    if (Perimeter_Wire_Enabled == 1) Serial.println(F("ON"));
  }

  if (Perimeter_Wire_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: Wire Module ON/OFF: "));
    if (Perimeter_Wire_Enabled == 0) Serial.println(F("OFF"));
    if (Perimeter_Wire_Enabled == 1) Serial.println(F("ON"));
  }


  int Max_Cycle_Wire_Find_dueFlashStorage = dueFlashStorage.read(69);
  if (Max_Cycle_Wire_Find_dueFlashStorage == 1) {
    Max_Cycle_Wire_Find = dueFlashStorage.read(70);
    Max_Cycle_Wire_Find = (Max_Cycle_Wire_Find * 10);
    Serial.print(F("FLASH: Track Cycles Forwards to find Wire "));
    Serial.println(Max_Cycle_Wire_Find);
  }


  if (Max_Cycle_Wire_Find_dueFlashStorage != 1) {
    Serial.print(F("Settings: Track Cycles Forwards to find Wire "));
    Serial.println(Max_Cycle_Wire_Find);
  }


  int Max_Cycle_Wire_Find_Back_dueFlashStorage = dueFlashStorage.read(71);
  if (Max_Cycle_Wire_Find_Back_dueFlashStorage == 1) {
    Max_Cycle_Wire_Find_Back = dueFlashStorage.read(72);
    Max_Cycle_Wire_Find_Back = (Max_Cycle_Wire_Find_Back * 10);
    Serial.print(F("FLASH: Track Cycles Back to find Wire "));
    Serial.println(Max_Cycle_Wire_Find_Back);
  }


  if (Max_Cycle_Wire_Find_Back_dueFlashStorage != 1) {
    Serial.print(F("Settings: Track Cycles Back to find Wire "));
    Serial.println(Max_Cycle_Wire_Find_Back);
  }


  int Max_Tracking_Turn_Right_dueFlashStorage = dueFlashStorage.read(73);
  if (Max_Tracking_Turn_Right_dueFlashStorage == 1) {
    Max_Tracking_Turn_Right = dueFlashStorage.read(74);
    Max_Tracking_Turn_Right = (Max_Tracking_Turn_Right * 10);
    Serial.print(F("FLASH: Wheel RH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Right);
  }

  if (Max_Tracking_Turn_Right_dueFlashStorage != 1) {
    Serial.print(F("Settings: Wheel RH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Right);
  }



  int Max_Tracking_Turn_Left_dueFlashStorage = dueFlashStorage.read(75);
  if (Max_Tracking_Turn_Left_dueFlashStorage == 1) {
    Max_Tracking_Turn_Left = dueFlashStorage.read(76);
    Max_Tracking_Turn_Left = (Max_Tracking_Turn_Left * 10);
    Serial.print(F("FLASH: Wheel LH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Left);
  }


  if (Max_Tracking_Turn_Left_dueFlashStorage != 1) {
    Serial.print(F("Settings: Wheel LH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Left);
  }


  int Rain_Sensor_Installed_dueFlashStorage = dueFlashStorage.read(77);
  if (Rain_Sensor_Installed_dueFlashStorage == 1) {
    Rain_Sensor_Installed = dueFlashStorage.read(78);  
    Serial.print(F("FLASH: Rain Sensor ON/OFF:  "));
    if (Rain_Sensor_Installed == 0) Serial.println(F("OFF"));
    if (Rain_Sensor_Installed == 1) Serial.println(F("ON"));
  }


  if (Rain_Sensor_Installed_dueFlashStorage != 1) {
    Serial.print(F("Settings: Rain Sensor ON/OFF: "));
    if (Rain_Sensor_Installed == 0) Serial.println(F("OFF"));
    if (Rain_Sensor_Installed == 1) Serial.println(F("ON"));
  }


  int Rain_Total_Hits_Go_Home_dueFlashStorage = dueFlashStorage.read(79);
  if (Rain_Total_Hits_Go_Home_dueFlashStorage == 1) {
    Rain_Total_Hits_Go_Home = dueFlashStorage.read(80); 
    Serial.print(F("Settings: Rain Sensitivity: "));
    Serial.println(Rain_Total_Hits_Go_Home);
  }

  if (Rain_Total_Hits_Go_Home_dueFlashStorage != 1) {
    Serial.print(F("Settings: Rain Sensitivity: "));
    Serial.println(Rain_Total_Hits_Go_Home);
  }

  int WIFI_Enabled_dueFlashStorage = dueFlashStorage.read(81);
  if (WIFI_Enabled_dueFlashStorage == 1) {
    WIFI_Enabled = dueFlashStorage.read(82);  
    Serial.print(F("FLASH: WIFI Enabled: "));
    if (WIFI_Enabled == 0) Serial.println(F("OFF"));
    if (WIFI_Enabled == 1) Serial.println(F("ON"));
  }

  if (WIFI_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: WIFI Enabled: "));
    if (WIFI_Enabled == 0) Serial.println(F("OFF"));
    if (WIFI_Enabled == 1) Serial.println(F("ON"));
  }

  int Cutting_Blades_Activate_dueFlashStorage = dueFlashStorage.read(83);
  if (Cutting_Blades_Activate_dueFlashStorage == 1) {
    Cutting_Blades_Activate = dueFlashStorage.read(84);  
    Serial.print(F("FLASH: CUTTING BLADE Motor SET TO : "));
    if (Cutting_Blades_Activate == 0) Serial.println(F("OFF"));
    if (Cutting_Blades_Activate == 1) Serial.println(F("ON"));
  }


  if (Cutting_Blades_Activate_dueFlashStorage != 1) {
    Serial.print(F("Settings: CUTTING BLADE Motor SET TO : "));
    if (Cutting_Blades_Activate == 0) Serial.println(F("OFF"));
    if (Cutting_Blades_Activate == 1) Serial.println(F("ON"));
  }

  int Low_Battery_Instances_Chg_dueFlashStorage = dueFlashStorage.read(85);
  if (Low_Battery_Instances_Chg_dueFlashStorage == 1) {
    Low_Battery_Instances_Chg = dueFlashStorage.read(86); 
    Serial.print(F("FLASH: Battery to Charge Sensitivity: "));
    Serial.println(Low_Battery_Instances_Chg);
  }

  if (Low_Battery_Instances_Chg_dueFlashStorage != 1) {
    Serial.print(F("Settings: Battery to Charge Sensitivity: "));
    Serial.println(Low_Battery_Instances_Chg);
  }


//  int Bumper_Activate_Frnt_dueFlashStorage = dueFlashStorage.read(90);
//  if (Bumper_Activate_Frnt_dueFlashStorage == 1) {
//    Bumper_Activate_Frnt = dueFlashStorage.read(91);  
//    Serial.print(F("FLASH: Bumper Bar Enabled: "));
//    if (Bumper_Activate_Frnt == 0) Serial.println(F("OFF"));
//    if (Bumper_Activate_Frnt == 1) Serial.println(F("ON"));
//  }
//
//  if (Bumper_Activate_Frnt_dueFlashStorage != 1) {
//    Serial.print(F("Settings: Bumper Bar Enabled: "));
//    if (Bumper_Activate_Frnt == 0) Serial.println(F("OFF"));
//    if (Bumper_Activate_Frnt == 1) Serial.println(F("ON"));
//  }


  int Turn_90_Delay_LH_dueFlashStorage = dueFlashStorage.read(101);
  if (Turn_90_Delay_LH_dueFlashStorage == 1) {
    Turn_90_Delay_LH = dueFlashStorage.read(102);  
    Turn_90_Delay_LH = Turn_90_Delay_LH * 10;
    Serial.print(F("FLASH: Turn_90_Delay_LH Enabled: "));
    Serial.println(Turn_90_Delay_LH);
  }

  if (Turn_90_Delay_LH_dueFlashStorage != 1) {
    Serial.print(F("Settings: Turn_90_Delay_LH Enabled: "));
    Serial.println(Turn_90_Delay_LH);
  }


  int Turn_90_Delay_RH_dueFlashStorage = dueFlashStorage.read(103);
  if (Turn_90_Delay_RH_dueFlashStorage == 1) {
    Turn_90_Delay_RH = dueFlashStorage.read(104);  
    Turn_90_Delay_RH = Turn_90_Delay_RH * 10;
    Serial.print(F("FLASH: Turn_90_Delay_RH Enabled: "));
    Serial.println(Turn_90_Delay_RH);
  }

  if (Turn_90_Delay_RH_dueFlashStorage != 1) {
    Serial.print(F("Settings: Turn_90_Delay_RH Enabled: "));
    Serial.println(Turn_90_Delay_RH);
  }


  int Line_Length_Cycles_dueFlashStorage = dueFlashStorage.read(105);
  if (Line_Length_Cycles_dueFlashStorage == 1) {
    Line_Length_Cycles = dueFlashStorage.read(106);  
    Line_Length_Cycles = Line_Length_Cycles * 10;
    Serial.print(F("FLASH: Line_Length_Cycles Enabled: "));
    Serial.println(Line_Length_Cycles);
  }


  if (Line_Length_Cycles_dueFlashStorage != 1) {
    Serial.print(F("Settings: Line_Length_Cycles Enabled: "));
    Serial.println(Line_Length_Cycles);
  }



  int GPS_Enabled_dueFlashStorage = dueFlashStorage.read(107);
  if (GPS_Enabled_dueFlashStorage == 1) {
    GPS_Enabled = dueFlashStorage.read(108);  
    Serial.print(F("FLASH: GPS Enabled: "));
    if (GPS_Enabled == 0) Serial.println(F("Disabled"));
    if (GPS_Enabled == 1) Serial.println(F("Enabled"));
  }

  if (GPS_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: GPS Enabled: "));
    if (GPS_Enabled == 0) Serial.println(F("Disabled"));
    if (GPS_Enabled == 1) Serial.println(F("Enabled"));
  }

  int GYRO_Enabled_dueFlashStorage = dueFlashStorage.read(109);
  if (GYRO_Enabled_dueFlashStorage == 1) {
    GYRO_Enabled = dueFlashStorage.read(110);  
    Serial.print(F("FLASH: GYRO Enabled: "));
    if (GYRO_Enabled == 0) Serial.println(F("Disabled"));
    if (GYRO_Enabled == 1) Serial.println(F("Enabled"));
  }

  if (GYRO_Enabled_dueFlashStorage == 1) {
    Serial.print(F("Settings: GYRO Enabled: "));
    if (GYRO_Enabled == 0) Serial.println(F("Disabled"));
    if (GYRO_Enabled == 1) Serial.println(F("Enabled"));
  }

  
  int GPower_dueFlashStorage = dueFlashStorage.read(111);
  if (GPower_dueFlashStorage == 1) {
    GPower = dueFlashStorage.read(112); 
    GPower = GPower / 10; 
    Serial.print(F("FLASH: GYRO PID : "));
    Serial.println(GPower);
  }

  if (GPower_dueFlashStorage != 1) {
    Serial.print(F("Settings: GYRO PID : "));
    Serial.println(GPower);
  }

  int Wheel_Amp_Sensor_ON_Enabled_dueFlashStorage = dueFlashStorage.read(115);
  if (Wheel_Amp_Sensor_ON_Enabled_dueFlashStorage == 1) {
    Wheel_Amp_Sensor_ON = dueFlashStorage.read(116);  
    Serial.print(F("FLASH: Wheel Block Amp Sensor Enabled: "));
    if (Wheel_Amp_Sensor_ON == 0) Serial.println(F("Disabled"));
    if (Wheel_Amp_Sensor_ON == 1) Serial.println(F("Enabled"));
  }


  if (Wheel_Amp_Sensor_ON_Enabled_dueFlashStorage != 1) {
    Serial.print(F("Settings: Wheel Block Amp Sensor Enabled: "));
    if (Wheel_Amp_Sensor_ON == 0) Serial.println(F("Disabled"));
    if (Wheel_Amp_Sensor_ON == 1) Serial.println(F("Enabled"));
  }

  
  int Max_Wheel_Amps_dueFlashStorage = dueFlashStorage.read(117);
  if (Max_Wheel_Amps_dueFlashStorage == 1) {
    Max_Wheel_Amps = dueFlashStorage.read(118); 
    Max_Wheel_Amps = Max_Wheel_Amps / 10; 
    Serial.print(F("FLASH: Wheel Block Amps: "));
    Serial.println(Max_Wheel_Amps);
  }

  if (Max_Wheel_Amps_dueFlashStorage != 1) {
    Max_Wheel_Amps = Max_Wheel_Amps / 10; 
    Serial.print(F("Settings: Wheel Block Amps: "));
    Serial.println(Max_Wheel_Amps);
  }

  if (Max_Wheel_Amps_dueFlashStorage != 1) {
    Max_Wheel_Amps = Max_Wheel_Amps / 10; 
    Serial.print(F("Settings: Wheel Block Amps: "));
    Serial.println(Max_Wheel_Amps);
  }







Serial.println(F("*********** DONE **************"));
 delay(500);

#endif
}


void Clear_FLASH() {
  
  #if defined(BOARD_DUE)
  
  dueFlashStorage.write(1,0);      // Clear Alarm 1
  dueFlashStorage.write(5,0);      // Clear Alarm 2
  dueFlashStorage.write(9,0);      // Clear Alarm 3
  dueFlashStorage.write(13,0);     // Clear PWM Left Wheel
  dueFlashStorage.write(15,0);     // Clear PWM Right Wheel
  dueFlashStorage.write(17,0);     // Clear PWM Blade
  dueFlashStorage.write(19,0);     // Clear Compass Setting dueFlashStorage
  dueFlashStorage.write(21,0);     // Clear PID Setting
  dueFlashStorage.write(23,0);     // Clear Pattern Mow
  dueFlashStorage.write(25,0);     // Clear Volt Minimum
  dueFlashStorage.write(27,0);     // Clear Compass Home
  dueFlashStorage.write(29,0);     // Clear Tilt Tip Safety
  dueFlashStorage.write(31,0);     // Clear Turn Time Min
  dueFlashStorage.write(33,0);     // Clear Turn Time Max
  dueFlashStorage.write(35,0);     // Clear Reverse Time
  dueFlashStorage.write(37,0);     // Clear Sonar 1
  dueFlashStorage.write(39,0);     // Clear Sonar 2
  dueFlashStorage.write(41,0);     // Clear Sonar 3
  dueFlashStorage.write(43,0);     // Clear Zone 1 Cycles
  dueFlashStorage.write(45,0);     // Clear Zone 2 Cycles
  dueFlashStorage.write(47,0);     // Clear Charging Station Options
  dueFlashStorage.write(49,0);     // Reset CW and CCW Exit and Dock Directions
  dueFlashStorage.write(51,0);     //  CW CCW Tracking
  dueFlashStorage.write(53,0);     //  CW CCW Tracking
  dueFlashStorage.write(55,0);     //  CW CCW Tracking
  dueFlashStorage.write(57,0);     // Max Cycle Straight
  dueFlashStorage.write(59,0);     // Compass Heading HOld ON/OFF
  dueFlashStorage.write(61,0);     // Compass PID reset.
  dueFlashStorage.write(63,0);     // Sonar sensitivity.
  dueFlashStorage.write(65,0);      // MAx distance sonar reset.
  dueFlashStorage.write(67,0);     // Wire Sensor ON/OFF
  dueFlashStorage.write(69,0);     // Track cycles Forwards
  dueFlashStorage.write(71,0);     // Track Cycles Back
  dueFlashStorage.write(73,0);     // RH Cycles to restart
  dueFlashStorage.write(75,0);     // LH Cycles to restart
  dueFlashStorage.write(77,0);     // Rain ON/OFF
  dueFlashStorage.write(79,0);     // Rain sensitivity
  dueFlashStorage.write(81,0);     // WIFI ON/OFF
  dueFlashStorage.write(83,0);     // Cutting Blades ON/OFF
  dueFlashStorage.write(85,0);     // Batt sensitivity;
  dueFlashStorage.write(87,0);     // Alarm Actions 1-3
  dueFlashStorage.write(88,0);
  dueFlashStorage.write(89,0);
//  dueFlashStorage.write(90,0);     // Bumper Bar
  dueFlashStorage.write(92,0);     // Tip Over sensor
  dueFlashStorage.write(94,0);     // Wheel Slow Speed LH
  dueFlashStorage.write(96,0);     // Wheel Slow Speed RH
  dueFlashStorage.write(98,0);     // Slow MAG Point
  dueFlashStorage.write(102,0);    // Turn_90_Delay_LH 
  dueFlashStorage.write(104,0);    // Turn_90_Delay_RH
  dueFlashStorage.write(106,0);    // Line_Length_Cycles
  dueFlashStorage.write(107,0);    // GPS Enabled
  dueFlashStorage.write(109,0);    // GYRO Enabled
  dueFlashStorage.write(111,0);    // GYRO Power
  dueFlashStorage.write(113,0);    // Compass Setup Mode
  dueFlashStorage.write(115,0);    // Wheel amp sensor 
  dueFlashStorage.write(117,0);    // Wheel amp sensor value

  #endif
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// EEPROM
// The forst block/address of the dueFlashStorage is either 1 or 0.  
// If information is stored = 1, then the rest of the EEPROM values for that
// block are loaded into the global variable values.


void Load_EEPROM_Saved_Data() {

  #if defined(BOARD_MEGA)

Serial.println("Loading EEPROM / Settings");

  int Robot_Type_EEPROM = EEPROM.read(121);
  if (Robot_Type_EEPROM == 1) {
    Robot_Type = EEPROM.read(122); 
    Serial.print(F("EEPROM: Robot_Type: "));
    if (Robot_Type == 1) Serial.println(F("= Mower"));
 //   if (Robot_Type == 2) Serial.println(F("= Aerator"));
  }

  if (Robot_Type_EEPROM != 1) {
    if (Robot_Type == 1) Serial.println(F("Settings: Robot Type = Mower"));
//    if (Robot_Type == 2) Serial.println(F("Settings: Robot Type = Aerator"));
  }


  int PCB_EEPROM = EEPROM.read(119);
  if (PCB_EEPROM == 1) {
    PCB = EEPROM.read(120); 
    Serial.print(F("EEPROM: PCB Enabled: "));
      if (PCB == 1) Serial.println(F("Enabled"));
      if (PCB == 0) Serial.println(F("Disabled"));
      }

  if (PCB_EEPROM != 1) {
    Serial.print(F("Settings: PCB Enabled: "));
      if (PCB == 1) Serial.println(F("Enabled"));
      if (PCB == 0) Serial.println(F("Disabled"));
      }

 
  int Alarm_1_Saved_EEPROM = EEPROM.read(1);
  
  if (Alarm_1_Saved_EEPROM == 1) {
    Alarm_1_Hour    = EEPROM.read(2);
    Alarm_1_Minute  = EEPROM.read(3);
    Alarm_1_ON      = EEPROM.read(4);
    Alarm_1_Action  = EEPROM.read(87);

    if (Alarm_1_Hour == 255)   {
      Alarm_1_Hour = 0;
      Alarm_1_ON = 0;
      }
    if (Alarm_1_Minute == 255) {
      Alarm_1_Minute = 0;
      Alarm_1_ON  = 0;
      }
    
    if (Alarm_1_ON == 0)  Serial.println("Alarm 1 : OFF");
 
    if (Alarm_1_ON == 1) {
      Serial.print(F("EEPROM Alarm 1 Active | Time Set:"));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      if (Alarm_1_Minute < 10) Serial.print("0");
      Serial.print(Alarm_1_Minute);
      Serial.print(" | Action :");
      if (Alarm_1_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_1_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_1_Action == 3) Serial.println("Mow the Line");
      if (Alarm_1_Action == 4) Serial.println("Quick Start");
      if (Alarm_1_Action == 5) Serial.println("Custom");
      }
    }
  if (Alarm_1_Saved_EEPROM != 1) {
    if (Alarm_1_ON == 1) {
      Serial.print(F("Settings: Alarm 1 Active | Time Set:"));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      if (Alarm_1_Minute < 10) Serial.print("0");
      Serial.print(Alarm_1_Minute);
      Serial.print(" | Action :");
      if (Alarm_1_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_1_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_1_Action == 3) Serial.println("Mow the Line");
      if (Alarm_1_Action == 4) Serial.println("Quick Start");
      if (Alarm_1_Action == 5) Serial.println("Custom");
      }
  }
    
  int Alarm_2_Saved_EEPROM = EEPROM.read(5);
  
  if (Alarm_2_Saved_EEPROM == 1) {
    Alarm_2_Hour    = EEPROM.read(6);
    Alarm_2_Minute  = EEPROM.read(7);
    Alarm_2_ON      = EEPROM.read(8);
    Alarm_2_Action  = EEPROM.read(88);
    
    if (Alarm_2_Hour == 255)   {
      Alarm_2_Hour = 0;
      Alarm_2_ON = 0;
      }
    if (Alarm_2_Minute == 255) {
      Alarm_2_Minute = 0;
      Alarm_2_ON  = 0;
      }
    
    if (Alarm_2_ON == 0) Serial.println("Alarm 2 : OFF:");
    
    if (Alarm_2_ON == 1) {
      Serial.print(F("Alarm 2 Active | Time Set:"));
      Serial.print(Alarm_2_Hour);
      Serial.print(F(":"));
      if (Alarm_2_Minute < 10) Serial.print("0");
      Serial.print(Alarm_2_Minute);  
      Serial.print(" | Action :");
      if (Alarm_2_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_2_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_2_Action == 3) Serial.println("Mow the Line");
      if (Alarm_2_Action == 4) Serial.println("Quick Start");
      if (Alarm_2_Action == 5) Serial.println("Custom");
      }

    }

  if (Alarm_2_Saved_EEPROM != 1) {
    if (Alarm_2_ON == 1) {
      Serial.print(F("Settings: Alarm 2 Active | Time Set:"));
      Serial.print(Alarm_2_Hour);
      Serial.print(F(":"));
      if (Alarm_2_Minute < 10) Serial.print("0");
      Serial.print(Alarm_2_Minute);
      Serial.print(" | Action :");
      if (Alarm_2_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_2_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_2_Action == 3) Serial.println("Mow the Line");
      if (Alarm_2_Action == 4) Serial.println("Quick Start");
      if (Alarm_2_Action == 5) Serial.println("Custom");
      }
  }


  int Alarm_3_Saved_EEPROM = EEPROM.read(9);
  
  if (Alarm_3_Saved_EEPROM == 1) {
    Alarm_3_Hour    = EEPROM.read(10);
    Alarm_3_Minute  = EEPROM.read(11);
    Alarm_3_ON      = EEPROM.read(12);
    Alarm_3_Action  = EEPROM.read(89);

    if (Alarm_3_Hour == 255)   {
      Alarm_3_Hour = 0;
      Alarm_3_ON = 0;
      }
    if (Alarm_3_Minute == 255) {
      Alarm_3_Minute = 0;
      Alarm_3_ON  = 0;
      }
    
    
    if (Alarm_3_ON == 0) Serial.println("Alarm 3 : OFF: ");
   
    if (Alarm_3_ON == 1) {
      Serial.print(F("Alarm 3 Active | Time Set:"));
      Serial.print(Alarm_3_Hour);
      Serial.print(F(":"));
      if (Alarm_3_Minute < 10) Serial.print("0");
      Serial.print(Alarm_3_Minute);
      Serial.print(" | Action :");
      if (Alarm_3_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_3_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_3_Action == 3) Serial.println("Mow the Line");
      if (Alarm_3_Action == 4) Serial.println("Quick Start");
      if (Alarm_3_Action == 5) Serial.println("Custom");  
      }

    }

  if (Alarm_3_Saved_EEPROM != 1) {
    if (Alarm_3_ON == 1) {
      Serial.print(F("Settings: Alarm 3 Active | Time Set:"));
      Serial.print(Alarm_3_Hour);
      Serial.print(F(":"));
      if (Alarm_3_Minute < 10) Serial.print("0");
      Serial.print(Alarm_3_Minute);
      Serial.print(" | Action :");
      if (Alarm_3_Action == 1) Serial.println("Exit Zone 1");
      if (Alarm_3_Action == 2) Serial.println("Exit Zone 2");
      if (Alarm_3_Action == 3) Serial.println("Mow the Line");
      if (Alarm_3_Action == 4) Serial.println("Quick Start");
      if (Alarm_3_Action == 5) Serial.println("Custom");
      }
  }
    

  int PWM_LEFT_EEPROM = EEPROM.read(13);
  if (PWM_LEFT_EEPROM == 1) {
    PWM_MaxSpeed_LH = EEPROM.read(14);
    Serial.print(F("EEPROM PWM Wheel Left value : "));
    Serial.println(PWM_MaxSpeed_LH);
  }

  if (PWM_LEFT_EEPROM != 1) {
    Serial.print(F("Settings: PWM Wheel Left value: "));
    Serial.println(PWM_MaxSpeed_LH);
    }

  int PWM_RIGHT_EEPROM = EEPROM.read(15);
  if (PWM_RIGHT_EEPROM == 1) {
    PWM_MaxSpeed_RH = EEPROM.read(16); 
    Serial.print(F("EEPROM: PWM Wheel Right value: ")); 
    Serial.println(PWM_MaxSpeed_RH);
  }

  if (PWM_RIGHT_EEPROM != 1) {
    Serial.print(F("Settings: PWM Wheel Right value: ")); 
    Serial.println(PWM_MaxSpeed_RH);
  }

  int PWM_LEFT_Slow_EEPROM = EEPROM.read(94);
  if (PWM_LEFT_Slow_EEPROM == 1) {
    PWM_Slow_Speed_LH = EEPROM.read(95);
    Serial.print(F("EEPROM: PWM Wheel Slow Left value: "));
    Serial.println(PWM_Slow_Speed_LH);
  }

  if (PWM_LEFT_Slow_EEPROM != 1) {
    Serial.print(F("Settings: PWM Wheel Slow Left value: "));
    Serial.println(PWM_Slow_Speed_LH);
  }

  int PWM_RIGHT_Slow_EEPROM = EEPROM.read(96);
  if (PWM_RIGHT_Slow_EEPROM == 1) {
    PWM_Slow_Speed_RH = EEPROM.read(97); 
    Serial.print(F("EEPROM: PWM Wheel Slow Right value: ")); 
    Serial.println(PWM_Slow_Speed_RH);
  }

  if (PWM_RIGHT_Slow_EEPROM != 1) {
    //PWM_Slow_Speed_RH = EEPROM.read(97);
    Serial.print(F("Settings: PWM Wheel Slow Right value: ")); 
    Serial.println(PWM_Slow_Speed_RH);
  }
  
  int Slow_Speed_MAG_EEPROM = EEPROM.read(98);
  if (Slow_Speed_MAG_EEPROM == 1) {
    Slow_Speed_MAG = EEPROM.read(99); 
    Slow_Speed_MAG = (Slow_Speed_MAG * -1 ) * 10;
    Serial.print(F("EEPROM: Slow Speed MAG: ")); 
    Serial.println(Slow_Speed_MAG);
  }

 if (Slow_Speed_MAG_EEPROM == 1) {
    Serial.print(F("Settings: Slow Speed MAG: ")); 
    Serial.println(Slow_Speed_MAG);
  }


  int PWM_BLADE_EEPROM = EEPROM.read(17);
  if (PWM_BLADE_EEPROM == 1) {
    PWM_Blade_Speed = EEPROM.read(18); 
    Serial.print(F("EEPROM: PWM Blade value: ")); 
    Serial.println(PWM_Blade_Speed);
  }

  if (PWM_BLADE_EEPROM != 1) {
    Serial.print(F("Settings: PWM Blade value: ")); 
    Serial.println(PWM_Blade_Speed);
  }


  int COMPASS_EEPROM = EEPROM.read(19);
  if (COMPASS_EEPROM == 1) {
    Compass_Activate = EEPROM.read(20);  
    Serial.print(F("EEPROM: Compass Activated: "));
    if (Compass_Activate == 0) Serial.println(F("OFF"));
    if (Compass_Activate == 1) Serial.println(F("ON"));
  }

  if (COMPASS_EEPROM != 1) {
    Serial.print(F("Settings: Compass Activated: "));
    if (Compass_Activate == 0) Serial.println(F("OFF"));
    if (Compass_Activate == 1) Serial.println(F("ON"));
  }

  int COMPASS_Setup_Mode_EEPROM = EEPROM.read(113);
  if (COMPASS_Setup_Mode_EEPROM == 1) {
    Compass_Setup_Mode = EEPROM.read(114);  
    Serial.print(F("EEPROM: Compass Setup Mode: "));
   if (Compass_Setup_Mode == 1) Serial.println(F("Setup DFRobot_QMC5883 Compass"));
   if (Compass_Setup_Mode == 2) Serial.println(F("Setup QMC5883_Manual Compass"));           
   if (Compass_Setup_Mode == 3) Serial.println(F("Setup QMC5883L Compass"));      
   }

  if (COMPASS_Setup_Mode_EEPROM != 1) {
    Serial.print(F("Settings: Compass Setup Mode: "));
   if (Compass_Setup_Mode == 1) Serial.println(F("Setup DFRobot_QMC5883 Compass"));
   if (Compass_Setup_Mode == 2) Serial.println(F("Setup QMC5883_Manual Compass"));           
   if (Compass_Setup_Mode == 3) Serial.println(F("Setup QMC5883L Compass"));      
   }

  int Compass_Heading_Hold_Enabled_EEPROM = EEPROM.read(59);
  if (Compass_Heading_Hold_Enabled_EEPROM == 1) {
    Compass_Heading_Hold_Enabled = EEPROM.read(60);  
    Serial.print(F("EEPROM: Compass Heading Hold: "));
    if (Compass_Heading_Hold_Enabled == 0) Serial.println(F("OFF"));
    if (Compass_Heading_Hold_Enabled == 1) Serial.println(F("ON"));
  }

  if (Compass_Heading_Hold_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: Compass Heading Hold: "));
    if (Compass_Heading_Hold_Enabled == 0) Serial.println(F("OFF"));
    if (Compass_Heading_Hold_Enabled == 1) Serial.println(F("ON"));
  }

  int CPower_EEPROM = EEPROM.read(61);
  if (CPower_EEPROM == 1) {
    CPower = EEPROM.read(62); 
    CPower = CPower / 10; 
    Serial.print(F("EEPROM: Compass Power: "));
    Serial.println(CPower);
  }

  if (CPower_EEPROM != 1) {
    CPower = CPower; 
    Serial.print(F("Settings: Compass Power: "));
    Serial.println(CPower);
  }

  int Compass_Home_EEPROM = EEPROM.read(27);
  if (Compass_Home_EEPROM == 1) {
    Home_Wire_Compass_Heading = (EEPROM.read(28) * 10);    // *10 as value can be more than 255. Vaule is therefore stored as a tenth value
    Serial.print(F("EEPROM: Compass Home Degrees : ")); 
    Serial.println(Home_Wire_Compass_Heading);
  }

  if (Compass_Home_EEPROM != 1) {
    Serial.print(F("Settings: Compass Home Degrees : ")); 
    Serial.println(Home_Wire_Compass_Heading);
  }

 int Tracking_PID_P_EEPROM = EEPROM.read(21);
  if (Tracking_PID_P_EEPROM == 1) {
    P = EEPROM.read(22); 
    P = P / 100; 
    Serial.print(F("EEPROM: Traking PID P: "));
    Serial.println(P);
  }


  if (Tracking_PID_P_EEPROM != 1) {
    Serial.print(F("Settings: Traking PID P: "));
    Serial.println(P);
  }

  int Pattern_Mow_EEPROM = EEPROM.read(23);
  if (Pattern_Mow_EEPROM == 1) {
    Pattern_Mow = EEPROM.read(24);  
    //Pattern_Mow = 1;
    Serial.print(F("EEPROM: Pattern Mow Type: "));
    if (Pattern_Mow == 0) Serial.println(F("OFF"));
    if (Pattern_Mow == 1) Serial.println(F("ON Parallel"));
    if (Pattern_Mow == 2) Serial.println(F("ON Spiral"));
  }

  if (Pattern_Mow_EEPROM != 1) {
    Serial.print(F("Settings: Pattern Mow Type: "));
    if (Pattern_Mow == 0) Serial.println(F("OFF"));
    if (Pattern_Mow == 1) Serial.println(F("ON Parallel"));
    if (Pattern_Mow == 2) Serial.println(F("ON Spiral"));
  }


  int Minimum_Volt_EEPROM = EEPROM.read(25);
  if (Minimum_Volt_EEPROM == 1) {
    Battery_Min = EEPROM.read(26); 
    Battery_Min = Battery_Min / 10; 
    Serial.print(F("EEPROM: Minimum Battery Voltage: "));
    Serial.println(Battery_Min);
  }

  if (Minimum_Volt_EEPROM != 1) {
    Serial.print(F("Settings: Minimum Battery Voltage: "));
    Serial.println(Battery_Min);
  }


  int Angle_Sensor_Enabled_EEPROM = EEPROM.read(29);
  if (Angle_Sensor_Enabled_EEPROM == 1) {
    Angle_Sensor_Enabled = EEPROM.read(30);  
    Serial.print(F("EEPROM: Angle Sensor Enabled: "));
    if (Angle_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Angle_Sensor_Enabled == 1) Serial.println(F("ON"));
  }

  if (Angle_Sensor_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: Angle Sensor Enabled: "));
    if (Angle_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Angle_Sensor_Enabled == 1) Serial.println(F("ON"));
  }


  int Tip_Over_Sensor_Enabled_EEPROM = EEPROM.read(92);
  if (Tip_Over_Sensor_Enabled_EEPROM == 1) {
    Tip_Over_Sensor_Enabled = EEPROM.read(93);  
    Serial.print(F("EEPROM Tip Sensor Enabled:  "));
    if (Tip_Over_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Tip_Over_Sensor_Enabled == 1) Serial.println(F("ON"));
  }

  if (Tip_Over_Sensor_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: Tip Sensor Enabled:  "));
    if (Tip_Over_Sensor_Enabled == 0) Serial.println(F("OFF"));
    if (Tip_Over_Sensor_Enabled == 1) Serial.println(F("ON"));
  }

  
  int Mower_Turn_Delay_Min_EEPROM = EEPROM.read(31);
  if (Mower_Turn_Delay_Min_EEPROM == 1) {
    Mower_Turn_Delay_Min = EEPROM.read(32);
    Mower_Turn_Delay_Min = Mower_Turn_Delay_Min * 100;
    Serial.print(F("EEPROM: Mower Turn Delay Min: "));
    Serial.println(Mower_Turn_Delay_Min);
  }

  if (Mower_Turn_Delay_Min_EEPROM != 1) {
    Serial.print(F("Settings: Mower Turn Delay Min: "));
    Serial.println(Mower_Turn_Delay_Min);
  }


  int Mower_Turn_Delay_Max_EEPROM = EEPROM.read(33);
  if (Mower_Turn_Delay_Max_EEPROM == 1) {
    Mower_Turn_Delay_Max = EEPROM.read(34);
    Mower_Turn_Delay_Max =  Mower_Turn_Delay_Max * 100; 
    Serial.print(F("EEPROM: Mower Turn Delay Max: "));
    Serial.println(Mower_Turn_Delay_Max);
  }


  if (Mower_Turn_Delay_Max_EEPROM != 1) {
    Serial.print(F("Settings: Mower Turn Delay Max: "));
    Serial.println(Mower_Turn_Delay_Max);
  }


  int Mower_Reverse_Delay_EEPROM = EEPROM.read(35);
  if (Mower_Reverse_Delay_EEPROM == 1) {
    Mower_Reverse_Delay = EEPROM.read(36);
    Mower_Reverse_Delay = (Mower_Reverse_Delay * 100);
    Serial.print(F("EEPROM: Mower Reverse Time Delay/ms: "));
    Serial.println(Mower_Reverse_Delay);
  }

  if (Mower_Reverse_Delay_EEPROM != 1) {
    Serial.print(F("Settings: Mower Reverse Time Delay/ms: "));
    Serial.println(Mower_Reverse_Delay);
  }



  int Track_Wire_Zone_1_Cycles_EEPROM = EEPROM.read(43);
  if (Track_Wire_Zone_1_Cycles_EEPROM == 1) {
    Track_Wire_Zone_1_Cycles = EEPROM.read(44);
    Track_Wire_Zone_1_Cycles = (Track_Wire_Zone_1_Cycles * 100);
    Serial.print(F("EEPROM: Zone 1 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_1_Cycles);
  }

  if (Track_Wire_Zone_1_Cycles_EEPROM != 1) {
    Serial.print(F("Settings: Zone 1 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_1_Cycles);
  }

  int Track_Wire_Zone_2_Cycles_EEPROM = EEPROM.read(45);
  if (Track_Wire_Zone_2_Cycles_EEPROM == 1) {
    Track_Wire_Zone_2_Cycles = EEPROM.read(46);
    Track_Wire_Zone_2_Cycles = (Track_Wire_Zone_2_Cycles * 100);
    Serial.print(F("EEPROM: Zone 2 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_2_Cycles);
  }

  if (Track_Wire_Zone_2_Cycles_EEPROM != 1) {
    Serial.print(F("Settings: Zone 2 Tracking Cycles "));
    Serial.println(Track_Wire_Zone_2_Cycles);
  }


  int Use_Charging_Station_EEPROM = EEPROM.read(47);
  if (Use_Charging_Station_EEPROM == 1) {
    Use_Charging_Station = EEPROM.read(48);  
    Serial.print(F("EEPROM: Charge Station: "));
    if (Use_Charging_Station == 0) Serial.println(F("OFF"));
    if (Use_Charging_Station == 1) Serial.println(F("ON"));
  }

  if (Use_Charging_Station_EEPROM != 1) {
    Serial.print(F("Settings: Charge Station: "));
    if (Use_Charging_Station == 0) Serial.println(F("OFF"));
    if (Use_Charging_Station == 1) Serial.println(F("ON"));
  }


  int CW_Tracking_To_Charge_EEPROM = EEPROM.read(49);
  if (CW_Tracking_To_Charge_EEPROM == 1) {
    CW_Tracking_To_Charge = EEPROM.read(50);  
    Serial.print(F("EEPROM: Tracking Direction to Charge : "));
    if (CW_Tracking_To_Charge == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }


  if (CW_Tracking_To_Charge_EEPROM != 1) {
    Serial.print(F("Settings: Tracking Direction to Charge : "));
    if (CW_Tracking_To_Charge == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }

  int CCW_Tracking_To_Charge_EEPROM = EEPROM.read(51);
  if (CCW_Tracking_To_Charge_EEPROM == 1) {
    CCW_Tracking_To_Charge = EEPROM.read(52);  
    Serial.print(F("EEPROM Tracking Direction to Charge : "));
    if (CCW_Tracking_To_Charge == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }

  if (CCW_Tracking_To_Charge_EEPROM != 1) {
    Serial.print(F("Settings: Tracking Direction to Charge : "));
    if (CCW_Tracking_To_Charge == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Charge == 0) Serial.println(F("OFF"));
  }


  int CW_Tracking_To_Start_EEPROM = EEPROM.read(53);
  if (CW_Tracking_To_Start_EEPROM == 1) {
    CW_Tracking_To_Start = EEPROM.read(54);  
    Serial.print(F("EEPROM: Tracking Direction to Start : "));
    if (CW_Tracking_To_Start == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }

  if (CW_Tracking_To_Start_EEPROM != 1) {
    Serial.print(F("Settings: Tracking Direction to Start : "));
    if (CW_Tracking_To_Start == 1) Serial.println(F("CW"));
    if (CW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }


  int CCW_Tracking_To_Start_EEPROM = EEPROM.read(55);
  if (CCW_Tracking_To_Start_EEPROM == 1) {
    CCW_Tracking_To_Start = EEPROM.read(56);  
    Serial.print(F("EERPOM: Tracking Direction to Start : "));
    if (CCW_Tracking_To_Start == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }

  if (CCW_Tracking_To_Start_EEPROM != 1) {
    Serial.print(F("Settings: Tracking Direction to Start : "));
    if (CCW_Tracking_To_Start == 1) Serial.println(F("CCW"));
    if (CCW_Tracking_To_Start == 0) Serial.println(F("OFF"));
  }

  int Max_Cycles_Straight_EEPROM = EEPROM.read(57);
  if (Max_Cycles_Straight_EEPROM == 1) {
    Max_Cycles_Straight = EEPROM.read(58);
    Max_Cycles_Straight = (Max_Cycles_Straight * 10);
    Serial.print(F("EEPROM: Straight Line Cycles before turn: "));
    Serial.println(Max_Cycles_Straight);
  }

  if (Max_Cycles_Straight_EEPROM != 1) {
    Serial.print(F("Settings: Straight Line Cycles before turn: "));
    Serial.println(Max_Cycles_Straight);
  }

  int Sonar_1_Activate_EEPROM = EEPROM.read(37);
  if (Sonar_1_Activate_EEPROM == 1) {
    Sonar_1_Activate = EEPROM.read(38);
    Serial.print(F("EEPROM: Sonar 1 Activated: "));
    Serial.println(Sonar_1_Activate);
  }

  if (Sonar_1_Activate_EEPROM != 1) {
    Serial.print(F("Settings: Sonar 1 Activated: "));
    Serial.println(Sonar_1_Activate);
  }

  int Sonar_2_Activate_EEPROM = EEPROM.read(39);
  if (Sonar_2_Activate_EEPROM == 1) {
    Sonar_2_Activate = EEPROM.read(40);
    Serial.print(F("EEPROM: Sonar 2 Activated: "));
    Serial.println(Sonar_2_Activate);
  }

  if (Sonar_2_Activate_EEPROM != 1) {
    Serial.print(F("Settings: Sonar 2 Activated: "));
    Serial.println(Sonar_2_Activate);
  }

  int Sonar_3_Activate_EEPROM = EEPROM.read(41);
  if (Sonar_3_Activate_EEPROM == 1) {
    Sonar_3_Activate = EEPROM.read(42);
    Serial.print(F("EEPROM: Sonar 3 Activated: "));
    Serial.println(Sonar_3_Activate);
  }

  if (Sonar_3_Activate_EEPROM != 1) {
    Serial.print(F("Settings: Sonar 3 Activated: "));
    Serial.println(Sonar_3_Activate);
  }

  int Max_Sonar_Hit_EEPROM = EEPROM.read(63);
  if (Max_Sonar_Hit_EEPROM == 1) {
    Max_Sonar_Hit = EEPROM.read(64); 
    Serial.print(F("EEPROM: Sonar Sensitivity: "));
    Serial.println(Max_Sonar_Hit);
  }

  if (Max_Sonar_Hit_EEPROM != 1) {
    Serial.print(F("Settings: Sonar Sensitivity: "));
    Serial.println(Max_Sonar_Hit);
  }

  int maxdistancesonar_EEPROM = EEPROM.read(65);
  if (maxdistancesonar_EEPROM == 1) {
    maxdistancesonar = EEPROM.read(66); 
    Serial.print(F("EEPROM: Sonar Activation Distance: "));
    Serial.println(maxdistancesonar);
  }

  if (maxdistancesonar_EEPROM != 1) {
    Serial.print(F("Settings: Sonar Activation Distance: "));
    Serial.println(maxdistancesonar);
  }

  int Perimeter_Wire_Enabled_EEPROM = EEPROM.read(67);
  if (Perimeter_Wire_Enabled_EEPROM == 1) {
    Perimeter_Wire_Enabled = EEPROM.read(68);  
    Serial.print(F("EEPROM: Wire Module ON/OFF: "));
    if (Perimeter_Wire_Enabled == 0) Serial.println(F("OFF"));
    if (Perimeter_Wire_Enabled == 1) Serial.println(F("ON"));
  }

  if (Perimeter_Wire_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: Wire Module ON/OFF: "));
    if (Perimeter_Wire_Enabled == 0) Serial.println(F("OFF"));
    if (Perimeter_Wire_Enabled == 1) Serial.println(F("ON"));
  }


  int Max_Cycle_Wire_Find_EEPROM = EEPROM.read(69);
  if (Max_Cycle_Wire_Find_EEPROM == 1) {
    Max_Cycle_Wire_Find = EEPROM.read(70);
    Max_Cycle_Wire_Find = (Max_Cycle_Wire_Find * 10);
    Serial.print(F("EEPROM: Track Cycles Forwards to find Wire "));
    Serial.println(Max_Cycle_Wire_Find);
  }


  if (Max_Cycle_Wire_Find_EEPROM != 1) {
    Serial.print(F("Settings: Track Cycles Forwards to find Wire "));
    Serial.println(Max_Cycle_Wire_Find);
  }


  int Max_Cycle_Wire_Find_Back_EEPROM = EEPROM.read(71);
  if (Max_Cycle_Wire_Find_Back_EEPROM == 1) {
    Max_Cycle_Wire_Find_Back = EEPROM.read(72);
    Max_Cycle_Wire_Find_Back = (Max_Cycle_Wire_Find_Back * 10);
    Serial.print(F("EEPROM: Track Cycles Back to find Wire "));
    Serial.println(Max_Cycle_Wire_Find_Back);
  }


  if (Max_Cycle_Wire_Find_Back_EEPROM != 1) {
    Serial.print(F("Settings: Track Cycles Back to find Wire "));
    Serial.println(Max_Cycle_Wire_Find_Back);
  }


  int Max_Tracking_Turn_Right_EEPROM = EEPROM.read(73);
  if (Max_Tracking_Turn_Right_EEPROM == 1) {
    Max_Tracking_Turn_Right = EEPROM.read(74);
    Max_Tracking_Turn_Right = (Max_Tracking_Turn_Right * 10);
    Serial.print(F("EEPROM: Wheel RH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Right);
  }

  if (Max_Tracking_Turn_Right_EEPROM != 1) {
    Serial.print(F("Settings: Wheel RH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Right);
  }



  int Max_Tracking_Turn_Left_EEPROM = EEPROM.read(75);
  if (Max_Tracking_Turn_Left_EEPROM == 1) {
    Max_Tracking_Turn_Left = EEPROM.read(76);
    Max_Tracking_Turn_Left = (Max_Tracking_Turn_Left * 10);
    Serial.print(F("EEPROM: Wheel LH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Left);
  }


  if (Max_Tracking_Turn_Left_EEPROM != 1) {
    Serial.print(F("Settings: Wheel LH Cycles before restart "));
    Serial.println(Max_Tracking_Turn_Left);
  }


  int Rain_Sensor_Installed_EEPROM = EEPROM.read(77);
  if (Rain_Sensor_Installed_EEPROM == 1) {
    Rain_Sensor_Installed = EEPROM.read(78);  
    Serial.print(F("EEPROM: Rain Sensor ON/OFF:  "));
    if (Rain_Sensor_Installed == 0) Serial.println(F("OFF"));
    if (Rain_Sensor_Installed == 1) Serial.println(F("ON"));
  }


  if (Rain_Sensor_Installed_EEPROM != 1) {
    Serial.print(F("Settings: Rain Sensor ON/OFF: "));
    if (Rain_Sensor_Installed == 0) Serial.println(F("OFF"));
    if (Rain_Sensor_Installed == 1) Serial.println(F("ON"));
  }


  int Rain_Total_Hits_Go_Home_EEPROM = EEPROM.read(79);
  if (Rain_Total_Hits_Go_Home_EEPROM == 1) {
    Rain_Total_Hits_Go_Home = EEPROM.read(80); 
    Serial.print(F("Settings: Rain Sensitivity: "));
    Serial.println(Rain_Total_Hits_Go_Home);
  }

  if (Rain_Total_Hits_Go_Home_EEPROM != 1) {
    Serial.print(F("Settings: Rain Sensitivity: "));
    Serial.println(Rain_Total_Hits_Go_Home);
  }

  int WIFI_Enabled_EEPROM = EEPROM.read(81);
  if (WIFI_Enabled_EEPROM == 1) {
    WIFI_Enabled = EEPROM.read(82);  
    Serial.print(F("EEPROM: WIFI Enabled: "));
    if (WIFI_Enabled == 0) Serial.println(F("OFF"));
    if (WIFI_Enabled == 1) Serial.println(F("ON"));
  }

  if (WIFI_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: WIFI Enabled: "));
    if (WIFI_Enabled == 0) Serial.println(F("OFF"));
    if (WIFI_Enabled == 1) Serial.println(F("ON"));
  }

  int Cutting_Blades_Activate_EEPROM = EEPROM.read(83);
  if (Cutting_Blades_Activate_EEPROM == 1) {
    Cutting_Blades_Activate = EEPROM.read(84);  
    Serial.print(F("EEPROM: CUTTING BLADE Motor SET TO : "));
    if (Cutting_Blades_Activate == 0) Serial.println(F("OFF"));
    if (Cutting_Blades_Activate == 1) Serial.println(F("ON"));
  }


  if (Cutting_Blades_Activate_EEPROM != 1) {
    Serial.print(F("Settings: CUTTING BLADE Motor SET TO : "));
    if (Cutting_Blades_Activate == 0) Serial.println(F("OFF"));
    if (Cutting_Blades_Activate == 1) Serial.println(F("ON"));
  }

  int Low_Battery_Instances_Chg_EEPROM = EEPROM.read(85);
  if (Low_Battery_Instances_Chg_EEPROM == 1) {
    Low_Battery_Instances_Chg = EEPROM.read(86); 
    Serial.print(F("EEPROM: Battery to Charge Sensitivity: "));
    Serial.println(Low_Battery_Instances_Chg);
  }

  if (Low_Battery_Instances_Chg_EEPROM != 1) {
    Serial.print(F("Settings: Battery to Charge Sensitivity: "));
    Serial.println(Low_Battery_Instances_Chg);
  }


//  int Bumper_Activate_Frnt_EEPROM = EEPROM.read(90);
//  if (Bumper_Activate_Frnt_EEPROM == 1) {
//    Bumper_Activate_Frnt = EEPROM.read(91);  
//    Serial.print(F("EEPROM: Bumper Bar Enabled: "));
//    if (Bumper_Activate_Frnt == 0) Serial.println(F("OFF"));
//    if (Bumper_Activate_Frnt == 1) Serial.println(F("ON"));
//  }
//
//  if (Bumper_Activate_Frnt_EEPROM != 1) {
//    Serial.print(F("Settings: Bumper Bar Enabled: "));
//    if (Bumper_Activate_Frnt == 0) Serial.println(F("OFF"));
//    if (Bumper_Activate_Frnt == 1) Serial.println(F("ON"));
//  }


  int Turn_90_Delay_LH_EEPROM = EEPROM.read(101);
  if (Turn_90_Delay_LH_EEPROM == 1) {
    Turn_90_Delay_LH = EEPROM.read(102);  
    Turn_90_Delay_LH = Turn_90_Delay_LH * 10;
    Serial.print(F("EEPROM: Turn_90_Delay_LH Enabled: "));
    Serial.println(Turn_90_Delay_LH);
  }

  if (Turn_90_Delay_LH_EEPROM != 1) {
    Serial.print(F("Settings: Turn_90_Delay_LH Enabled: "));
    Serial.println(Turn_90_Delay_LH);
  }


  int Turn_90_Delay_RH_EEPROM = EEPROM.read(103);
  if (Turn_90_Delay_RH_EEPROM == 1) {
    Turn_90_Delay_RH = EEPROM.read(104);  
    Turn_90_Delay_RH = Turn_90_Delay_RH * 10;
    Serial.print(F("EEPROM: Turn_90_Delay_RH Enabled: "));
    Serial.println(Turn_90_Delay_RH);
  }

  if (Turn_90_Delay_RH_EEPROM != 1) {
    Serial.print(F("Settings: Turn_90_Delay_RH Enabled: "));
    Serial.println(Turn_90_Delay_RH);
  }


  int Line_Length_Cycles_EEPROM = EEPROM.read(105);
  if (Line_Length_Cycles_EEPROM == 1) {
    Line_Length_Cycles = EEPROM.read(106);  
    Line_Length_Cycles = Line_Length_Cycles * 10;
    Serial.print(F("EEPROM: Line_Length_Cycles Enabled: "));
    Serial.println(Line_Length_Cycles);
  }


  if (Line_Length_Cycles_EEPROM != 1) {
    Serial.print(F("Settings: Line_Length_Cycles Enabled: "));
    Serial.println(Line_Length_Cycles);
  }



  int GPS_Enabled_EEPROM = EEPROM.read(107);
  if (GPS_Enabled_EEPROM == 1) {
    GPS_Enabled = EEPROM.read(108);  
    Serial.print(F("EEPROM: GPS: "));
    if (GPS_Enabled == 0) Serial.println(F("Disabled"));
    if (GPS_Enabled == 1) Serial.println(F("Enabled"));
  }

  if (GPS_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: GPS: "));
    if (GPS_Enabled == 0) Serial.println(F("Disabled"));
    if (GPS_Enabled == 1) Serial.println(F("Enabled"));
  }

 int GPS_Type_EEPROM = EEPROM.read(125);
  if (GPS_Type_EEPROM == 1) {
    GPS_Type = EEPROM.read(126);  
    Serial.print(F("EEPROM: GPS_Type: "));
    if (GPS_Type == 1) Serial.println(F("ReP_AL"));
    if (GPS_Type == 2) Serial.println(F("PIXHAWK"));
    if (GPS_Type == 3) Serial.println(F("SPARE"));
  }

  if (GPS_Type_EEPROM != 1) {
    Serial.print(F("Settings: GPS_Type: "));
    if (GPS_Type == 1) Serial.println(F("ReP_AL"));
    if (GPS_Type == 2) Serial.println(F("PIXHAWK"));
    if (GPS_Type == 3) Serial.println(F("SPARE"));
  }

  int GYRO_Enabled_EEPROM = EEPROM.read(109);
  if (GYRO_Enabled_EEPROM == 1) {
    GYRO_Enabled = EEPROM.read(110);  
    Serial.print(F("EEPROM: GYRO Enabled: "));
    if (GYRO_Enabled == 0) Serial.println(F("Disabled"));
    if (GYRO_Enabled == 1) Serial.println(F("Enabled"));
  }

  if (GYRO_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: GYRO Enabled: "));
    if (GYRO_Enabled == 0) Serial.println(F("Disabled"));
    if (GYRO_Enabled == 1) Serial.println(F("Enabled"));
  }

  
  int GPower_EEPROM = EEPROM.read(111);
  if (GPower_EEPROM == 1) {
    GPower = EEPROM.read(112); 
    GPower = GPower / 10; 
    Serial.print(F("EEPROM: GYRO PID : "));
    Serial.println(GPower);
  }

  if (GPower_EEPROM != 1) {
    Serial.print(F("Settings: GYRO PID : "));
    Serial.println(GPower);
  }

  int Wheel_Amp_Sensor_ON_Enabled_EEPROM = EEPROM.read(115);
  if (Wheel_Amp_Sensor_ON_Enabled_EEPROM == 1) {
    Wheel_Amp_Sensor_ON = EEPROM.read(116);  
    Serial.print(F("EEPROM: Wheel Block Amp Sensor Enabled: "));
    if (Wheel_Amp_Sensor_ON == 0) Serial.println(F("Disabled"));
    if (Wheel_Amp_Sensor_ON == 1) Serial.println(F("Enabled"));
  }


  if (Wheel_Amp_Sensor_ON_Enabled_EEPROM != 1) {
    Serial.print(F("Settings: Wheel Block Amp Sensor Enabled: "));
    if (Wheel_Amp_Sensor_ON == 0) Serial.println(F("Disabled"));
    if (Wheel_Amp_Sensor_ON == 1) Serial.println(F("Enabled"));
  }

  
  int Max_Wheel_Amps_EEPROM = EEPROM.read(117);
  if (Max_Wheel_Amps_EEPROM == 1) {
    Max_Wheel_Amps = EEPROM.read(118); 
    Max_Wheel_Amps = Max_Wheel_Amps / 10; 
    Serial.print(F("EEPROM: Wheel Block Amps: "));
    Serial.println(Max_Wheel_Amps);
  }

  if (Max_Wheel_Amps_EEPROM != 1) {
    Max_Wheel_Amps = Max_Wheel_Amps; 
    Serial.print(F("Settings: Wheel Block Amps: "));
    Serial.println(Max_Wheel_Amps);
  }

  int Wheels_Enabled_EEPROM = EEPROM.read(123);
  if (Wheels_Enabled_EEPROM == 1) {
  Wheels_Activate = EEPROM.read(124);
    Serial.print(F("Wheels Enabled from EEPROM : "));
      if (Wheels_Activate == 1) Serial.println(F("Enabled"));
      if (Wheels_Activate == 0) Serial.println(F("Disabled"));
  }
  
  if (Wheels_Enabled_EEPROM != 1) {
    Serial.print(F("Wheels Enabled from EEPROM : "));
      if (Wheels_Activate == 1) Serial.println(F("Enabled"));
      if (Wheels_Activate == 0) Serial.println(F("Disabled"));
  }
  
  if (Boost_Turn == 1) {
    Serial.println(F("Settings: Boost Turn Activated"));
    Serial.print(F("Min Track PWM = "));
    Serial.println(Min_Track_PWM);
    Serial.print(F("Boost Time = "));
    Serial.println(Hard_Track_Turn_Delay);
        
  }
  if (Boost_Turn == 0) Serial.print(F("Settings: Boost Turn OFF"));

  Serial.println(F(""));






Serial.println(F("*********** DONE **************"));

#endif
}


void Clear_EERPOM_Data() {

  #if defined(BOARD_MEGA)

  Serial.println(F(""));
  Serial.println(F("CLEARING EEPROM Data"));
  Serial.println(F(""));
  
  EEPROM.write(1,0);      // Clear Alarm 1
  EEPROM.write(5,0);      // Clear Alarm 2
  EEPROM.write(9,0);      // Clear Alarm 3
  EEPROM.write(13,0);     // Clear PWM Left Wheel
  EEPROM.write(15,0);     // Clear PWM Right Wheel
  EEPROM.write(17,0);     // Clear PWM Blade
  EEPROM.write(19,0);     // Clear Compass Setting EEPROM
  EEPROM.write(21,0);     // Clear PID Setting
  EEPROM.write(23,0);     // Clear Pattern Mow
  EEPROM.write(25,0);     // Clear Volt Minimum
  EEPROM.write(27,0);     // Clear Compass Home
  EEPROM.write(29,0);     // Clear Tilt Tip Safety
  EEPROM.write(31,0);     // Clear Turn Time Min
  EEPROM.write(33,0);     // Clear Turn Time Max
  EEPROM.write(35,0);     // Clear Reverse Time
  EEPROM.write(37,0);     // Clear Sonar 1
  EEPROM.write(39,0);     // Clear Sonar 2
  EEPROM.write(41,0);     // Clear Sonar 3
  EEPROM.write(43,0);     // Clear Zone 1 Cycles
  EEPROM.write(45,0);     // Clear Zone 2 Cycles
  EEPROM.write(47,0);     // Clear Charging Station Options
  EEPROM.write(49,0);     // Reset CW and CCW Exit and Dock Directions
  EEPROM.write(51,0);     //  CW CCW Tracking
  EEPROM.write(53,0);     //  CW CCW Tracking
  EEPROM.write(55,0);     //  CW CCW Tracking
  EEPROM.write(57,0);     // Max Cycle Straight
  EEPROM.write(59,0);     // Compass Heading HOld ON/OFF
  EEPROM.write(61,0);     // Compass PID reset.
  EEPROM.write(63,0);     // Sonar sensitivity.
  EEPROM.write(65,0);      // MAx distance sonar reset.
  EEPROM.write(67,0);     // Wire Sensor ON/OFF
  EEPROM.write(69,0);     // Track cycles Forwards
  EEPROM.write(71,0);     // Track Cycles Back
  EEPROM.write(73,0);     // RH Cycles to restart
  EEPROM.write(75,0);     // LH Cycles to restart
  EEPROM.write(77,0);     // Rain ON/OFF
  EEPROM.write(79,0);     // Rain sensitivity
  EEPROM.write(81,0);     // WIFI ON/OFF
  EEPROM.write(83,0);     // Cutting Blades ON/OFF
  EEPROM.write(85,0);     // Batt sensitivity;
  EEPROM.write(87,0);     // Alarm Actions 1-3
  EEPROM.write(88,0);
  EEPROM.write(89,0);
//  EEPROM.write(90,0);     // Bumper Bar
  EEPROM.write(92,0);     // Tip Over sensor
  EEPROM.write(94,0);     // Wheel Slow Speed LH
  EEPROM.write(96,0);     // Wheel Slow Speed RH
  EEPROM.write(98,0);     // Slow MAG Point
  EEPROM.write(102,0);    // Turn_90_Delay_LH 
  EEPROM.write(104,0);    // Turn_90_Delay_RH
  EEPROM.write(106,0);    // Line_Length_Cycles
  EEPROM.write(107,0);    // GPS Enabled
  EEPROM.write(109,0);    // GYRO Enabled
  EEPROM.write(111,0);    // GYRO Power
  EEPROM.write(113,0);    // Compass Setup Mode
  EEPROM.write(115,0);    // Wheel amp sensor 
  EEPROM.write(117,0);    // Wheel amp sensor value
  EEPROM.write(119,0);    // PCB
  EEPROM.write(121,0);    // Robot Type
  EEPROM.write(123,0);    // Wheel ON/OFF
  EEPROM.write(125,0);    // GPS_Type
  
  Serial.println(F(""));
  Serial.println(F("EEPROM CLEARED"));
  Serial.println(F(""));
  
  #endif
  
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// GPS
// GPS Module Code
// The ESP32 sends send 3.3V on the output pin when the mower is inside the GPS fence.
// when the mower leaves the GPS fence the pin is at 0 volts.
// The pin A7 is read by the adcman feature as analog.read cant be used anymore.


void Check_GPS_In_Out() {

// Run the ADCman to get the pin values as per the settings
ADCMan.run();
GPS_Fence_Signal = ADCMan.read(GPS_Fence_Signal_Pin);    // Read GPS in out status
GPS_Lock_Signal  = ADCMan.read(GPS_Lock_Pin);            // Read GPS Fixed status

// Print the value to the serial monitor
Serial.print("|GPS:");
Serial.print(GPS_Fence_Signal);
Serial.print("/");

if (GPS_Fence_Signal < 100) GPS_Inside_Fence = 0;   // Mower is Outside the GPS Fence
if (GPS_Fence_Signal > 100) GPS_Inside_Fence = 1;   // Mower is Inside the GPS Fence

    if (GPS_Inside_Fence == 0) Serial.print(":OUT");
    if (GPS_Inside_Fence == 1) Serial.print(":IN");

//Serial.print("|Lock:");
//Serial.print(GPS_Lock_Signal);
//Serial.print("/");

if (GPS_Lock_Signal < 100)  GPS_Lock_OK = 0;   // Mower is Outside the GPS Fence
if (GPS_Lock_Signal > 100)  GPS_Lock_OK = 1;   // Mower is Inside the GPS Fence
    
   if (GPS_Lock_OK == 0) Serial.print(":NoLOCK");
   if (GPS_Lock_OK == 1) Serial.print(":RTKFIX");
        
        // if there is no GPS Lock and the mower is running then keep within the whiole loop
        // until a GPS RTK lock is found.
        while ((GPS_Lock_OK == 0) && (Mower_Running == 1)) {          
          Motor_Action_Stop_Motors();                               // Stop Wheels
          Motor_Action_Stop_Spin_Blades();                          // Stop Blade
          Serial.println(F(""));
          Serial.print(F("- Checking for GPS Lock "));          
          Serial.print(F("| Lock:"));
          ADCMan.run();
          GPS_Lock_Signal = ADCMan.read(GPS_Lock_Pin);
          Serial.println(GPS_Lock_Signal); 
          delay(100);
          if (GPS_Lock_Signal < 50)  GPS_Lock_OK = 0;   // Mower is Outside the GPS Fence
          if (GPS_Lock_Signal > 50)  GPS_Lock_OK = 1;   // Mower is Inside the GPS Fence 
          if (TFT_Screen_Menu == 1) {
            if (Robot_Type == 1) Send_Mower_Running_Data();
//            if (Robot_Type == 2) Send_Aerator_Running_Data();
            }
          delay(1000);
          }
          
    
    Serial.print("|"); 
  
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// GYRO
void Get_GYRO_Reading() {
         Wire.beginTransmission(MPU_addr);
         Wire.write(0x3B);
         //Wire.endTransmission(false);                 // try to remove the false
         Wire.endTransmission(true); 
         Wire.requestFrom(MPU_addr,7*2,true);

         if (Wire.available() <= 14) {         
             AcX = Wire.read()<<8 | Wire.read();
             AcY = Wire.read()<<8 | Wire.read();
             AcZ = Wire.read()<<8 | Wire.read();
             Temp = Wire.read()<<8 | Wire.read();
             GyX = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
             GyY = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
             GyZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x47    
             } 
             
         int xAng = map(AcX,minVal,maxVal,-90,90);
         int yAng = map(AcY,minVal,maxVal,-90,90);
         int zAng = map(AcZ,minVal,maxVal,-90,90);
              
         GYRO_Angle_X = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
         GYRO_Angle_Y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
         GYRO_Angle_Z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
        
         Heading = atan2(yAng, xAng);
                
         Serial.print("|GX=");
         Serial.print(GYRO_Angle_X);
         Serial.print("|GY=");
         Serial.print(GYRO_Angle_Y);             
         Serial.print("|GZ=");
         Serial.print(GYRO_Angle_Z);
         //delay(100);
         } 

void Print_GYRO_Reading() {              
         Serial.print(" | tmp = ");    Serial.print(Temp/340.00+36.53);
         Serial.print(" | Gyro X = "); Serial.print(convert_int16_to_str(GyX));
         Serial.print(" | Gyro Y = "); Serial.print(convert_int16_to_str(GyY));
         Serial.print(" | Gyro Z = "); Serial.println(convert_int16_to_str(GyZ));
         delay(100);
         } 


void Calculate_GYRO_Wheel_Compensation() {

  int GYRO_Error_X = 0;
  
  if (GYRO_Angle_X > 180) GYRO_Error_X = (360 - GYRO_Angle_X);        // Calculates the error in compass heading from the saved lock heading
  if (GYRO_Angle_X <= 180) GYRO_Error_X = GYRO_Angle_X;        // Calculates the error in compass heading from the saved lock heading

  
  // Mower is tipping to the Left
  // Left Wheel needs to be at full power and less power to the right wheel to steer right.
  if (GYRO_Angle_X > 180) {                                             // Steer Right
     Serial.print(F("GR|"));
    
    
    // With no adjustment of wheel speed according to MAG Level    
    if (MAG_Speed_Adjustment == 0) {
      PWM_Left = PWM_MaxSpeed_LH;         
      PWM_Right = PWM_MaxSpeed_RH - (GPower * GYRO_Error_X);            // Multiply the difference by D to increase the power then subtract from the PWM
      if (PWM_Right < 0) PWM_Right = PWM_MaxSpeed_RH - 50;
      }
    
    GYRO_Steering_Status = 2;
    } 




  // Mower is tipping to the Right  
  // Right Wheel needs to be at full power and less power to the left wheel to steer left.
  if (GYRO_Angle_X <= 180) {      
    Serial.print(F("GL|"));

    // With no adjustment of wheel speed according to MAG Level
    if (MAG_Speed_Adjustment == 0) {
      PWM_Right = PWM_MaxSpeed_RH; 
      PWM_Left = PWM_MaxSpeed_LH - (GPower * GYRO_Error_X);            // Multiply the difference by D to increase the power then subtract from the PWM
      if (PWM_Left < 0) PWM_Left = PWM_MaxSpeed_LH - 50;
      }
    

    GYRO_Steering_Status = 3;
    }

}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Beta
// BETA SETTINGS MENU


// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_BETA(byte LCD_Menu_BETA) {
  #if defined(LCD_KEYPAD)
  if (LCD_Menu_BETA == 1) lcd.print(F("Tilt Test"));
  if (LCD_Menu_BETA == 2) lcd.print(F("Test"));
  if (LCD_Menu_BETA == 3) lcd.print(F("Test"));          // needs writing
  Max_Options_BETA = 3;
  #endif
  }
  


void Print_Membrane_Switch_Input_BETA() {
     #if defined(LCD_KEYPAD)
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

      Serial.println();
      Serial.println(F("BETA Menu Activated"));
      Menu_Complete = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
      
      while (Menu_Complete == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_BETA(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_BETA(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete = true;
          Serial.println(F("Start key is pressed"));
          Activate_Menu_Option_BETA();
          lcd.clear();
          
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_BETA();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_BETA();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit BETA");
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          Print_Membrane_Switch_Input_Settings();
          }
      }
    //Activate_Menu_Option_BETA();
    #endif
    }

    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_BETA() {
     #if defined(LCD_KEYPAD)
     
     if (Menu_View > Max_Options_BETA) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_BETA(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_BETA( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     #endif
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_BETA() {

      #if defined(LCD_KEYPAD)
      if (Menu_Mode_Selection == 3) {
        
        lcd.clear();
        lcd.print("Tilt Test");
        
        Serial.println(F("Tilt Test Selected"));
        Menu_Mode_Selection = 0;
        delay(2000);
        lcd.clear();
        
        //Calibrate_Compass_Angle();
        Menu_Complete = false;
          delay(100);
          while (Menu_Complete == false) {
          // insert Test Code Here
          Read_Membrane_Keys();
          Check_Tilt_Tip_Angle();
          
             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Test Stopped");
             delay(2000);
             lcd.clear();                     
             Menu_Mode_Selection = 0;
             }
         }
      
      }

     
Print_Membrane_Switch_Input_BETA();

#endif
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Docked
// BUTTONS DOCKED MENU



// Reads each of the membrane keys and detects if a key is pressed.
void Read_Membrane_Keys(){
  #if defined(LCD_KEYPAD)
  Start_Key_X   = digitalRead(Start_Key);    
  Plus_Key_X    = digitalRead(Plus_Key);   
  Minus_Key_X   = digitalRead(Minus_Key);
  Stop_Key_X    = digitalRead(Stop_Key);  
  #endif
  }


// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Docked(byte LCD_Menu_Docked) {
  #if defined(LCD_KEYPAD)
  if (LCD_Menu_Docked == 1) lcd.print(F("Exit Dock Z-1"));
  if (LCD_Menu_Docked == 2) lcd.print(F("Exit Dock Z-2"));
  if (LCD_Menu_Docked == 3) lcd.print(F("Quick Start"));
  if (LCD_Menu_Docked == 4) lcd.print(F("Trampoline Cut"));
  if (LCD_Menu_Docked == 5) lcd.print(F("Mow the Line"));
  if (LCD_Menu_Docked == 6) lcd.print(F("Test Mower"));
  if (LCD_Menu_Docked == 7) lcd.print(F("Setup Mower"));
  if (LCD_Menu_Docked == 8) lcd.print(F("-- Spare 8 --"));
  if (LCD_Menu_Docked == 9) lcd.print(F("-- Spare 9 --"));
  if (LCD_Menu_Docked == 10) lcd.print(F("-- Spare 10 --"));
  if (LCD_Menu_Docked == 11) lcd.print(F("-- Spare 11 --"));
  if (LCD_Menu_Docked == 12) lcd.print(F("-- Spare 12 --"));
  Max_Options_Docked = 12;
  #endif
  }


void Check_Membrane_Switch_Input_Docked() {
  #if defined(LCD_KEYPAD)
  //Menu Options if the Mower is Docked
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

    
    if(!Start_Key_X)  {                                    // If the start key is pressed
        Serial.println();
        Serial.println(F("Start Key Pressed"));
        Menu_Complete = false;                                // Menu complete will return to the normal loop
        lcd.clear();
        delay(5);
        Serial.println();
        Serial.println(F("Docked Menu Activated"));
        delay(500);
        lcd.clear();
        delay(5);
 

      while (Menu_Complete == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Docked(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Docked(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete = true;
          Serial.println(F("Start key is pressed"));
          lcd.clear();
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Docked();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Docked();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Menu Cancelled");
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    Activate_Menu_Option_Docked();
    }
    #endif
}
    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_Docked() {
     #if defined(LCD_KEYPAD)
     if (Menu_View > Max_Options_Docked) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Docked(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Docked( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     #endif
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_Docked() {
     #if defined(LCD_KEYPAD)
     if (Menu_Mode_Selection == 1) {
       // Exit the mower from the Garage and go to Zone 1;
       lcd.clear();
       lcd.print("Manual Start");
       lcd.setCursor(0,1);
       lcd.print("Exit Dock Z1");
       Serial.println(F("Exit to Zone 1 - Free Mow"));
       delay(1000);
       lcd.clear();
       Print_Membrane_Switch_Input_Timing();
       Menu_Mode_Selection = 0;
       delay(1000);
       lcd.clear();
       if (Mow_Time_Set == 1) {
          //Enter here the code to go to zone 1 from dock
          Exit_Zone = 1;
          Track_Wire_Itterations = Track_Wire_Zone_1_Cycles;
          Manouver_Exit_To_Zone_X();    
          }
     }
 
     
     if (Menu_Mode_Selection == 2) {
       // Exit the mower from the Garage and go to Zone 2;
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.print("Manual Start");
       lcd.setCursor(0,1);
       lcd.print("Exit Dock Z2");
       delay(1000);
       lcd.clear();
       Print_Membrane_Switch_Input_Timing();                            // Changes the menu to select the mow time
       Menu_Mode_Selection = 0;
       delay(1000);
       lcd.clear();
       if (Mow_Time_Set == 1) {
          Exit_Zone = 2;
          Track_Wire_Itterations = Track_Wire_Zone_2_Cycles;
          Manouver_Exit_To_Zone_X();
          }
     }

     if (Menu_Mode_Selection == 3) {
        // Quick Start the Mower in the middle of the Garden;
        Serial.println("Quick Start Selected");
        Print_Membrane_Switch_Input_Timing();                             // Changes the menu to select the mow time
        Menu_Mode_Selection = 0;
        delay(1000);
        if (Mow_Time_Set == 1) Manouver_Start_Mower();
        lcd.clear();     
        }
        
   
     if (Menu_Mode_Selection == 4) {
        lcd.clear();
        lcd.print("Trampoline Cut!");
        Serial.println(F("Mower Set to Cut under Trampoline"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        Special_Cut_Under_Trampoline_Function();
        }
          
      if (Menu_Mode_Selection == 5) {
       lcd.clear();
       lcd.print("Blade will spin");
       lcd.setCursor(0,1);
       lcd.print("Mow the Line");
       delay(1000);
       lcd.clear();
       Print_Membrane_Switch_Input_Timing();                             // Changes the menu to select the mow time
       if (Mow_Time_Set == 1) {
         Serial.println(F("Cutting the grass on the boundary wire"));
         Blade_Override = 1;
         Track_Wire_Itterations = 6000;
         Exit_Zone = 3;    
         Manouver_Exit_To_Zone_X();
         Manouver_Start_Mower();                                  // Sets up the mower to go.
         }
        }

      if (Menu_Mode_Selection == 6) {
        lcd.clear();
        lcd.print("Test Mower Menu");
        Serial.println(F("Test Menu Selected"));
        Menu_Mode_Selection = 0;
        delay(1000);
        lcd.clear();
        Print_Membrane_Switch_Input_Tests();
        }
      if (Menu_Mode_Selection == 7) {
        lcd.clear();
        lcd.print("Mower Setup");
        Serial.println(F("Mower Setup Selected"));
        Menu_Mode_Selection = 0;
        delay(1000);
        lcd.clear();
        Print_Membrane_Switch_Input_Settings();
        }  
   #endif 
  }
  

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Motion
// Motion SETTINGS MENU

#if defined(LCD_KEYPAD)

// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Motion(byte LCD_Menu_Motion) {
  if (LCD_Menu_Motion == 1) lcd.print(F("Wheel Speed"));
  if (LCD_Menu_Motion == 2) lcd.print(F("Blade Speed"));  //+need to add ON/OFF
  if (LCD_Menu_Motion == 3) lcd.print(F("Blades ON/OFF"));
  if (LCD_Menu_Motion == 4) lcd.print(F("Turn Angles"));
  if (LCD_Menu_Motion == 5) lcd.print(F("Reverse Dist"));
  if (LCD_Menu_Motion == 6) lcd.print(F("Max Length"));
  if (LCD_Menu_Motion == 7) lcd.print(F("Pattern Mow"));
  if (LCD_Menu_Motion == 8) lcd.print(F("Wheel ON/OFF")); // RVES added

  
  Max_Options_Motion = 8;
  }


void Print_Membrane_Switch_Input_Motion() {
     Read_Membrane_Keys();
     Menu_Complete_Motion = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

      Serial.println();
      Serial.println(F("Motion Menu Activated"));
      Menu_Complete_Motion = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
      
      while (Menu_Complete_Motion == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Motion(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Motion(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete_Motion = true;
          Serial.println(F("Start key is pressed"));
          Activate_Menu_Option_Motion();
          lcd.clear();
          
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Motion();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Motion();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete_Motion = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit Motion");
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    }

    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_Motion() {
     if (Menu_View > Max_Options_Motion) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Motion(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Motion( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_Motion() {


   if (Menu_Mode_Selection == 1) {
       // LH and RH wheel SPEED PWM settings
     
       int Set = 1;
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Set Wheel PWM"));
       lcd.setCursor(0,1);
       lcd.print(F("LH & RH Wheels"));
       delay(1000);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("> LH_PWM: "));
       lcd.print(PWM_MaxSpeed_LH);
       lcd.setCursor(0,1);
       lcd.print(F("  RH_PWM: "));
       lcd.print(PWM_MaxSpeed_RH);
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 2) { 
               Serial.println(F("Settings Saved"));
               Menu_Complete_Motion = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Wheel PWM"));
               lcd.setCursor(0,1);
               lcd.print(F("SAVED"));
               delay(2000);
               lcd.clear();          
               EEPROM.write(13, 1);
               EEPROM.write(14, PWM_MaxSpeed_LH);
               EEPROM.write(15, 1);
               EEPROM.write(16, PWM_MaxSpeed_RH);
               Menu_Mode_Selection = 0;
               }
              
             if (Set == 1) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F("  LH_PWM: "));
                lcd.print(PWM_MaxSpeed_LH);
                lcd.setCursor(0,1);
                lcd.print(F("> RH_PWM: "));
                lcd.print(PWM_MaxSpeed_RH);   
                Set = Set + 1;          
                }


             }
             if (!Plus_Key_X) {
               if (Set == 1) {
                 PWM_MaxSpeed_LH = PWM_MaxSpeed_LH + 1;
                 if (PWM_MaxSpeed_LH > 255) PWM_MaxSpeed_LH = 255;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("> LH_PWM: "));
                 lcd.print(PWM_MaxSpeed_LH);
                 lcd.setCursor(0,1);
                 lcd.print(F("  RH_PWM: "));
                 lcd.print(PWM_MaxSpeed_RH);
                 }
               if (Set == 2) {
                 PWM_MaxSpeed_RH = PWM_MaxSpeed_RH + 1;
                 if (PWM_MaxSpeed_RH > 255) PWM_MaxSpeed_RH = 255;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("  LH_PWM: "));
                 lcd.print(PWM_MaxSpeed_LH);
                 lcd.setCursor(0,1);
                 lcd.print(F("> RH_PWM: "));
                 lcd.print(PWM_MaxSpeed_RH);       
                 }
               }
             if (!Minus_Key_X) {
               if (Set == 1) {
                 PWM_MaxSpeed_LH = PWM_MaxSpeed_LH - 1;
                 if (PWM_MaxSpeed_LH < 0) PWM_MaxSpeed_LH = 0;
                 lcd.setCursor(0,0);
                 lcd.setCursor(0,0);
                 lcd.print(F("> LH_PWM: "));
                 lcd.print(PWM_MaxSpeed_LH);
                 lcd.setCursor(0,1);
                 lcd.print(F("  RH_PWM: "));
                 lcd.print(PWM_MaxSpeed_RH);
                 }
               if (Set == 2) {
                 PWM_MaxSpeed_RH = PWM_MaxSpeed_RH - 1;
                 if (PWM_MaxSpeed_RH < 0) PWM_MaxSpeed_RH = 0;
                 lcd.setCursor(0,0);
                 lcd.setCursor(0,0);
                 lcd.print(F("  LH_PWM: "));
                 lcd.print(PWM_MaxSpeed_LH);
                 lcd.setCursor(0,1);
                 lcd.print(F("> RH_PWM: "));
                 lcd.print(PWM_MaxSpeed_RH);       
                 }
               }
     
             
     }
     }




     if (Menu_Mode_Selection == 2) {
       // Blade SPEED PWM Settings
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Blade Speed:"));
       lcd.setCursor(0,1);
       lcd.print(PWM_Blade_Speed);
       Serial.print(F("Blade PWM:"));
       Serial.println(PWM_Blade_Speed);
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Motion = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Blade PWM:"));
             lcd.print(PWM_Blade_Speed);
             lcd.setCursor(0,1);
             lcd.print("SAVED");
             delay(2000);
             lcd.clear();          
             EEPROM.write(17, 1);
             EEPROM.write(18, PWM_Blade_Speed);   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               PWM_+_Speed = PWM_Blade_Speed + 1;
               if (PWM_Blade_Speed > 255) PWM_Blade_Speed = 255;
               lcd.setCursor(0,1);
               lcd.print(F("      "));    // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print(PWM_Blade_Speed);
               Serial.print(F("Blade PWM:"));
               Serial.println(PWM_Blade_Speed);
               }
             if (!Minus_Key_X) {
               PWM_Blade_Speed = PWM_Blade_Speed - 1;
               if (PWM_Blade_Speed < 0) PWM_Blade_Speed = 0;
               lcd.setCursor(0,1);
               lcd.print(F("      "));   // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print(PWM_Blade_Speed);
               Serial.print(F("Blade PWM:"));
               Serial.println(PWM_Blade_Speed);
               }
             
             }
     }
        

   if (Menu_Mode_Selection == 3) {
       // Cutting Blades ON/OFF
       lcd.clear();
       lcd.print(F("Cutting Blades"));
       lcd.setCursor(0,1);
       lcd.print(F("ON/OFF "));       
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Cutting Blades"));
       lcd.setCursor(0,1);
       lcd.print(F("Status: "));
       if (Cutting_Blades_Activate == 1) lcd.print(F("ON "));
       if (Cutting_Blades_Activate == 0) lcd.print(F("OFF"));
       
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Cutting Blades ON/OFF Settings Saved"));
               Menu_Complete_Motion = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Cutting Blades"));
               lcd.setCursor(0,1);
               lcd.print(F("Saved: "));
               if (Cutting_Blades_Activate == 1) lcd.print(F("ON "));
               if (Cutting_Blades_Activate == 0) lcd.print(F("OFF"));
               Serial.print(F("Status:"));
               Serial.println(Cutting_Blades_Activate);
               delay(2000);
               lcd.clear();          
               EEPROM.write(83 , 1);
               EEPROM.write(84 , Cutting_Blades_Activate);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status: "));
               Cutting_Blades_Activate = 1;
               lcd.print(F("ON "));
               Serial.print(F("Status:"));
               Serial.println(Cutting_Blades_Activate);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status: "));
               Cutting_Blades_Activate = 0;
               lcd.print(F("OFF"));
               Serial.print(F("Status:"));
               Serial.println(Cutting_Blades_Activate);
               delay(100);
               }
     }
     }

     if (Menu_Mode_Selection == 4) {
       // Turn Time / Angles at Wire: Sets the turning time when the mower changes direction
     
       int Set = 1;
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Set Turn time"));
       lcd.setCursor(0,1);
       lcd.print(F("Min/Max ms"));
       delay(1000);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("> Min/ms:"));
       lcd.print(Mower_Turn_Delay_Min);
       lcd.setCursor(0,1);
       lcd.print(F("  Max/ms:"));
       lcd.print(Mower_Turn_Delay_Max);
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 2) { 
               Serial.println(F("Settings Saved"));
               Menu_Complete_Motion = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Min Max Turn"));
               lcd.setCursor(0,1);
               lcd.print(F("SAVED"));
               delay(2000);
               lcd.clear();          
               EEPROM.write(31, 1);
               EEPROM.write(32, (Mower_Turn_Delay_Min/100));
               EEPROM.write(33, 1);
               EEPROM.write(34, (Mower_Turn_Delay_Max/100));
               Menu_Mode_Selection = 0;
               }
              
             if (Set == 1) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F("  Min/ms:"));
                lcd.print(Mower_Turn_Delay_Min);
                lcd.setCursor(0,1);
                lcd.print(F("> Max/ms:"));
                lcd.print(Mower_Turn_Delay_Max);   
                Set = Set + 1;          
                }


             }
             if (!Plus_Key_X) {
               if (Set == 1) {
                 Mower_Turn_Delay_Min = Mower_Turn_Delay_Min + 100;
                 if (Mower_Turn_Delay_Min >= Mower_Turn_Delay_Max) {
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("Min must be");
                  lcd.setCursor(0,1);
                  lcd.print("lower than max");
                  Mower_Turn_Delay_Min = Mower_Turn_Delay_Max - 100;
                  delay(1000);
                  lcd.clear();
                  }
                 lcd.setCursor(0,0);
                 lcd.setCursor(0,0);
                 lcd.print(F("> Min/ms:"));
                 lcd.print(Mower_Turn_Delay_Min);
                 lcd.setCursor(0,1);
                 lcd.print(F("  Max/ms:"));
                 lcd.print(Mower_Turn_Delay_Max);
                 }
               if (Set == 2) {
                 Mower_Turn_Delay_Max = Mower_Turn_Delay_Max + 100;
                 lcd.setCursor(0,0);
                 lcd.setCursor(0,0);
                 lcd.print(F("  Min/ms:"));
                 lcd.print(Mower_Turn_Delay_Min);
                 lcd.setCursor(0,1);
                 lcd.print(F("> Max/ms:"));
                 lcd.print(Mower_Turn_Delay_Max);          
                 }
               }
             if (!Minus_Key_X) {
               if (Set == 1) {
                 Mower_Turn_Delay_Min = Mower_Turn_Delay_Min - 100;
                   if (Mower_Turn_Delay_Min < 1000) lcd.clear();     
                   if (Mower_Turn_Delay_Min <= 0) {
                     lcd.clear();
                     Mower_Turn_Delay_Min = 100;      // cant be less than zero.
                     }
                 lcd.setCursor(0,0);
                 lcd.print(F("> Min/ms:"));
                 lcd.print(Mower_Turn_Delay_Min);
                 lcd.setCursor(0,1);
                 lcd.print(F("  Max/ms:"));
                 lcd.print(Mower_Turn_Delay_Max);
                 }
               if (Set == 2) {
                 Mower_Turn_Delay_Max = Mower_Turn_Delay_Max - 100;
                 if (Mower_Turn_Delay_Max < 1000) lcd.clear();
                 if (Mower_Turn_Delay_Max <= Mower_Turn_Delay_Min) {     // Max vant be less than min Value
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("Max must be");
                  lcd.setCursor(0,1);
                  lcd.print("Higher than Min");
                  Mower_Turn_Delay_Max = Mower_Turn_Delay_Min + 100;
                  delay(1000);
                  lcd.clear();
                 }
                 lcd.setCursor(0,0);
                 lcd.print(F("  Min/ms:"));
                 lcd.print(Mower_Turn_Delay_Min);
                 lcd.setCursor(0,1);
                 lcd.print(F("> Max/ms:"));
                 lcd.print(Mower_Turn_Delay_Max);          
                 }
               }
     
             
     }
     }


     if (Menu_Mode_Selection == 5) {
       // Mower Reverse Time
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Reverse Time/ms:"));
       lcd.setCursor(0,1);
       lcd.print(Mower_Reverse_Delay);
       Serial.print(F("Mower Reverse Time /ms:"));
       Serial.println(Mower_Reverse_Delay);
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Motion = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Reverse Time/ms:"));
             lcd.print(Mower_Reverse_Delay);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(35, 1);
             EEPROM.write(36, (Mower_Reverse_Delay/100));
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Mower_Reverse_Delay = Mower_Reverse_Delay + 100;
               lcd.setCursor(0,0);
               lcd.print(F("Reverse Time/ms:"));
               lcd.setCursor(0,1);
               lcd.print(Mower_Reverse_Delay);
               }
             if (!Minus_Key_X) {
               Mower_Reverse_Delay = Mower_Reverse_Delay - 100;
               if (Mower_Reverse_Delay <= 1000) lcd.clear(); 
               if (Mower_Reverse_Delay <= 0) Mower_Reverse_Delay = 100;
               lcd.setCursor(0,0);
               lcd.print(F("Reverse Time/ms:"));
               lcd.setCursor(0,1);
               lcd.print(Mower_Reverse_Delay);
               }
             
             }
     }


      if (Menu_Mode_Selection == 6) {
       // Straight Line Distance before Automatic Turn
       // Counts the number of loops ran.  If the max number of loops are ran the mower
       // stops and turns around anyway.
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Distance Max:"));
       lcd.setCursor(0,1);
       lcd.print("Before Turn");
       delay(1000);
       lcd.clear();
       lcd.setCursor(0,1);
       lcd.print(F("Max Loops: "));
       lcd.print(Max_Cycles_Straight);
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
             Read_Membrane_Keys();
             delay(100);
           //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Settings Saved"));
               Menu_Complete_Motion = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("SAVED"));
               lcd.setCursor(0,1);
               lcd.print(F("Max Loops: "));
               lcd.print(Max_Cycles_Straight);
               delay(2000);
               lcd.clear();          
               EEPROM.write(57, 1);
               EEPROM.write(58, (Max_Cycles_Straight / 10));
  
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Max_Cycles_Straight = Max_Cycles_Straight + 10;
               lcd.clear();
               lcd.setCursor(0,1);
               lcd.print(F("Max Loops: "));
               lcd.print(Max_Cycles_Straight);
               }
             if (!Minus_Key_X) {
               Max_Cycles_Straight = Max_Cycles_Straight - 10;
               if (Max_Cycles_Straight < 10) Max_Cycles_Straight = 10;
               lcd.clear();
               lcd.setCursor(0,1);
               lcd.print(F("Max Loops: "));
               lcd.print(Max_Cycles_Straight);
               }
             
             }
     }



     
      if (Menu_Mode_Selection == 7) {
       // Pattern Mow Setup
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Pattern Mow"));
       lcd.setCursor(0,1);
       lcd.print(F("Setting"));
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Pattern ON/OFF"));
       lcd.setCursor(0,1);
       lcd.print("Status : ");
       if (Pattern_Mow == 1) lcd.print(F("Parallel"));
       if (Pattern_Mow == 2) lcd.print(F("Spiral  "));
       if (Pattern_Mow == 0) lcd.print(F("OFF"));
       
       Menu_Complete_Motion = false;
       while (Menu_Complete_Motion == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Compass Settings Saved"));
               Menu_Complete_Motion = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print("Pattern Saved");
               lcd.setCursor(0,1);
               if (Pattern_Mow == 0) {
                Pattern_Mow = 0;
                lcd.print(F("OFF"));
               }
               if (Pattern_Mow == 1) {
                lcd.print(F("Parallel"));
                Pattern_Mow = 1;
               }
               if (Pattern_Mow == 2) {
                lcd.print(F("Spiral  "));
                Pattern_Mow = 2;
                Spiral_Mow = 1;
               }
               Serial.print(F("Pattern Mow:"));
               Serial.println(Pattern_Mow);
               delay(2000);
               lcd.clear();          
               EEPROM.write(23 , 1);
               EEPROM.write(24 , Pattern_Mow);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status:");
               Pattern_Mow = Pattern_Mow + 1;
               if (Pattern_Mow > 2) Pattern_Mow = 2;
               if (Pattern_Mow == 1) lcd.print("Parallel");
               if (Pattern_Mow == 2) lcd.print("Spiral  ");  
               Serial.print(F("Pattern Mow:"));
               Serial.println(Pattern_Mow);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Pattern_Mow = 0;
               lcd.print("OFF       ");
               Serial.print(F("Pattern Mow:"));
               Serial.println(Pattern_Mow);
               delay(100);
               }
     }
     }

      // RVES added
      if (Menu_Mode_Selection == 8) {
          // Wheels ON/OFF
          lcd.clear();
          lcd.print(F("Wheels"));
          lcd.setCursor(0,1);
          lcd.print(F("ON/OFF "));
          delay(1000);
          lcd.clear();
          Menu_Mode_Selection = 0;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(F("Wheels"));
          lcd.setCursor(0,1);
          lcd.print(F("Status: "));
          if (Wheels_Activate == 1) lcd.print(F("ON "));
          if (Wheels_Activate == 0) lcd.print(F("OFF"));

          Menu_Complete_Motion = false;
          while (Menu_Complete_Motion == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
                if(!Start_Key_X){
                  Serial.println(F("Wheels ON/OFF Settings Saved"));
                  Menu_Complete_Motion = true;
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print(F("Wheels"));
                  lcd.setCursor(0,1);
                  lcd.print(F("Saved: "));
                  if (Wheels_Activate == 1) lcd.print(F("ON "));
                  if (Wheels_Activate == 0) lcd.print(F("OFF"));
                  Serial.print(F("Status:"));
                  Serial.println(Wheels_Activate);
                  delay(2000);
                  lcd.clear();
                  EEPROM.write(123 , 1);
                  EEPROM.write(124 , Wheels_Activate);
                  Menu_Mode_Selection = 0;

                  }
                if (!Plus_Key_X) {
                  lcd.setCursor(0,1);
                  lcd.print(F("Status: "));
                  Wheels_Activate = 1;
                  lcd.print(F("ON "));
                  Serial.print(F("Status:"));
                  Serial.println(Wheels_Activate);
                  delay(100);
                  }
                if (!Minus_Key_X) {
                  lcd.setCursor(0,1);
                  lcd.print(F("Status: "));
                  Wheels_Activate = 0;
                  lcd.print(F("OFF"));
                  Serial.print(F("Status:"));
                  Serial.println(Wheels_Activate);
                  delay(100);
                  }
        }
        }


if (Menu_Complete_Motion == true) Print_Membrane_Switch_Input_Motion();  

}

#endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Mowing_Time
// Test to displyed on the LCD screen when using the membrane key menus

// BUTTONS TIMING MENU

#if defined(LCD_KEYPAD)

void Print_LCD_Menu_Timing(byte LCD_Menu_Timing) {
  if (LCD_Menu_Timing == 1) lcd.print(F("Max Mow Time"));
  if (LCD_Menu_Timing == 2) lcd.print(F("1hr Mow Time"));
  if (LCD_Menu_Timing == 3) lcd.print(F("2hr Mow Time"));
  if (LCD_Menu_Timing == 4) lcd.print(F("  "));                  //leave blank
  Max_Options_Timing = 4;
 }  



void Print_Membrane_Switch_Input_Timing() {

  //Menu Options if the Mower is Timing.
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

    
      Serial.println();
      Serial.println(F("Test Menu Activated"));
      Menu_Complete = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
 

      while (Menu_Complete == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Timing(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Timing(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete = true;
          Serial.println(F("Start key is pressed"));
          lcd.clear();
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Timing();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Timing();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(F("Menu Cancelled"));
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    Activate_Menu_Option_Timing();
    }

 void Run_Menu_Order_Timing() {
     if (Menu_View > Max_Options_Timing) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Timing(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Timing( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }


void Activate_Menu_Option_Timing() {
     
     if (Menu_Mode_Selection == 1) {
       // Maximum Mower Timing
       lcd.clear();
       lcd.print(F("Max Mow"));
       lcd.setCursor(0,1);
       lcd.print(F("Selected"));
       Serial.println(F("Maximum Mow Time Selected"));
       delay(5000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       Menu_Complete = false;
       // Add actions here
       Alarm_Timed_Mow_ON = 0;
       Mow_Time_Set = 1;
       }
     
 
     
     if (Menu_Mode_Selection == 2) {
       // 1hr Mowing
       Serial.println(F("1 hr Mow Time Selected"));
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       Menu_Complete = false;

           // Ations Here
           if (PCB == 0) {Time t = rtc.time();}
           if (PCB == 1) Display_DS3231_Time();
           Alarm_Timed_Mow_ON = 1;                          // Activate the Mow Timer Alarm
           Alarm_Timed_Mow_Hour = Time_Hour + 1;                 // Sets time to 1 hour later.
           Alarm_Timed_Mow_Minute = Time_Minute;                  // Minutes are the same

           // Displays the Finish time on the Serial Monitor
           Serial.print(F("Finish Time set to : "));
           Serial.print(Alarm_Timed_Mow_Hour);
           Serial.print(F(":"));
           if (Alarm_Timed_Mow_Minute < 10) Serial.print ("0");
           Serial.println(Alarm_Timed_Mow_Minute);
           
           lcd.print("1hr Mow Selected");
           lcd.setCursor(0,1);
           lcd.print("Ends: ");
           lcd.print(Alarm_Timed_Mow_Hour);
           lcd.print(":");
          
           if (Alarm_Timed_Mow_Minute < 10) lcd.print("0");
           lcd.print(Alarm_Timed_Mow_Minute);
           
           Mow_Time_Set = 1;
           delay(2000);
       }
    

     if (Menu_Mode_Selection == 3) {
       // 2hr Mowing
       Serial.println(F("2 hr Mow Time Selected"));
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       Menu_Complete = false;

           // Ations Here
           if (PCB == 0) {Time t = rtc.time();}
           if (PCB == 1) Display_DS3231_Time();
           Alarm_Timed_Mow_ON = 1;                          // Activate the Mow Timer Alarm
           Alarm_Timed_Mow_Hour = Time_Hour + 2;                 // Sets time to 1 hour later.
           Alarm_Timed_Mow_Minute = Time_Minute;                  // Minutes are the same

           // Displays the Finish time on the Serial Monitor
           Serial.print(F("Finish Time set to : "));
           Serial.print(Alarm_Timed_Mow_Hour);
           Serial.print(F(":"));
           if (Alarm_Timed_Mow_Minute < 10) Serial.print ("0");
           Serial.println(Alarm_Timed_Mow_Minute);
           
           lcd.print(F("2hr Mow Selected"));
           lcd.setCursor(0,1);
           lcd.print(F("Ends: "));
           lcd.print(Alarm_Timed_Mow_Hour);
           lcd.print(":");
           if (Alarm_Timed_Mow_Minute < 10) lcd.print("0");
           lcd.print(Alarm_Timed_Mow_Minute);
           Mow_Time_Set = 1;
           delay(2000);
       }
        
   
  }

  #endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_NAVI
// NAVI SETTINGS MENU

#if defined(LCD_KEYPAD)

// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_NAVI(byte LCD_Menu_NAVI) {
  if (LCD_Menu_NAVI == 1) lcd.print(F("Compass ON/OFF"));
  if (LCD_Menu_NAVI == 2) lcd.print(F("Compass Home Dir"));
  if (LCD_Menu_NAVI == 3) lcd.print(F("H-Hold ON/OFF"));         
  if (LCD_Menu_NAVI == 4) lcd.print(F("Compass PID")); 
  if (LCD_Menu_NAVI == 5) lcd.print(F("Gyro ON/OFF")); // RVES added
  if (LCD_Menu_NAVI == 6) lcd.print(F("Gyro PID"));    // RVES added
  if (LCD_Menu_NAVI == 7) lcd.print(F("")); 
  if (LCD_Menu_NAVI == 8) lcd.print(F("")); 
  if (LCD_Menu_NAVI == 9) lcd.print(F("")); 
  if (LCD_Menu_NAVI == 10) lcd.print(F("")); 
  Max_Options_NAVI = 10;
  }


void Print_Membrane_Switch_Input_NAVI() {
     Read_Membrane_Keys();
     Menu_Complete_NAVI = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

      Serial.println();
      Serial.println(F("NAVI Menu Activated"));
      Menu_Complete_NAVI = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
      
      while (Menu_Complete_NAVI == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_NAVI(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_NAVI(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete_NAVI = true;
          Serial.println(F("Start key is pressed"));
          Activate_Menu_Option_NAVI();
          lcd.clear();
          
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_NAVI();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_NAVI();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete_NAVI = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit NAVI");
          delay(400);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    }

    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_NAVI() {
     if (Menu_View > Max_Options_NAVI) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_NAVI(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_NAVI( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_NAVI() {


   if (Menu_Mode_Selection == 1) {
       // Compass setup
       lcd.clear();
       lcd.print(F("Compass Setup"));
       delay(400);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Compass ON/OFF"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Compass_Activate == 1) lcd.print(F("ON "));
       if (Compass_Activate == 0) lcd.print(F("OFF"));
       
       Menu_Complete_NAVI = false;
       while (Menu_Complete_NAVI == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Compass Settings Saved"));
               Menu_Complete_NAVI = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Compass Saved"));
               Serial.print(F("Compass:"));
               Serial.println(Compass_Activate);
               delay(2000);
               lcd.clear();          
               EEPROM.write(19 , 1);
               EEPROM.write(20 , Compass_Activate);
               if (Compass_Activate == 1)   {
                   if (Compass_Setup_Mode == 1) Setup_DFRobot_QMC5883_HMC5883L_Compass();   // USes the DFRobot Library
                   if (Compass_Setup_Mode == 2) Setup_DFRobot_QMC5883_HMC5883L_Compass();   // Uses manual i2C address
                   if (Compass_Setup_Mode == 3) Setup_DFRobot_QMC5883_HMC5883L_Compass();   // Uses QMC5883L Library               
                   Menu_Mode_Selection = 0;
                   }
             }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Compass_Activate = 1;
               lcd.print(F("ON "));
               Serial.print(F("Compass:"));
               Serial.println(Compass_Activate);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Compass_Activate = 0;
               lcd.print(F("OFF"));
               Serial.print(F("Compass:"));
               Serial.println(Compass_Activate);
               delay(100);
               }
     }
     }


     if (Menu_Mode_Selection == 2) {
       // Compass Home Degrees
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Compass Home Degrees:"));
       lcd.setCursor(0,1);
       lcd.print(Home_Wire_Compass_Heading);
       Serial.print(F("Compass Home Degrees:"));
       Serial.println(Home_Wire_Compass_Heading);
       Menu_Complete_NAVI = false;
       while (Menu_Complete_NAVI == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_NAVI = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Compass Home Degrees:"));
             lcd.print(Home_Wire_Compass_Heading);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(27, 1);
             EEPROM.write(28, (Home_Wire_Compass_Heading/10));   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Home_Wire_Compass_Heading = Home_Wire_Compass_Heading + 10;
               if (Home_Wire_Compass_Heading > 360) Home_Wire_Compass_Heading = 0;
               lcd.setCursor(0,1);
               lcd.print("      ");    // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print(Home_Wire_Compass_Heading);
               Serial.print(F("Compass Home Degrees:"));
               Serial.println(Home_Wire_Compass_Heading);
               }
             if (!Minus_Key_X) {
               Home_Wire_Compass_Heading = Home_Wire_Compass_Heading - 10;
               if (Home_Wire_Compass_Heading < 0) Home_Wire_Compass_Heading = 360;
               lcd.setCursor(0,1);
               lcd.print("      ");   // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print(Home_Wire_Compass_Heading);
               Serial.print(F("Compass Home Degrees : "));
               Serial.println(Home_Wire_Compass_Heading);
               }
             
             }
     }



   if (Menu_Mode_Selection == 3) {
       // Heading Hold ON/OFF
       lcd.clear();
       lcd.print(F("Heading Hold Setup"));
       delay(400);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("H-Hold ON/OFF"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Compass_Heading_Hold_Enabled == 1) lcd.print(F("ON "));
       if (Compass_Heading_Hold_Enabled == 0) lcd.print(F("OFF"));
       
       Menu_Complete_NAVI = false;
       while (Menu_Complete_NAVI == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Heading Hold Settings Saved"));
               Menu_Complete_NAVI = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("H-Hold Saved"));
               Serial.print(F("Heading Hold:"));
               Serial.println(Compass_Heading_Hold_Enabled);
               delay(2000);
               lcd.clear();          
               EEPROM.write(59 , 1);
               EEPROM.write(60 , Compass_Heading_Hold_Enabled);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Compass_Heading_Hold_Enabled = 1;
               lcd.print(F("ON "));
               Serial.print(F("H-Hold:"));
               Serial.println(Compass_Heading_Hold_Enabled);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Compass_Heading_Hold_Enabled = 0;
               lcd.print(F("OFF"));
               Serial.print(F("H-Hold:"));
               Serial.println(Compass_Heading_Hold_Enabled);
               delay(100);
               }
     }
     }



     if (Menu_Mode_Selection == 4) {
       // Compass PID Settings
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Compass PID:"));
       lcd.setCursor(0,1);
       lcd.print(F("P = "));
       lcd.print(CPower);
       Serial.print(F("Compass PID P = :"));
       Serial.println(CPower);
       Menu_Complete_NAVI = false;
       while (Menu_Complete_NAVI == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_NAVI = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Compass P : ");
             lcd.print(CPower);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(61, 1);
             EEPROM.write(62, (CPower*10));   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               CPower = CPower + 0.01;
               if (CPower > 10) CPower = 10;
               lcd.clear();
               lcd.print(F("Compass PID:"));
               lcd.setCursor(0,1);
               lcd.print(F("P = "));
               lcd.print(CPower);
               Serial.print(F("Compass PID P = :"));
               Serial.println(CPower);
               }
             if (!Minus_Key_X) {
               CPower = CPower - 0.01;
               if (CPower < 0) CPower = 0.1;
               lcd.clear();
               lcd.print(F("Compass PID:"));
               lcd.setCursor(0,1);
               lcd.print(F("P = "));
               lcd.print(CPower);
               Serial.print(F("Compass PID P = :"));
               Serial.println(CPower);
               }
             
             }
     }

     if (Menu_Mode_Selection == 5) {
         // Gyro ON/OFF
         lcd.clear();
         lcd.print(F("Gyro Setup"));
         delay(400);
         lcd.clear();
         Menu_Mode_Selection = 0;
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print(F("Gyro ON/OFF"));
         lcd.setCursor(0,1);
         lcd.print(F("Status : "));
         if (GYRO_Enabled == 1) lcd.print(F("ON "));
         if (GYRO_Enabled == 0) lcd.print(F("OFF"));

         Menu_Complete_NAVI = false;
         while (Menu_Complete_NAVI == false) {
            Read_Membrane_Keys();
            delay(100);
            //Enter Code Here to Cycle until stop key is pressed.
               if(!Start_Key_X){
                 Serial.println(F("Gyro Settings Saved"));
                 Menu_Complete_NAVI = true;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Gyro Saved"));
                 Serial.print(F("Gyro:"));
                 Serial.println(GYRO_Enabled);
                 delay(2000);
                 lcd.clear();
                 EEPROM.write(109 , 1);
                 EEPROM.write(110 , GYRO_Enabled);
                 Menu_Mode_Selection = 0;

                 }
               if (!Plus_Key_X) {
                 lcd.setCursor(0,1);
                 lcd.print(F("Status : "));
                 GYRO_Enabled = 1;
                 lcd.print(F("ON "));
                 Serial.print(F("Gyro:"));
                 Serial.println(GYRO_Enabled);
                 delay(100);
                 }
               if (!Minus_Key_X) {
                 lcd.setCursor(0,1);
                 lcd.print(F("Status : "));
                 GYRO_Enabled = 0;
                 lcd.print(F("OFF"));
                 Serial.print(F("Gyro:"));
                 Serial.println(GYRO_Enabled);
                 delay(100);
                 }
       }
       }

     if (Menu_Mode_Selection == 6) {
       // Gyro PID Settings
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Gyro PID:"));
       lcd.setCursor(0,1);
       lcd.print(F("P = "));
       lcd.print(GPower);
       Serial.print(F("Gyro PID P = :"));
       Serial.println(GPower);
       Menu_Complete_NAVI = false;
       while (Menu_Complete_NAVI == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_NAVI = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Gyro P : ");
             lcd.print(GPower);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();
             EEPROM.write(111, 1);
             EEPROM.write(112, (GPower*100));
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               GPower = GPower + 0.01;
               if (GPower > 10) GPower = 10;
               lcd.clear();
               lcd.print(F("Gyro PID:"));
               lcd.setCursor(0,1);
               lcd.print(F("P = "));
               lcd.print(GPower);
               Serial.print(F("Gyro PID P = :"));
               Serial.println(GPower);
               }
             if (!Minus_Key_X) {
               GPower = GPower - 0.01;
               if (GPower < 0) GPower = 0.1;
               lcd.clear();
               lcd.print(F("Gyro PID:"));
               lcd.setCursor(0,1);
               lcd.print(F("P = "));
               lcd.print(GPower);
               Serial.print(F("Gyro PID P = :"));
               Serial.println(GPower);
               }

             }
     }

if (Menu_Complete_NAVI == true) Print_Membrane_Switch_Input_NAVI();   
}

#endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Parked
// BUTTONS PARKED MENU




// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Parked(byte LCD_Menu_Parked) {
  #if defined(LCD_KEYPAD)
  if (LCD_Menu_Parked == 1)   lcd.print("Mow Grass     ");
  if (LCD_Menu_Parked == 2)   {
    if (Use_Charging_Station == 1)  lcd.print(F("Go To Dock"));
    if (Use_Charging_Station == 0)  lcd.print(F("Dock Removed"));
    }
  if (LCD_Menu_Parked == 3)   lcd.print(F("Trampoline Cut"));
  if (LCD_Menu_Parked == 4)   lcd.print(F("Test Mower"));
  if (LCD_Menu_Parked == 5)   lcd.print(F("Setup Mower"));
  if (LCD_Menu_Parked == 6)   lcd.print(F(""));    // Leave Blank
  Max_Options_Parked = 6;
  #endif
  }
  

 void Check_Membrane_Switch_Input_Parked() {
  #if defined(LCD_KEYPAD)
  //Menu Options if the Mower is Parked.
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

     if(!Start_Key_X)  {                                    // If the start key is pressed
        Serial.println();
        Serial.println(F("Start Key Pressed"));
        Menu_Complete = false;                                // Menu complete will return to the normal loop
        lcd.clear();
        delay(5);
        Serial.println();
        Serial.println(F("Parked Menu Activated"));
        delay(500);

 

      while (Menu_Complete == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Parked(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Parked(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete = true;
          Serial.println(F("Start key is pressed"));
          lcd.clear();
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Parked();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Parked();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Menu Cancelled");
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    Activate_Menu_Option_Parked();
    }
    #endif
 }

 void Run_Menu_Order_Parked() {
     #if defined(LCD_KEYPAD)
     if (Menu_View > Max_Options_Parked) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Parked(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Parked( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     #endif
     }


// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_Parked() {
     #if defined(LCD_KEYPAD)
     if (Menu_Mode_Selection == 1) {
      Menu_Mode_Selection = 0;
      lcd.clear();
      Print_Membrane_Switch_Input_Timing();
      lcd.clear();
      lcd.print(F("Mow Re-Starting"));
      Serial.println(F("Mower Starting"));
      delay(2000);
      Manouver_Start_Mower();                                             // Restarts the mower again from standing position
      lcd.clear();     
      }

     if (Menu_Mode_Selection == 2) {
      if (Use_Charging_Station == 1) {
          lcd.clear();
          lcd.print(F("Returning Home"));
          Serial.println(F("Sending Mower Home"));
          delay(100);
          Menu_Mode_Selection = 0;                                      // Releases the loop in the membrane button section.
          delay(1000);
          lcd.clear();
          Manouver_Go_To_Charging_Station();        
          }
      if (Use_Charging_Station == 0) {
          lcd.clear();
          lcd.print(F("No Dock Active"));
          Serial.println(F("Activate Docking Station in Settings"));
          delay(100);
          Menu_Mode_Selection = 0;                                      // Releases the loop in the membrane button section.
          delay(1000);
          lcd.clear();
          }
     }
      
      if (Menu_Mode_Selection == 3) {
        lcd.clear();
        lcd.print("Trampoline Cut!");
        Serial.println(F("Mower Set to Cut under Trampoline"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        Special_Cut_Under_Trampoline_Function();
        }

      if (Menu_Mode_Selection == 4) {
        lcd.clear();
        lcd.print("Test Mower Menu");
        Serial.println(F("Test Menu Selected"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        Print_Membrane_Switch_Input_Tests();
        }

      if (Menu_Mode_Selection == 5) {
        lcd.clear();
        lcd.print("Setup Mower");
        Serial.println(F("Mower Setup Selected"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        Print_Membrane_Switch_Input_Settings();
        }

      if (Menu_Mode_Selection == 6) {
        lcd.clear();
        lcd.print("Slot 6 - Empty");
        Serial.println(F("Slot 6 Selected"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        // Insert Function Here();
        }
  #endif
  }


void Check_Membrane_Keys_Running() {
     #if defined(LCD_KEYPAD)
     Read_Membrane_Keys();
     if(!Stop_Key_X){
        Manouver_Park_The_Mower();
        Check_Membrane_Switch_Input_Parked();   
        Menu_Mode_Selection = 0;
        }

     // RVES added another page with running data
     if(!Plus_Key_X){

     }

     if(!Minus_Key_X){

     }
     #endif
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Sensors
// SENSORS SETTINGS MENU

#if defined(LCD_KEYPAD)


// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Sensors(byte LCD_Menu_Sensors) {
  if (LCD_Menu_Sensors == 1)  lcd.print(F("SONAR ON/OFF"));
  if (LCD_Menu_Sensors == 2)  lcd.print(F("SONAR Distance"));
  if (LCD_Menu_Sensors == 3)  lcd.print(F("SONAR Sensitivity"));   
  if (LCD_Menu_Sensors == 4)  lcd.print(F("Battery Min/Max"));      // add max
  if (LCD_Menu_Sensors == 5)  lcd.print(F("Batt Sensitivity"));
  if (LCD_Menu_Sensors == 6)  lcd.print(F("WIRE ON/OFF"));   
  if (LCD_Menu_Sensors == 7)  lcd.print(F("RAIN ON/OFF"));
  if (LCD_Menu_Sensors == 8)  lcd.print(F("Rain Sensitivity"));
  if (LCD_Menu_Sensors == 9)  lcd.print(F("WIFI ON/OFF"));
//  if (LCD_Menu_Sensors == 10)  lcd.print(F("Bumper ON/OFF"));
  if (LCD_Menu_Sensors == 11) lcd.print(F("ANGLE ON/OFF"));
  if (LCD_Menu_Sensors == 12)  lcd.print(F("TIP ON/OFF"));
  if (LCD_Menu_Sensors == 13)  lcd.print(F("Wheel Amps ON/OFF")); // RVES added
  if (LCD_Menu_Sensors == 14)  lcd.print(F("Wheel Amps limit"));  // RVES added
  if (LCD_Menu_Sensors == 15) lcd.print(F(""));
  
  Max_Options_Sensors = 15;
  }


void Print_Membrane_Switch_Input_Sensors() {
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

      Serial.println();
      Serial.println(F("Sensors Menu Activated"));
      Menu_Complete_Sensors = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
      
      while (Menu_Complete_Sensors == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Sensors(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Sensors(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete_Sensors = true;
          Serial.println(F("Start key is pressed"));
          Activate_Menu_Option_Sensors();
          lcd.clear();
          
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Sensors();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Sensors();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete_Sensors = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit Sensors");
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    }

    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_Sensors() {
     if (Menu_View > Max_Options_Sensors) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Sensors(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Sensors(Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_Sensors() {

     if (Menu_Mode_Selection == 1) {
       // SONAR ON/OFF
     
       int Set = 1;
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("SONAR Active Y/N"));
       lcd.setCursor(0,1);
       lcd.print(F(">S2:"));
       if (Sonar_2_Activate == 1) lcd.print("Y");
       if (Sonar_2_Activate == 0) lcd.print("N");
       lcd.print(F(" S1:"));
       if (Sonar_1_Activate == 1) lcd.print("Y");
       if (Sonar_1_Activate == 0) lcd.print("N");
       lcd.print(F(" S3:"));
       if (Sonar_3_Activate == 1) lcd.print("Y");
       if (Sonar_3_Activate == 0) lcd.print("N");
       delay(2000);

       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 3) {
               lcd.clear(); 
               lcd.setCursor(0,0);
               lcd.print(F("SONAR"));
               lcd.setCursor(0,1);
               lcd.print(F("Settings Saved"));
               delay(2000);            
               EEPROM.write(37, 1);
               EEPROM.write(38, Sonar_1_Activate);
               EEPROM.write(39, 1);
               EEPROM.write(40, Sonar_2_Activate);
               EEPROM.write(41, 1);
               EEPROM.write(42, Sonar_3_Activate);
               Menu_Mode_Selection = 0;
               Menu_Complete_Sensors = true;
               }

             
             if (Set == 2) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F("Y = Yes  N = No"));
                lcd.setCursor(0,1);
                lcd.print(F(" S2:"));
                if (Sonar_2_Activate == 1) lcd.print("Y");
                if (Sonar_2_Activate == 0) lcd.print("N");
                lcd.print(F(" S1:"));
                if (Sonar_1_Activate == 1) lcd.print("Y");
                if (Sonar_1_Activate == 0) lcd.print("N");
                lcd.print(F(">S3:"));
                if (Sonar_3_Activate == 1) lcd.print("Y");
                if (Sonar_3_Activate == 0) lcd.print("N");
                Set = Set + 1;         
               }
            
             if (Set == 1) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F("Y = Yes  N = No"));
                lcd.setCursor(0,1);
                lcd.print(F(" S2:"));
                if (Sonar_2_Activate == 1) lcd.print("Y");
                if (Sonar_2_Activate == 0) lcd.print("N");
                lcd.print(F(">S1:"));
                if (Sonar_1_Activate == 1) lcd.print("Y");
                if (Sonar_1_Activate == 0) lcd.print("N");
                lcd.print(F(" S3:"));
                if (Sonar_3_Activate == 1) lcd.print("Y");
                if (Sonar_3_Activate == 0) lcd.print("N");
                Set = Set + 1;         
               }
             }

             if (!Plus_Key_X) {
               if (Set == 1) {
                 Sonar_2_Activate = 1;
                 if (Sonar_2_Activate > 1) Sonar_2_Activate = 1;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Y = Yes  N = No"));
                 lcd.setCursor(0,1);
                 lcd.print(F(">S2:"));
                 if (Sonar_2_Activate == 1) lcd.print("Y");
                 if (Sonar_2_Activate == 0) lcd.print("N");
                 lcd.print(F(" S1:"));
                 if (Sonar_1_Activate == 1) lcd.print("Y");
                 if (Sonar_1_Activate == 0) lcd.print("N");
                 lcd.print(F(" S3:"));
                 if (Sonar_3_Activate == 1) lcd.print("Y");
                 if (Sonar_3_Activate == 0) lcd.print("N");
                 }
               if (Set == 2) {
                 Sonar_1_Activate = 1;
                 if (Sonar_1_Activate > 1) Sonar_1_Activate = 1;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Y = Yes  N = No"));
                 lcd.setCursor(0,1);
                 lcd.print(F(" S2:"));
                 if (Sonar_2_Activate == 1) lcd.print("Y");
                 if (Sonar_2_Activate == 0) lcd.print("N");
                 lcd.print(F(">S1:"));
                 if (Sonar_1_Activate == 1) lcd.print("Y");
                 if (Sonar_1_Activate == 0) lcd.print("N");
                 lcd.print(F(" S3:"));
                 if (Sonar_3_Activate == 1) lcd.print("Y");
                 if (Sonar_3_Activate == 0) lcd.print("N");
                 }
               if (Set == 3) {
                 Sonar_3_Activate = 1;
                 if (Sonar_3_Activate > 1) Sonar_3_Activate = 1;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Y = Yes  N = No"));
                 lcd.setCursor(0,1);
                 lcd.print(F(" S2:"));
                 if (Sonar_2_Activate == 1) lcd.print("Y");
                 if (Sonar_2_Activate == 0) lcd.print("N");
                 lcd.print(F(" S1:"));
                 if (Sonar_1_Activate == 1) lcd.print("Y");
                 if (Sonar_1_Activate == 0) lcd.print("N");
                 lcd.print(F(">S3:"));
                 if (Sonar_3_Activate == 1) lcd.print("Y");
                 if (Sonar_3_Activate == 0) lcd.print("N");
                 }
             }
             
             if (!Minus_Key_X) {
               if (Set == 1) {
                 Sonar_2_Activate = 0;
                 if (Sonar_2_Activate < 0) Sonar_2_Activate = 0;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Y = Yes  N = No"));
                 lcd.setCursor(0,1);
                 lcd.print(F(">S2:"));
                 if (Sonar_2_Activate == 1) lcd.print("Y");
                 if (Sonar_2_Activate == 0) lcd.print("N");
                 lcd.print(F(" S1:"));
                 if (Sonar_1_Activate == 1) lcd.print("Y");
                 if (Sonar_1_Activate == 0) lcd.print("N");
                 lcd.print(F(" S3:"));
                 if (Sonar_3_Activate == 1) lcd.print("Y");
                 if (Sonar_3_Activate == 0) lcd.print("N");
                 }
               if (Set == 2) {
                 Sonar_1_Activate = 0;
                 if (Sonar_1_Activate < 0) Sonar_1_Activate = 0;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Y = Yes  N = No"));
                 lcd.setCursor(0,1);
                 lcd.print(F(" S2:"));
                 if (Sonar_2_Activate == 1) lcd.print("Y");
                 if (Sonar_2_Activate == 0) lcd.print("N");
                 lcd.print(F(">S1:"));
                 if (Sonar_1_Activate == 1) lcd.print("Y");
                 if (Sonar_1_Activate == 0) lcd.print("N");
                 lcd.print(F(" S3:"));
                 if (Sonar_3_Activate == 1) lcd.print("Y");
                 if (Sonar_3_Activate == 0) lcd.print("N");
                 }
               if (Set == 3) {
                 Sonar_3_Activate = 0;
                 if (Sonar_3_Activate < 0) Sonar_3_Activate = 0;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("Y = Yes  N = No"));
                 lcd.setCursor(0,1);
                 lcd.print(F(" S2:"));
                 if (Sonar_2_Activate == 1) lcd.print("Y");
                 if (Sonar_2_Activate == 0) lcd.print("N");
                 lcd.print(F(" S1:"));
                 if (Sonar_1_Activate == 1) lcd.print("Y");
                 if (Sonar_1_Activate == 0) lcd.print("N");
                 lcd.print(F(">S3:"));
                 if (Sonar_3_Activate == 1) lcd.print("Y");
                 if (Sonar_3_Activate == 0) lcd.print("N");
               }         
           }
     }
     }

     if (Menu_Mode_Selection == 2) {
       // SONAR Distance
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Sonar Distance:"));
       lcd.setCursor(0,1);
       lcd.print(F("cm = "));
       lcd.print(maxdistancesonar);
       Serial.print(F("Sonar Distnace = :"));
       Serial.println(maxdistancesonar);
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Sensors = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("cm = "));
             lcd.print(maxdistancesonar);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(65, 1);
             EEPROM.write(66, maxdistancesonar);   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               maxdistancesonar = maxdistancesonar + 1;
               if (maxdistancesonar> 100) maxdistancesonar = 100;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Sonar Distance:"));
               lcd.setCursor(0,1);
               lcd.print(F("cm = "));
               lcd.print(maxdistancesonar);
               Serial.print(F("Sonar Distnace = :"));
               Serial.println(maxdistancesonar);
               }
             if (!Minus_Key_X) {
               maxdistancesonar = maxdistancesonar - 1;
               if (maxdistancesonar < 10) maxdistancesonar = 10;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Sonar Distance:"));
               lcd.setCursor(0,1);
               lcd.print(F("cm = "));
               lcd.print(maxdistancesonar);
               Serial.print(F("Sonar Distnace = :"));
               Serial.println(maxdistancesonar);
               }
             
             }
     }



     if (Menu_Mode_Selection == 3) {
       // SONAR Sensitivity
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Sonar Sensitivity:"));
       lcd.setCursor(0,1);
       lcd.print(F("Hits = "));
       lcd.print(Max_Sonar_Hit);
       Serial.print(F("Sonar Sensitivity = :"));
       Serial.println(Max_Sonar_Hit);
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Sensors = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Hits : ");
             lcd.print(Max_Sonar_Hit);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(63, 1);
             EEPROM.write(64, Max_Sonar_Hit);   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Max_Sonar_Hit = Max_Sonar_Hit + 1;
               if (Max_Sonar_Hit> 10) Max_Sonar_Hit = 10;
               lcd.clear();
               lcd.setCursor(0,0); 
               lcd.print(F("Sonar Sensitivity:"));
               lcd.setCursor(0,1);
               lcd.print(F("Hits = "));
               lcd.print(Max_Sonar_Hit);
               Serial.print(F("Sonar sensitivity = :"));
               Serial.println(Max_Sonar_Hit);
               }
             if (!Minus_Key_X) {
               Max_Sonar_Hit = Max_Sonar_Hit - 1;
               if (Max_Sonar_Hit < 1) Max_Sonar_Hit = 1;
               lcd.clear();
               lcd.setCursor(0,0); 
               lcd.print(F("Sonar Sensitivity:"));
               lcd.setCursor(0,1);
               lcd.print(F("Hits = "));
               lcd.print(Max_Sonar_Hit);
               Serial.print(F("Sonar sensitivity = :"));
               Serial.println(Max_Sonar_Hit);
               }
             
             }
     }


     if (Menu_Mode_Selection == 4) {
       // Battery Min Setting
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print("Min Battery Volt");
       lcd.setCursor(0,1);
       lcd.print("V = ");
       lcd.print(Battery_Min);
       Serial.print(F("Battery Minimum Volt = :"));
       Serial.println(Battery_Min);
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Sensors = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("V = ");
             lcd.print(Battery_Min);
             lcd.setCursor(0,1);
             lcd.print("SAVED");
             delay(2000);
             lcd.clear();          
             EEPROM.write(25, 1);
             EEPROM.write(26, (Battery_Min * 10));   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Battery_Min = Battery_Min + 0.1;
               if (Battery_Min > 12.6) Battery_Min = 12.6;
               lcd.setCursor(0,1);
               lcd.print("      ");    // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print("V = ");
               lcd.print(Battery_Min);
               Serial.print(F("Minimum Battery Voltage = :"));
               Serial.println(Battery_Min);
               }
             if (!Minus_Key_X) {
               Battery_Min = Battery_Min - 0.1;
               if (Battery_Min < 10.5) Battery_Min = 10.5;
               lcd.setCursor(0,1);
               lcd.print("      ");   // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print("V = ");
               lcd.print(Battery_Min);
               Serial.print(F("Minimum Battery Voltage = :"));
               Serial.println(Battery_Min);
               }
             
             }
     }


     if (Menu_Mode_Selection == 5) {
       // Battery to Charge Sensitivity
       Menu_Mode_Selection = 0;
       delay(500);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Batt Sensitivity:"));
       lcd.setCursor(0,1);
       lcd.print(F("Hits = "));
       lcd.print(Low_Battery_Instances_Chg);
       Serial.print(F("Batt Sensitivity = :"));
       Serial.println(Low_Battery_Instances_Chg);
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Sensors = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Hits : ");
             lcd.print(Low_Battery_Instances_Chg);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(85, 1);
             EEPROM.write(86, Low_Battery_Instances_Chg);   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Low_Battery_Instances_Chg = Low_Battery_Instances_Chg + 1;
               if (Low_Battery_Instances_Chg> 50) Low_Battery_Instances_Chg = 50;
               lcd.clear();
               lcd.setCursor(0,0); 
               lcd.print(F("Batt Sensitivity:"));
               lcd.setCursor(0,1);
               lcd.print(F("Hits = "));
               lcd.print(Low_Battery_Instances_Chg);
               Serial.print(F("Battery sensitivity = :"));
               Serial.println(Low_Battery_Instances_Chg);
               }
             if (!Minus_Key_X) {
               Low_Battery_Instances_Chg = Low_Battery_Instances_Chg - 1;
               if (Low_Battery_Instances_Chg < 1) Low_Battery_Instances_Chg = 1;
               lcd.clear();
               lcd.setCursor(0,0); 
               lcd.print(F("Batt Sensitivity:"));
               lcd.setCursor(0,1);
               lcd.print(F("Hits = "));
               lcd.print(Low_Battery_Instances_Chg);
               Serial.print(F("Battery sensitivity = :"));
               Serial.println(Low_Battery_Instances_Chg);
               }
             
             }
     }



   if (Menu_Mode_Selection == 6) {
       // Wire Sensor ON/OFF
       lcd.clear();
       lcd.print(F("Wire Sensor"));
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Wire Sensor"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Perimeter_Wire_Enabled == 1) lcd.print(F("ON "));
       if (Perimeter_Wire_Enabled == 0) lcd.print(F("OFF"));
       
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Wire Sensor ON/OFF Settings Saved"));
               Menu_Complete_Sensors = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Wire Sensor"));
               lcd.setCursor(0,1);
               lcd.print(F("Saved: "));
               if (Perimeter_Wire_Enabled == 1) lcd.print(F("ON "));
               if (Perimeter_Wire_Enabled == 0) lcd.print(F("OFF"));
               Serial.print(F("Sensor:"));
               Serial.println(Perimeter_Wire_Enabled);
               delay(2000);
               lcd.clear();          
               EEPROM.write(67 , 1);
               EEPROM.write(68 , Perimeter_Wire_Enabled);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Perimeter_Wire_Enabled = 1;
               lcd.print(F("ON "));
               Serial.print(F("Sensor:"));
               Serial.println(Perimeter_Wire_Enabled);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Perimeter_Wire_Enabled = 0;
               lcd.print(F("OFF"));
               Serial.print(F("Sensor:"));
               Serial.println(Perimeter_Wire_Enabled);
               delay(100);
               }
     }
     }

   if (Menu_Mode_Selection == 7) {
       // Rain Sensor ON/OFF
       lcd.clear();
       lcd.print(F("Rain Sensor"));
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Rain Sensor"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Rain_Sensor_Installed == 1) lcd.print(F("ON "));
       if (Rain_Sensor_Installed == 0) lcd.print(F("OFF"));
       
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Rain Sensor ON/OFF Settings Saved"));
               Menu_Complete_Sensors = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Rain Sensor"));
               lcd.setCursor(0,1);
               lcd.print(F("Saved: "));
               if (Rain_Sensor_Installed == 1) lcd.print(F("ON "));
               if (Rain_Sensor_Installed == 0) lcd.print(F("OFF"));
               Serial.print(F("Sensor:"));
               Serial.println(Rain_Sensor_Installed);
               delay(2000);
               lcd.clear();          
               EEPROM.write(77 , 1);
               EEPROM.write(78 , Rain_Sensor_Installed);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Rain_Sensor_Installed = 1;
               lcd.print(F("ON "));
               Serial.print(F("Sensor:"));
               Serial.println(Rain_Sensor_Installed);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Rain_Sensor_Installed = 0;
               lcd.print(F("OFF"));
               Serial.print(F("Sensor:"));
               Serial.println(Rain_Sensor_Installed);
               delay(100);
               }
     }
     }

     if (Menu_Mode_Selection == 8) {
       // Rain Sensitivity
       Menu_Mode_Selection = 0;
       delay(500);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Rain Sensitivity:"));
       lcd.setCursor(0,1);
       lcd.print(F("Hits = "));
       lcd.print(Rain_Total_Hits_Go_Home);
       Serial.print(F("Rain Sensitivity = :"));
       Serial.println(Rain_Total_Hits_Go_Home);
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Sensors = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Hits : ");
             lcd.print(Rain_Total_Hits_Go_Home);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(79, 1);
             EEPROM.write(80, Rain_Total_Hits_Go_Home);   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               Rain_Total_Hits_Go_Home = Rain_Total_Hits_Go_Home + 1;
               if (Rain_Total_Hits_Go_Home> 50) Rain_Total_Hits_Go_Home = 50;
               lcd.clear();
               lcd.setCursor(0,0); 
               lcd.print(F("Rain Sensitivity:"));
               lcd.setCursor(0,1);
               lcd.print(F("Hits = "));
               lcd.print(Rain_Total_Hits_Go_Home);
               Serial.print(F("Rain sensitivity = :"));
               Serial.println(Rain_Total_Hits_Go_Home);
               }
             if (!Minus_Key_X) {
               Rain_Total_Hits_Go_Home = Rain_Total_Hits_Go_Home - 1;
               if (Rain_Total_Hits_Go_Home < 1) Rain_Total_Hits_Go_Home = 1;
               lcd.clear();
               lcd.setCursor(0,0); 
               lcd.print(F("Rain Sensitivity:"));
               lcd.setCursor(0,1);
               lcd.print(F("Hits = "));
               lcd.print(Rain_Total_Hits_Go_Home);
               Serial.print(F("Rain sensitivity = :"));
               Serial.println(Rain_Total_Hits_Go_Home);
               }
             
             }
     }


   if (Menu_Mode_Selection == 9) {
       // WIFI ON/OFF
       lcd.clear();
       lcd.print(F("WIFI Function"));
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("WIFI Function"));
       lcd.setCursor(0,1);
       lcd.print(F("Status: "));
       if (WIFI_Enabled == 1) lcd.print(F("Enabled "));
       if (WIFI_Enabled == 0) lcd.print(F("Disabled"));
       
       Menu_Complete_Sensors = false;
       while (Menu_Complete_Sensors == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Rain Sensor ON/OFF Settings Saved"));
               Menu_Complete_Sensors = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("WIFI Function"));
               lcd.setCursor(0,1);
               lcd.print(F("Saved: "));
               if (WIFI_Enabled == 1) lcd.print(F("Enabled "));
               if (WIFI_Enabled == 0) lcd.print(F("Disabled"));
               Serial.print(F("Status:"));
               Serial.println(WIFI_Enabled);
               delay(2000);
               lcd.clear();          
               EEPROM.write(81 , 1);
               EEPROM.write(82 , WIFI_Enabled);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status: "));
               WIFI_Enabled = 1;
               lcd.print(F("Enabled "));
               Serial.print(F("Status:"));
               Serial.println(WIFI_Enabled);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status: "));
               WIFI_Enabled = 0;
               lcd.print(F("Disabled"));
               Serial.print(F("Status:"));
               Serial.println(WIFI_Enabled);
               delay(100);
               }
     }
     }

//   if (Menu_Mode_Selection == 10) {
//       // Bumper Bar ON/OFF
//       lcd.clear();
//       lcd.print(F("Bumper Bar"));
//       delay(1000);
//       lcd.clear();
//       Menu_Mode_Selection = 0;
//       lcd.clear();
//       lcd.setCursor(0,0);
//       lcd.print(F("Bumper Bar"));
//       lcd.setCursor(0,1);
//       lcd.print(F("Status : "));
//       if (Bumper_Activate_Frnt == 1) lcd.print(F("ON "));
//       if (Bumper_Activate_Frnt == 0) lcd.print(F("OFF"));
//       
//       Menu_Complete_Sensors = false;
//       while (Menu_Complete_Sensors == false) {
//          Read_Membrane_Keys();
//          delay(100);
//          //Enter Code Here to Cycle until stop key is pressed.
//             if(!Start_Key_X){
//               Serial.println(F("Wire Sensor ON/OFF Settings Saved"));
//               Menu_Complete_Sensors = true;
//               lcd.clear();
//               lcd.setCursor(0,0);
//               lcd.print(F("Bumper Bar"));
//               lcd.setCursor(0,1);
//               lcd.print(F("Saved: "));
//               if (Bumper_Activate_Frnt == 1) {
//                  lcd.print(F("ON "));
//                  Setup_Microswitches();
//                  }
//               if (Bumper_Activate_Frnt == 0) lcd.print(F("OFF"));
//               Serial.print(F("Sensor:"));
//               Serial.println(Bumper_Activate_Frnt);
//               delay(2000);
//               lcd.clear();          
//               EEPROM.write(90 , 1);
//               EEPROM.write(91 , Bumper_Activate_Frnt);
//               Menu_Mode_Selection = 0;
//               
//               }
//             if (!Plus_Key_X) {
//               lcd.setCursor(0,1);
//               lcd.print(F("Status : "));
//               Bumper_Activate_Frnt = 1;
//               lcd.print(F("ON "));
//               Serial.print(F("Sensor:"));
//               Serial.println(Bumper_Activate_Frnt);
//               delay(100);
//               }
//             if (!Minus_Key_X) {
//               lcd.setCursor(0,1);
//               lcd.print(F("Status : "));
//               Bumper_Activate_Frnt = 0;
//               lcd.print(F("OFF"));
//               Serial.print(F("Sensor:"));
//               Serial.println(Bumper_Activate_Frnt);
//               delay(100);
//               }
//     }
//     }


  if (Menu_Mode_Selection == 11) {
       // Angle Safety Sensor ON/OFF
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Angle ON/OFF"));
       lcd.setCursor(0,1);
       lcd.print(F("Mode"));
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Angle ON/OFF"));
       lcd.setCursor(0,1);
       lcd.print("Status : ");
       if (Angle_Sensor_Enabled == 1) lcd.print(F("ON "));
       if (Angle_Sensor_Enabled == 0) lcd.print(F("OFF"));
       
       Menu_Complete = false;
       while (Menu_Complete == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Tip Settings Saved"));
               Menu_Complete = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print("Angle Sensor");
               lcd.setCursor(0,1);
               if (Angle_Sensor_Enabled == 0) lcd.print("OFF");
               if (Angle_Sensor_Enabled == 1) lcd.print("ON"); 
               Serial.print(F("Angle Sensor ON/OFF: "));
               Serial.println(Angle_Sensor_Enabled);
               delay(2000);
               lcd.clear();          
               EEPROM.write(29 , 1);
               EEPROM.write(30 , Angle_Sensor_Enabled);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Angle_Sensor_Enabled = 1;
               lcd.print("ON ");
               Serial.print(F("Angle Enabled:"));
               Serial.println(Angle_Sensor_Enabled);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Angle_Sensor_Enabled = 0;
               lcd.print("OFF");
               Serial.print(F("Angle Disabled:"));
               Serial.println(Angle_Sensor_Enabled);
               delay(100);
               }
     }
     }

      
   if (Menu_Mode_Selection == 12) {
       // Tip Safety Sensor ON/OFF
       lcd.clear();
       delay(500);
       Menu_Mode_Selection = 0;
       lcd.setCursor(0,0);
       lcd.print(F("Tip Over Sensor"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Tip_Over_Sensor_Enabled == 1) lcd.print(F("ON "));
       if (Tip_Over_Sensor_Enabled == 0) lcd.print(F("OFF"));
       
       Menu_Complete = false;
       while (Menu_Complete == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Tip Settings Saved"));
               Menu_Complete = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print("Tip Over Sensor");
               lcd.setCursor(0,1);
               if (Tip_Over_Sensor_Enabled == 0) lcd.print("OFF");
               if (Tip_Over_Sensor_Enabled == 1) lcd.print("ON"); 
               Serial.print(F("Tip Over Sensor:"));
               Serial.println(Tip_Over_Sensor_Enabled);
               delay(2000);
               lcd.clear();          
               EEPROM.write(92 , 1);
               EEPROM.write(93 , Tip_Over_Sensor_Enabled);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Tip_Over_Sensor_Enabled = 1;
               lcd.print("ON ");
               Serial.print(F("Tip Over Enabled:"));
               Serial.println(Tip_Over_Sensor_Enabled);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Tip_Over_Sensor_Enabled = 0;
               lcd.print("OFF");
               Serial.print(F("Tip Over Disabled:"));
               Serial.println(Tip_Over_Sensor_Enabled);
               delay(100);
               }
     }
     }     


   if (Menu_Mode_Selection == 13) {
       // Wheel Amp Sensor ON/OFF
       lcd.clear();
       delay(500);
       Menu_Mode_Selection = 0;
       lcd.setCursor(0,0);
       lcd.print(F("Wheel Amp Sensor"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Wheel_Amp_Sensor_ON == 1) lcd.print(F("ON "));
       if (Wheel_Amp_Sensor_ON == 0) lcd.print(F("OFF"));

       Menu_Complete = false;
       while (Menu_Complete == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Wheel Amp Settings Saved"));
               Menu_Complete = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print("Wheel Amp Sensor");
               lcd.setCursor(0,1);
               if (Wheel_Amp_Sensor_ON == 0) lcd.print("OFF");
               if (Wheel_Amp_Sensor_ON == 1) lcd.print("ON");
               Serial.print(F("Wheel Amp Sensor:"));
               Serial.println(Wheel_Amp_Sensor_ON);
               delay(2000);
               lcd.clear();
               EEPROM.write(115 , 1);
               EEPROM.write(116 , Wheel_Amp_Sensor_ON);
               Menu_Mode_Selection = 0;

               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Wheel_Amp_Sensor_ON = 1;
               lcd.print("ON ");
               Serial.print(F("Wheel Amp Enabled:"));
               Serial.println(Wheel_Amp_Sensor_ON);
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print("Status : ");
               Wheel_Amp_Sensor_ON = 0;
               lcd.print("OFF");
               Serial.print(F("Wheel Amp Disabled:"));
               Serial.println(Wheel_Amp_Sensor_ON);
               delay(100);
               }
     }
     }


   if (Menu_Mode_Selection == 14) {
     // Wheel Amp level
     Menu_Mode_Selection = 0;
     delay(500);
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print(F("Wheel Amp level:"));
     lcd.setCursor(0,1);
     lcd.print(F("Level = "));
     lcd.print(Max_Wheel_Amps);
     Serial.print(F("Wheel Amp level= :"));
     Serial.println(Max_Wheel_Amps * 10);
     Menu_Complete_Sensors = false;
     while (Menu_Complete_Sensors == false) {
           Read_Membrane_Keys();
           delay(100);
           //Enter Code Here to Cycle until stop key is pressed.
           if(!Start_Key_X){
           Serial.println(F("Settings Saved"));
           Menu_Complete_Sensors = true;
           lcd.clear();
           lcd.setCursor(0,0);
           lcd.print("Level : ");
           lcd.print(Max_Wheel_Amps);
           lcd.setCursor(0,1);
           lcd.print(F("SAVED"));
           delay(2000);
           lcd.clear();
           EEPROM.write(117, 1);
           EEPROM.write(118, Max_Wheel_Amps * 10);
           Menu_Mode_Selection = 0;
           }
           if (!Plus_Key_X) {
           Max_Wheel_Amps = Max_Wheel_Amps  + 0.1;
             if (Max_Wheel_Amps > 5) Max_Wheel_Amps = 5;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Wheel Amp level:"));
             lcd.setCursor(0,1);
             lcd.print(F("Level = "));
             lcd.print(Max_Wheel_Amps);
             Serial.print(F("Wheel Amp level = :"));
             Serial.println(Max_Wheel_Amps * 10);
             }
           if (!Minus_Key_X) {
           Max_Wheel_Amps = Max_Wheel_Amps - 0.1;
             if (Max_Wheel_Amps < 0.1) Max_Wheel_Amps = 0.1;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Wheel Amp level:"));
             lcd.setCursor(0,1);
             lcd.print(F("Level = "));
             lcd.print(Max_Wheel_Amps);
             Serial.print(F("Wheel Amp level = :"));
             Serial.println(Max_Wheel_Amps * 10);
             }

           }
   }
if (Menu_Complete_Sensors == true) Print_Membrane_Switch_Input_Sensors();
}
#endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Set_Time
// BUTTONS ALARMS MENU

#if defined(LCD_KEYPAD)


// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Alarms(byte LCD_Menu_Alarms) {
  if (LCD_Menu_Alarms == 1) lcd.print(F("Set Alarm 1"));
  if (LCD_Menu_Alarms == 2) lcd.print(F("Set Alarm 2"));
  if (LCD_Menu_Alarms == 3) lcd.print(F("Set Alarm 3"));
  if (LCD_Menu_Alarms == 4) lcd.print(F("Set Clock")); 
  Max_Options_Alarms = 4;
  }


void Print_Membrane_Switch_Input_Alarms() {
     Read_Membrane_Keys();
     Menu_Complete_Alarms = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

      Serial.println();
      Serial.println(F("Alarms Menu Activated"));
      Menu_Complete_Alarms = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
      
      while (Menu_Complete_Alarms == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Alarms(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Alarms(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete_Alarms = true;
          Serial.println(F("Start key is pressed"));
          Activate_Menu_Option_Alarms();
          lcd.clear();         
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Alarms();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Alarms();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete_Alarms = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit Alarms");
          delay(1000);
          lcd.clear();          
          }
      }
    }

    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_Alarms() {
     if (Menu_View > Max_Options_Alarms) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Alarms(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Alarms( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_Alarms() {
   
     
     if (Menu_Mode_Selection == 1) {
       // Alarm 1 Settings
     
       int Set = 1;
       Menu_Mode_Selection = 1;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F(">Alarm 1: "));
       if (Alarm_1_ON == 1) lcd.print("ON");
       if (Alarm_1_ON == 0) lcd.print("OFF");
       lcd.setCursor(0,1);
       lcd.print(" Time: ");
       lcd.print(Alarm_1_Hour);
       lcd.print(F(":"));
       if (Alarm_1_Minute < 10) lcd.print ("0");
       lcd.print(Alarm_1_Minute);


       Menu_Complete_Alarms = false;
       while (Menu_Complete_Alarms == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 4) {
               lcd.clear(); 
               lcd.setCursor(0,0);
               lcd.print(F("Alarm 1"));
               lcd.setCursor(0,1);
               lcd.print(F("Settings Saved"));
               delay(2000);            
               EEPROM.write(1, 1);
               EEPROM.write(2, Alarm_1_Hour);
               EEPROM.write(3, Alarm_1_Minute);
               EEPROM.write(4, Alarm_1_ON);
               EEPROM.write(87, Alarm_1_Action);
               Menu_Mode_Selection = 0;
               Menu_Complete_Alarms = true;        
               }
             
             if (Set == 3) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(" Time: ");
                lcd.print(Alarm_1_Hour);
                lcd.print(F(":"));
                if (Alarm_1_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_1_Minute); 
                lcd.setCursor(0,1);           
                lcd.print(">Do: ");
                 if (Alarm_1_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_1_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_1_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_1_Action == 4) lcd.print("Quick Start");
                 if (Alarm_1_Action == 5) lcd.print("Custom     ");
                Set = Set + 1;          
               }
               
             if (Set == 2) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Alarm 1: "));
                if (Alarm_1_ON == 1) lcd.print("ON");
                if (Alarm_1_ON == 0) lcd.print("OFF");
                lcd.setCursor(0,1);
                lcd.print(" Time: ");
                lcd.print(Alarm_1_Hour);
                lcd.print(F(":"));
                if (Alarm_1_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_1_Minute);
                lcd.print("<");
                Set = Set + 1;         
               }
            
             if (Set == 1) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Alarm 1: "));
                if (Alarm_1_ON == 1) lcd.print("ON");
                if (Alarm_1_ON == 0) lcd.print("OFF");
                lcd.setCursor(0,1);
                lcd.print(" Time:>");
                lcd.print(Alarm_1_Hour);
                lcd.print(F(":"));
                if (Alarm_1_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_1_Minute);
                Set = Set + 1;         
               }
             }

             if (!Plus_Key_X) {
               if (Set == 1) {
                 lcd.clear();
                 Alarm_1_ON = 1;
                 if (Alarm_1_ON > 1) Alarm_1_ON = 1;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Alarm 1: "));
                 if (Alarm_1_ON == 1) lcd.print("ON");
                 if (Alarm_1_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 }
               if (Set == 2) {
                 lcd.clear();
                 Alarm_1_Hour = Alarm_1_Hour + 1;
                 if (Alarm_1_Hour > 24) Alarm_1_Hour = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 1: "));
                 if (Alarm_1_ON == 1) lcd.print("ON");
                 if (Alarm_1_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time:>");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 }
               if (Set == 3) {
                 Alarm_1_Minute = Alarm_1_Minute + 1;
                 if (Alarm_1_Minute > 60) Alarm_1_Minute = 0;
                 if (Alarm_1_Minute < 0) Alarm_1_Minute = 59;
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 1: "));
                 if (Alarm_1_ON == 1) lcd.print("ON");
                 if (Alarm_1_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");                 
                 lcd.print(Alarm_1_Minute);
                 lcd.print("<");
                 }
                if (Set == 4) {
                 lcd.clear();
                 Alarm_1_Action = Alarm_1_Action + 1;
                 if (Alarm_1_Action > 5) Alarm_1_Action = 1;
                 lcd.setCursor(0,0);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 lcd.setCursor(0,1);
                 lcd.print(">Do: ");
                 if (Alarm_1_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_1_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_1_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_1_Action == 4) lcd.print("Quick Start");
                 if (Alarm_1_Action == 5) lcd.print("Custom     ");
                 }
             }
             
             if (!Minus_Key_X) {
               if (Set == 1) {
                 lcd.clear();
                 Alarm_1_ON = 0;
                 if (Alarm_1_ON > 1) Alarm_1_ON = 1;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Alarm 1: "));
                 if (Alarm_1_ON == 1) lcd.print("ON");
                 if (Alarm_1_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 }
               if (Set == 2) {
                 lcd.clear();
                 Alarm_1_Hour = Alarm_1_Hour - 1;
                 if (Alarm_1_Hour < 0)  Alarm_1_Hour = 23;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 1: "));
                 if (Alarm_1_ON == 1) lcd.print("ON");
                 if (Alarm_1_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time:>");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 }
               if (Set == 3) {
                 lcd.clear();
                 Alarm_1_Minute = Alarm_1_Minute - 1;
                 if (Alarm_1_Minute < 0) Alarm_1_Minute = 59;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 1: "));
                 if (Alarm_1_ON == 1) lcd.print("ON");
                 if (Alarm_1_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 lcd.print("<");
                 }
                if (Set == 4) {
                 Alarm_1_Action = Alarm_1_Action - 1;
                 if (Alarm_1_Action < 1) Alarm_1_Action = 5;
                 lcd.setCursor(0,0);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_1_Hour);
                 lcd.print(F(":"));
                 if (Alarm_1_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_1_Minute);
                 lcd.setCursor(0,1);
                 lcd.print(">Do: ");
                 if (Alarm_1_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_1_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_1_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_1_Action == 4) lcd.print("Quick Start");
                 if (Alarm_1_Action == 5) lcd.print("Custom     ");
                 }
             }       
           }
     }
     



     if (Menu_Mode_Selection == 2) {

       // Alarm 2 Settings
     
       int Set = 1;
       Menu_Mode_Selection = 1;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F(">Alarm 2: "));
       if (Alarm_2_ON == 1) lcd.print("ON");
       if (Alarm_2_ON == 0) lcd.print("OFF");
       lcd.setCursor(0,1);
       lcd.print(" Time: ");
       lcd.print(Alarm_2_Hour);
       lcd.print(F(":"));
       if (Alarm_2_Minute < 10) lcd.print ("0");
       lcd.print(Alarm_2_Minute);


       Menu_Complete_Alarms = false;
       while (Menu_Complete_Alarms == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 4) {
               lcd.clear(); 
               lcd.setCursor(0,0);
               lcd.print(F("Alarm 2"));
               lcd.setCursor(0,1);
               lcd.print(F("Settings Saved"));
               delay(2000);           
               EEPROM.write(5, 1);
               EEPROM.write(6, Alarm_2_Hour);
               EEPROM.write(7, Alarm_2_Minute);
               EEPROM.write(8, Alarm_2_ON);
               EEPROM.write(88, Alarm_2_Action);
               Menu_Mode_Selection = 0;
               Menu_Complete_Alarms = true;
               }
             
             if (Set == 3) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(" Time: ");
                lcd.print(Alarm_2_Hour);
                lcd.print(F(":"));
                if (Alarm_2_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_2_Minute); 
                lcd.setCursor(0,1);           
                lcd.print(">Do: ");
                 if (Alarm_2_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_2_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_2_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_2_Action == 4) lcd.print("Quick Start");
                 if (Alarm_2_Action == 5) lcd.print("Custom     ");
                Set = Set + 1;          
               }
               
             if (Set == 2) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Alarm 2: "));
                if (Alarm_2_ON == 1) lcd.print("ON");
                if (Alarm_2_ON == 0) lcd.print("OFF");
                lcd.setCursor(0,1);
                lcd.print(" Time: ");
                lcd.print(Alarm_2_Hour);
                lcd.print(F(":"));
                if (Alarm_2_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_2_Minute);
                lcd.print("<");
                Set = Set + 1;         
               }
            
             if (Set == 1) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Alarm 2: "));
                if (Alarm_2_ON == 1) lcd.print("ON");
                if (Alarm_2_ON == 0) lcd.print("OFF");
                lcd.setCursor(0,1);
                lcd.print(" Time:>");
                lcd.print(Alarm_2_Hour);
                lcd.print(F(":"));
                if (Alarm_2_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_2_Minute);
                Set = Set + 1;         
               }
             }

             if (!Plus_Key_X) {
               if (Set == 1) {
                 lcd.clear();
                 Alarm_2_ON = 1;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Alarm 2: "));
                 if (Alarm_2_ON == 1) lcd.print("ON");
                 if (Alarm_2_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 }
               if (Set == 2) {
                 lcd.clear();
                 Alarm_2_Hour = Alarm_2_Hour + 1;
                 if (Alarm_2_Hour > 24) Alarm_2_Hour = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 2: "));
                 if (Alarm_2_ON == 1) lcd.print("ON");
                 if (Alarm_2_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time:>");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 }
               if (Set == 3) {
                 lcd.clear();
                 Alarm_2_Minute = Alarm_2_Minute + 1;
                 if (Alarm_2_Minute > 60) Alarm_2_Minute = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 2: "));
                 if (Alarm_2_ON == 1) lcd.print("ON");
                 if (Alarm_2_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");                 
                 lcd.print(Alarm_2_Minute);
                 lcd.print("<");
                 }
                if (Set == 4) {
                 lcd.clear();
                 Alarm_2_Action = Alarm_2_Action + 1;
                 if (Alarm_2_Action > 5) Alarm_2_Action = 1;
                 lcd.setCursor(0,0);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 lcd.setCursor(0,1);
                 lcd.print(">Do: ");
                 if (Alarm_2_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_2_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_2_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_2_Action == 4) lcd.print("Quick Start");
                 if (Alarm_2_Action == 5) lcd.print("Custom     ");
                 }
             }
             
             if (!Minus_Key_X) {
               if (Set == 1) {
                 lcd.clear();
                 Alarm_2_ON = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Alarm 2: "));
                 if (Alarm_2_ON == 1) lcd.print("ON");
                 if (Alarm_2_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 }
               if (Set == 2) {
                 lcd.clear();
                 Alarm_2_Hour = Alarm_2_Hour - 1;
                 if (Alarm_2_Hour < 0)  Alarm_2_Hour = 23;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 2: "));
                 if (Alarm_2_ON == 1) lcd.print("ON");
                 if (Alarm_2_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time:>");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 }
               if (Set == 3) {
                 lcd.clear();
                 Alarm_2_Minute = Alarm_2_Minute - 1;
                 if (Alarm_2_Minute < 0) Alarm_2_Minute = 59;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 2: "));
                 if (Alarm_2_ON == 1) lcd.print("ON");
                 if (Alarm_2_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 lcd.print("<");
                 }
                if (Set == 4) {
                 Alarm_2_Action = Alarm_2_Action - 1;
                 if (Alarm_2_Action < 1) Alarm_2_Action = 5;
                 lcd.setCursor(0,0);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_2_Hour);
                 lcd.print(F(":"));
                 if (Alarm_2_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_2_Minute);
                 lcd.setCursor(0,1);
                 lcd.print(">Do: ");
                 if (Alarm_2_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_2_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_2_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_2_Action == 4) lcd.print("Quick Start");
                 if (Alarm_2_Action == 5) lcd.print("Custom     ");
                 }
             }       
           }
     }



     if (Menu_Mode_Selection == 3) {

       // Alarm 3 Settings
     
       int Set = 1;
       Menu_Mode_Selection = 1;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F(">Alarm 3: "));
       if (Alarm_3_ON == 1) lcd.print("ON");
       if (Alarm_3_ON == 0) lcd.print("OFF");
       lcd.setCursor(0,1);
       lcd.print(" Time: ");
       lcd.print(Alarm_3_Hour);
       lcd.print(F(":"));
       if (Alarm_3_Minute < 10) lcd.print ("0");
       lcd.print(Alarm_3_Minute);


       Menu_Complete_Alarms = false;
       while (Menu_Complete_Alarms == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 4) {
               lcd.clear(); 
               lcd.setCursor(0,0);
               lcd.print(F("Alarm 3"));
               lcd.setCursor(0,1);
               lcd.print(F("Settings Saved"));
               delay(2000);           
               EEPROM.write(9, 1);
               EEPROM.write(10, Alarm_3_Hour);
               EEPROM.write(11, Alarm_3_Minute);
               EEPROM.write(12, Alarm_3_ON);
               EEPROM.write(89, Alarm_3_Action);
               Menu_Mode_Selection = 0;
               Menu_Complete_Alarms = true;
               }
             
             if (Set == 3) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(" Time: ");
                lcd.print(Alarm_3_Hour);
                lcd.print(F(":"));
                if (Alarm_3_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_3_Minute); 
                lcd.setCursor(0,1);           
                lcd.print(">Do: ");
                 if (Alarm_3_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_3_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_3_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_3_Action == 4) lcd.print("Quick Start");
                 if (Alarm_3_Action == 5) lcd.print("Custom     ");
                Set = Set + 1;          
               }
               
             if (Set == 2) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Alarm 3: "));
                if (Alarm_3_ON == 1) lcd.print("ON");
                if (Alarm_3_ON == 0) lcd.print("OFF");
                lcd.setCursor(0,1);
                lcd.print(" Time: ");
                lcd.print(Alarm_3_Hour);
                lcd.print(F(":"));
                if (Alarm_3_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_3_Minute);
                lcd.print("<");
                Set = Set + 1;         
               }
            
             if (Set == 1) { 
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Alarm 3: "));
                if (Alarm_3_ON == 1) lcd.print("ON");
                if (Alarm_3_ON == 0) lcd.print("OFF");
                lcd.setCursor(0,1);
                lcd.print(" Time:>");
                lcd.print(Alarm_3_Hour);
                lcd.print(F(":"));
                if (Alarm_3_Minute < 10) lcd.print ("0");
                lcd.print(Alarm_3_Minute);
                Set = Set + 1;         
               }
             }

             if (!Plus_Key_X) {
               if (Set == 1) {
                 lcd.clear();
                 Alarm_3_ON = 1;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Alarm 3: "));
                 if (Alarm_3_ON == 1) lcd.print("ON");
                 if (Alarm_3_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 }
               if (Set == 2) {
                 lcd.clear();
                 Alarm_3_Hour = Alarm_3_Hour + 1;
                 if (Alarm_3_Hour > 24) Alarm_3_Hour = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 3: "));
                 if (Alarm_3_ON == 1) lcd.print("ON");
                 if (Alarm_3_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time:>");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 }
               if (Set == 3) {
                 lcd.clear();
                 Alarm_3_Minute = Alarm_3_Minute + 1;
                 if (Alarm_3_Minute > 60) Alarm_3_Minute = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 3: "));
                 if (Alarm_3_ON == 1) lcd.print("ON");
                 if (Alarm_3_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");                 
                 lcd.print(Alarm_3_Minute);
                 lcd.print("<");
                 }
                if (Set == 4) {
                 lcd.clear();
                 Alarm_3_Action = Alarm_3_Action + 1;
                 if (Alarm_3_Action > 5) Alarm_3_Action = 1;
                 lcd.setCursor(0,0);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 lcd.setCursor(0,1);
                 lcd.print(">Do: ");
                 if (Alarm_3_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_3_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_3_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_3_Action == 4) lcd.print("Quick Start");
                 if (Alarm_3_Action == 5) lcd.print("Custom     ");
                 }
             }
             
             if (!Minus_Key_X) {
               if (Set == 1) {
                 lcd.clear();
                 Alarm_3_ON = 0;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Alarm 3: "));
                 if (Alarm_3_ON == 1) lcd.print("ON");
                 if (Alarm_3_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 }
               if (Set == 2) {
                 lcd.clear();
                 Alarm_3_Hour = Alarm_3_Hour - 1;
                 if (Alarm_3_Hour < 0)  Alarm_3_Hour = 23;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 3: "));
                 if (Alarm_3_ON == 1) lcd.print("ON");
                 if (Alarm_3_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time:>");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 }
               if (Set == 3) {
                 lcd.clear();
                 Alarm_3_Minute = Alarm_3_Minute - 1;
                 if (Alarm_3_Minute < 0) Alarm_3_Minute = 59;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Alarm 3: "));
                 if (Alarm_3_ON == 1) lcd.print("ON");
                 if (Alarm_3_ON == 0) lcd.print("OFF");
                 lcd.setCursor(0,1);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 lcd.print("<");
                 }
                if (Set == 4) {
                 Alarm_3_Action = Alarm_3_Action - 1;
                 if (Alarm_3_Action < 1) Alarm_3_Action = 5;
                 lcd.setCursor(0,0);
                 lcd.print(" Time: ");
                 lcd.print(Alarm_3_Hour);
                 lcd.print(F(":"));
                 if (Alarm_3_Minute < 10) lcd.print ("0");
                 lcd.print(Alarm_3_Minute);
                 lcd.setCursor(0,1);
                 lcd.print(">Do: ");
                 if (Alarm_3_Action == 1) lcd.print("Exit Zone 1");
                 if (Alarm_3_Action == 2) lcd.print("Exit Zone 2");
                 if (Alarm_3_Action == 3) lcd.print("Mow the Line");
                 if (Alarm_3_Action == 4) lcd.print("Quick Start");
                 if (Alarm_3_Action == 5) lcd.print("Custom     ");
                 }
             }       
           }
     }

 


// Set Mower CLock
      
if (Menu_Mode_Selection == 4) {
       // Set Mower Clock
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);            // Spaces to the right & down
       int set_hour;
       int set_min;
       
       if (PCB == 0) {
          Time t = rtc.time();
          set_hour = t.hr;
          set_min = t.min;
          }      
       
       if (PCB == 1) {
          Display_DS3231_Time();  
          set_hour = Time_Hour;
          set_min = Time_Minute;
          }            
     
       lcd.print(F("Time : "));
       lcd.print(set_hour);
       lcd.print(F(":"));
       if (set_min < 10) lcd.print ("0");
       lcd.print(set_min);
       Serial.print(F("Time : "));
       Serial.print(set_hour);
       Serial.print(F(":"));
       if (set_min < 10) Serial.print ("0");
       Serial.println(set_min);

       Menu_Complete_Alarms = false;
       delay(500);
       while (Menu_Complete_Alarms == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
               if(!Start_Key_X){
               Serial.println(F("Settings Saved"));
               Menu_Complete_Alarms = true;
               lcd.clear();
               lcd.print(F("Time : "));
               lcd.print(set_hour);
               lcd.print(":");
               if (set_min < 10) lcd.print ("0");
               lcd.print(set_min);
               lcd.setCursor(0,1);

       if (PCB == 0) {           
          rtc.writeProtect(false);
          rtc.halt(false);
          Serial.print(F("Clock : "));
          Serial.print(set_hour);
          Serial.print(":");
          if (set_min < 10) Serial.print("0");
          Serial.println(set_min);
          Time t(2019, 07, 19, set_hour, set_min, 00, Time::kFriday);            // Year XXXX, Month XX, Day XX, Hour XX, Minute XX, Second, kXYZday
          rtc.time(t);    
          delay(200);
          rtc.writeProtect(true);
          rtc.halt(true);
          rtc.time(t); 
          }

      if (PCB == 1) {
          Set_DS3231_Time(00,set_min, set_hour, 2,14,7,20);    //second, minute, hour, dayof week, day of month, month, year
          Serial.println(F("TIME SAVED"));
          delay(2000);
          }
               
         lcd.print(F("TIME SAVED"));
         delay(2000);
         lcd.clear();          
         Menu_Mode_Selection = 0;
               }
             if (!Plus_Key_X) {
               set_min = set_min + 1;
               if (set_min > 59) {
                set_min = 0;
                set_hour = set_hour + 1;
                }
               if (set_hour > 23) set_hour = 0;
               lcd.clear();
               lcd.print(F("Time : "));
               lcd.print(set_hour);
               lcd.print(":");
               if (set_min < 10) lcd.print ("0");
               lcd.print(set_min);
               }
             if (!Minus_Key_X) {
               set_min = set_min - 1;
               if (set_min < 0) {
                set_min = 59;
                set_hour = set_hour - 1;
                }
               if (set_hour < 0) set_hour = 23;
               lcd.clear();
               lcd.print(F("Time : "));
               lcd.print(set_hour);
               lcd.print(":");
               if (set_min < 10) lcd.print ("0");
               lcd.print(set_min);
                }
             if(!Stop_Key_X){
               Serial.println(F("Time Set Cancelled"));
               Menu_Complete_Alarms = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Time Set"));
               lcd.setCursor(0,1);
               lcd.print(F("Cancelled"));    
               delay(2000);
               lcd.clear();          
               Menu_Mode_Selection = 0;
               }
             }
       Serial.print(F("Time : "));
       delay(1000);
       }

if (Menu_Complete_Alarms == true) Print_Membrane_Switch_Input_Alarms();   

}

#endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Settings
// BUTTONS SETTINGS MENU

#if defined(LCD_KEYPAD)


// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Settings(byte LCD_Menu_Settings) {
  if (LCD_Menu_Settings == 1)  lcd.print(F("Time"));
  if (LCD_Menu_Settings == 2)  lcd.print(F("Sensors"));
  if (LCD_Menu_Settings == 3)  lcd.print(F("Motion"));  
  if (LCD_Menu_Settings == 4)  lcd.print(F("Tracking"));
  if (LCD_Menu_Settings == 5)  lcd.print(F("Navigation"));
  if (LCD_Menu_Settings == 6)  lcd.print(F("- -"));
  if (LCD_Menu_Settings == 7)  lcd.print(F("BETA"));   // Leave Blank
  if (LCD_Menu_Settings == 8)  lcd.print(F("- -"));   // Leave Blank
  if (LCD_Menu_Settings == 9)  lcd.print(F("- -"));   // Leave Blank
  if (LCD_Menu_Settings == 10) lcd.print(F("CLEAR EEPROM"));   // Leave Blank

  Max_Options_Settings = 8; 
  }







void Print_Membrane_Switch_Input_Settings() {

  //Menu Options if the Mower is Settings.
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

    
      Serial.println();
      Serial.println(F("Settings Menu Activated"));
      Menu_Complete_Settings = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
 

      while (Menu_Complete_Settings == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          //Print_LCD_Menu_Settings(1);
          lcd.print("-- Settings --");
          lcd.setCursor(1,1);
          Print_LCD_Menu_Settings(1);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete_Settings = true;
          Serial.println(F("Start key is pressed"));
          lcd.clear();
          Activate_Menu_Option_Settings();
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Settings();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Settings();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete_Settings = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit Settings");
          delay(400);
          lcd.clear();          
          Menu_Mode_Selection = -1;
          }
      }
    }


   
 
 void Run_Menu_Order_Settings() {
     if (Menu_View > Max_Options_Settings) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;     
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Settings(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Settings( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }


void Activate_Menu_Option_Settings() {

      

      if (Menu_Mode_Selection == 1) {
        lcd.clear();
        lcd.print("Alarms Setup");
        Serial.println(F("Alarms Setup Selected"));
        Menu_Mode_Selection = 0;
        delay(400);
        lcd.clear();
        Print_Membrane_Switch_Input_Alarms();
        Menu_Mode_Selection = -1;      // skips the re-writing of the menu
        }


      if (Menu_Mode_Selection == 2) {
        lcd.clear();
        lcd.print("Sensor Setup");
        Serial.println(F("Sensor Setup Selected"));
        Menu_Mode_Selection = 0;
        delay(400);
        lcd.clear();
        Print_Membrane_Switch_Input_Sensors();
        }


      if (Menu_Mode_Selection == 3) {
        lcd.clear();
        lcd.print("Motion Setup");
        Serial.println(F("Motion Setup Selected"));
        Menu_Mode_Selection = 0;
        delay(400);
        lcd.clear();
        Print_Membrane_Switch_Input_Motion();
        }

  
      if (Menu_Mode_Selection == 4) {
        lcd.clear();
        lcd.print("Tracking Setup");
        Serial.println(F("Tracking Setup Selected"));
        Menu_Mode_Selection = 0;
        delay(400);
        lcd.clear();
        Print_Membrane_Switch_Input_Tracking();
        //Menu_Mode_Selection = -1;      // skips the re-writing of the menu
        }

   
      if (Menu_Mode_Selection == 5) {
        lcd.clear();
        lcd.print("NAVI Setup");
        Serial.println(F("NAVISetup Selected"));
        Menu_Mode_Selection = 0;
        delay(400);
        lcd.clear();
        Print_Membrane_Switch_Input_NAVI();
        //Menu_Mode_Selection = -1;      // skips the re-writing of the menu
        }
          
 
      if (Menu_Mode_Selection == 7) {
        lcd.clear();
        lcd.print("BETA Commands");
        Serial.println(F("BETA Section"));
        Menu_Mode_Selection = 0;
        delay(400);
        lcd.clear();
        Print_Membrane_Switch_Input_BETA();
        //Menu_Mode_Selection = -1;      // skips the re-writing of the menu
        }     



// Menu Option to CLEAR EEPROM
//-----------------------------
     
      if (Menu_Mode_Selection == 10) {
       // EEPORM Clear MENU
       lcd.clear();
       lcd.print("EEPROM Clear ?");
       lcd.setCursor(0,1);
       lcd.print("Yes/No");
       Serial.println(F("Clear EEPROM Yes/No   - Press Up for Yes and Down for No.  Start to Confirm"));
       Menu_Mode_Selection = 0;
       Menu_Complete = false;
       int Answer = 0;
       while (Menu_Complete == false) {
          //Enter Code Here to Cycle until stop key is pressed.
          Read_Membrane_Keys();
          delay(100);
           if(!Start_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             if (Answer == 1) {
              Clear_EERPOM_Data();
              lcd.clear();
              lcd.print("EEPROM Cleared");
              lcd.setCursor(0,1);
              lcd.print("Power OFF Mower");
              }
             if (Answer == 0) {
              lcd.clear();
              lcd.print("Cancelled");
              lcd.setCursor(0,1);
              }
             delay(2000);
             lcd.clear();          
             Menu_Mode_Selection = 0;
             }
            if (!Plus_Key_X) {
               Answer = 1;
               lcd.setCursor(0,1);
               lcd.print("Yes    ");
               Serial.println(F("Clear EEPROM = YES....  Press Start to Confirm"));
               delay(100);
               }
             if (!Minus_Key_X) {
               Answer = 0;
               lcd.setCursor(0,1);
               lcd.print("No     ");               
               Serial.println(F("Clear EEPROM = No....Press Start to Confirm"));
               delay(100);
               }
       }
     }

  Print_Membrane_Switch_Input_Settings();
  }

  #endif
  

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Testing
// BUTTONS TEST MENU
#if defined(LCD_KEYPAD)

// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Tests(byte LCD_Menu_Tests) {
  if (LCD_Menu_Tests == 1)  lcd.print(F("Wire Test"));
  if (LCD_Menu_Tests == 2)  lcd.print(F("Relay Test"));
  if (LCD_Menu_Tests == 3)  lcd.print(F("Wheel Test"));
  if (LCD_Menu_Tests == 4)  lcd.print(F("Blade Test"));
  if (LCD_Menu_Tests == 5)  lcd.print(F("Sonar Test"));
  if (LCD_Menu_Tests == 6)  lcd.print(F("Turn Test"));
  if (LCD_Menu_Tests == 7)  lcd.print(F("Volt Amp Test"));
  if (LCD_Menu_Tests == 8)  lcd.print(F("Compass Test"));
  if (LCD_Menu_Tests == 9)  lcd.print(F("Go Home Test"));
//  if (LCD_Menu_Tests == 10) lcd.print(F("Bumper Test"));
  if (LCD_Menu_Tests == 11) lcd.print(F("Wheel Amps"));  //RVES added
  if (LCD_Menu_Tests == 12) lcd.print(F("Gyro Test"));  //RVES added
  if (LCD_Menu_Tests == 13) lcd.print(F("Spare 13"));  
  if (LCD_Menu_Tests == 14) lcd.print(F("BETA"));  
  if (LCD_Menu_Tests == 15) lcd.print(F("Spare 15")); 
  if (LCD_Menu_Tests == 16) lcd.print(F(""));   // Leave Blank
  Max_Options_Test = 16;
  }


void Print_Membrane_Switch_Input_Tests() {

  //Menu Options if the Mower is Tests.
     Read_Membrane_Keys();
     Menu_Complete = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

    
      Serial.println();
      Serial.println(F("Test Menu Activated"));
      Menu_Complete = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
 

      while (Menu_Complete == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Tests(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Tests(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete = true;
          Serial.println(F("Start key is pressed"));
          lcd.clear();
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Testing();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Testing();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(F("Exit Test"));
          lcd.setCursor(0,1);
          lcd.print(F("Menu"));
          delay(1000);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    Activate_Menu_Option_Testing();
    }


 void Run_Menu_Order_Testing() {
     if (Menu_View > Max_Options_Test) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Tests(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Tests( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(F(">"));
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }


void Activate_Menu_Option_Testing() {
     
     if (Menu_Mode_Selection == 1) {
       // Perimeter Wire Test
       lcd.clear();
       lcd.print(F("Wire Test"));
       lcd.setCursor(0,1);
       lcd.print(F("Activated"));
       Serial.println(F("Perimeter Wire Test Started"));
       delay(5000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       Menu_Complete = false;
       while (Menu_Complete == false) {
          Test_Mower_Check_Wire();
          Read_Membrane_Keys();
             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Test Stopped"));
             delay(2000);
             lcd.clear();          
             Menu_Mode_Selection = 0;
             }
     }
     }
 
     
     if (Menu_Mode_Selection == 2) {
       // Relay Test
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.print(F("Relay Test"));
       lcd.setCursor(0,1);
       lcd.print(F("Activated"));
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       Menu_Complete = false;
       while (Menu_Complete == false) {
          Test_Relay();
          Read_Membrane_Keys();
             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Test Stopped"));
             delay(2000);
             lcd.clear();          
             Menu_Mode_Selection = 0;
             }
     }
    }

     if (Menu_Mode_Selection == 3) {
       // Mower Motor Test
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.print(F("Wheel Motor Test"));
       lcd.setCursor(0,1);
       lcd.print("Activated");
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       I = 1;                       // sets the itteration for the motor test
       Test_Wheel_Motors();         // starts the mowe motor test 1x
       }
        
   
     if (Menu_Mode_Selection == 4) {
       // Blade Motor Test
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.print("Blade Motor Test");
       lcd.setCursor(0,1);
       lcd.print("Activated");
       delay(1000);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       Test_Mower_Blade_Motor();
        }
          
      if (Menu_Mode_Selection == 5) {
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.print("Sonar Test");
       lcd.setCursor(0,1);
       lcd.print("Activated");
       delay(1000);
       lcd.clear();
       Menu_Complete = false;
       while (Menu_Complete == false) {
          Test_Sonar_Array();
          Read_Membrane_Keys();
             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Test Stopped");
             delay(2000);
             lcd.clear();          
             Menu_Mode_Selection = 0;
             }
     }
      }

      if (Menu_Mode_Selection == 6) {
        lcd.clear();
        lcd.print("Pattern Mow");
        Serial.println(F("Slot 6 Selected"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        Test_Compass_Turn_Function();
        }
      if (Menu_Mode_Selection == 7) {
        lcd.clear();
        lcd.print("Volt Amps Test");
        Serial.println(F("Volts and Amps Test"));
        Menu_Mode_Selection = 0;
        delay(1000);
        lcd.clear();
        Menu_Complete = false;
        while (Menu_Complete == false) {
          Read_Membrane_Keys();
          Read_Serial1_Nano();
          delay(100);
          Serial.print(F("  Charging:"));
          Serial.print(Charging);
          lcd.setCursor(0,0);
          lcd.print(F("Volt:"));
          lcd.print(Volts);
          lcd.setCursor(0,1);
          lcd.print(F("Amps:"));
          lcd.print(Amps);
          lcd.setCursor(13,1);
          lcd.print(F("C:"));
          lcd.print(Charging);
;          Serial.println(F(""));
        
             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print(F("Test Stopped"));
             delay(2000);
             lcd.clear();          
             Menu_Mode_Selection = 0;
             }
         }   
        }
      if (Menu_Mode_Selection == 8) {
        lcd.clear();
        lcd.print("Compass Test");
        Serial.println(F("Compass Test Selected"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Heading:");
        lcd.setCursor(0,1);
        lcd.print("Degrees:");
        Menu_Complete = false;
        while (Menu_Complete == false) {
          // insert Test Code Here
          Read_Membrane_Keys();
          Get_Compass_Reading();
          delay(100);
          Serial.print(F("Heading:"));
          Serial.print(Heading);
          Serial.print(F("|"));
          Serial.println(F(""));
          lcd.setCursor(9,0);
          lcd.print(Heading);
          lcd.setCursor(9,1);
          lcd.print(Compass_Heading_Degrees);
          delay(500);
        
             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Test Stopped");
             delay(2000);
             lcd.clear();          
             Menu_Mode_Selection = 0;
             }
         }   
      }
        


      // Tests the compass direction finding of the mower when finding the wire.  
      if (Menu_Mode_Selection == 9) {
        lcd.clear();
        lcd.print("Go Home Test");
        Serial.println(F("Test Compass Turn to Home Test and Follow Wire"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        Manouver_Go_To_Charging_Station();
        }


      // Tests the bumper bar is functioning correctly.  
//      if (Menu_Mode_Selection == 10) {
//        lcd.clear();
//        lcd.print("Bumper Bar");
//        lcd.setCursor(0,1);
//        lcd.print("Test");
//        Serial.println(F("Test Compass Turn to Home Test and Follow Wire"));
//        Menu_Mode_Selection = 0;
//        delay(3000);
//        lcd.clear();
//        Start_Bumper_Bar_Test();
//        }


      // Tests the Wheel Amps is functioning correctly.
      if (Menu_Mode_Selection == 11) {
        lcd.clear();
        lcd.print("Wheel Amps");
        lcd.setCursor(0,1);
        lcd.print("Test");
        Serial.println(F("Running Wheel Amp Test"));
        Menu_Mode_Selection = 0;
        delay(3000);
        Test_Wheel_Amps();
        }

      if (Menu_Mode_Selection == 12) {
        lcd.clear();
        lcd.print("Gyro Test");
        Serial.println(F("Gyro Test Selected"));
        Menu_Mode_Selection = 0;
        delay(3000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("X:");
        lcd.setCursor(9,0);
        lcd.print("Y:");
        lcd.setCursor(0,1);
        lcd.print("Z:");
        Menu_Complete = false;
        while (Menu_Complete == false) {
          // insert Test Code Here
          Read_Membrane_Keys();
          Get_GYRO_Reading();
          Print_GYRO_Reading();
          delay(100);
          Serial.println("");
          lcd.setCursor(2,0);
          lcd.print(GYRO_Angle_X);
          lcd.setCursor(11,0);
          lcd.print(GYRO_Angle_Y);
          lcd.setCursor(2,1);
          lcd.print(GYRO_Angle_Z);
          delay(500);

             if(!Stop_Key_X){
             Serial.println(F("Stop key is pressed"));
             Menu_Complete = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("Test Stopped");
             delay(2000);
             lcd.clear();
             Menu_Mode_Selection = 0;
             }
         }
      }

      if (Menu_Mode_Selection == 14) {
        lcd.clear();
        lcd.print("BETA Commands");
        Serial.println(F("BETA Section"));
        Menu_Mode_Selection = 0;
        delay(1000);
        lcd.clear();
        Print_Membrane_Switch_Input_BETA();
        //Menu_Mode_Selection = -1;      // skips the re-writing of the menu
        } 
  }
#endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Tracking
// Tracking SETTINGS MENU

#if defined(LCD_KEYPAD)


// Test to displyed on the LCD screen when using the membrane key menus
void Print_LCD_Menu_Tracking(byte LCD_Menu_Tracking) {
  if (LCD_Menu_Tracking == 1) lcd.print(F("Set PID"));
  if (LCD_Menu_Tracking == 2) lcd.print(F("Docking ON/OFF"));
  if (LCD_Menu_Tracking == 3) lcd.print(F("Tracking Dir"));
  if (LCD_Menu_Tracking == 4) lcd.print(F("Exit Point 1&2"));
  if (LCD_Menu_Tracking == 5) lcd.print(F("WireFind FwdBck")); 
  if (LCD_Menu_Tracking == 6) lcd.print(F("Wheel Counter"));    
  if (LCD_Menu_Tracking == 7) lcd.print(F(""));
  Max_Options_Tracking = 7;
  }


void Print_Membrane_Switch_Input_Tracking() {
     Read_Membrane_Keys();
     Menu_Complete_Tracking = 1;
     Menu_Mode_Selection = 0;
     Menu_View = 0;

      Serial.println();
      Serial.println(F("Tracking Menu Activated"));
      Menu_Complete_Tracking = false;                                // Menu complete will return to the normal loop
      lcd.clear();
      delay(5);
      
      while (Menu_Complete_Tracking == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
        if (Menu_View == 0) {
          lcd.setCursor(1,0);
          Print_LCD_Menu_Tracking(1);
          lcd.setCursor(1,1);
          Print_LCD_Menu_Tracking(2);
          }
        // Gets the values again from the keys
        Read_Membrane_Keys();
        delay(100);
             
        if(!Start_Key_X){
          Menu_Complete_Tracking = true;
          Serial.println(F("Start key is pressed"));
          Activate_Menu_Option_Tracking();
          lcd.clear();
          
          }
        if(!Plus_Key_X) {
          Serial.println(F("+ key is pressed"));
          Menu_View = Menu_View - 1;
          Run_Menu_Order_Tracking();
          }
        if(!Minus_Key_X) {
          Menu_View = Menu_View + 1;
          Run_Menu_Order_Tracking();
        }
        if(!Stop_Key_X){
          Serial.println(F("Stop key is pressed"));
          Menu_Complete_Tracking = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Exit Tracking");
          delay(400);
          lcd.clear();          
          Menu_Mode_Selection = 0;
          }
      }
    //Activate_Menu_Option_Tracking();
    }

    


// Code to scroll the menu and print the menu options in the LCD
 void Run_Menu_Order_Tracking() {
     if (Menu_View > Max_Options_Tracking) Menu_View = Menu_View -1;
     if (Menu_View < 0) Menu_View = Menu_View + 1;      
     Serial.print(F("- key is pressed "));
     lcd.clear();
     lcd.setCursor(1,0);
     Print_LCD_Menu_Tracking(Menu_View);
     lcd.setCursor(1,1);
     Print_LCD_Menu_Tracking( Menu_View + 1);
     lcd.setCursor(0,0);
     lcd.print(">");
     Menu_Mode_Selection = Menu_View;
     Serial.print(F("Menu View : "));
     Serial.print(Menu_View);
     Serial.print(F("| Menu Selection"));
     Serial.println(Menu_Mode_Selection);
     delay(100);
     }

     
// Defines the actions when that option is selected with the keypad.
void Activate_Menu_Option_Tracking() {


     if (Menu_Mode_Selection == 1) {
       // Tracking PID Settings
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(500);
       lcd.setCursor(0,0);
       lcd.print(F("Tracking PID:"));
       lcd.setCursor(0,1);
       lcd.print(F("P = "));
       lcd.print(P);
       Serial.print(F("Tracking PID P = :"));
       Serial.println(P);
       Menu_Complete_Tracking = false;
       while (Menu_Complete_Tracking == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             Serial.println(F("Settings Saved"));
             Menu_Complete_Tracking = true;
             lcd.clear();
             lcd.setCursor(0,0);
             lcd.print("P : ");
             lcd.print(P);
             lcd.setCursor(0,1);
             lcd.print(F("SAVED"));
             delay(2000);
             lcd.clear();          
             EEPROM.write(21, 1);
             EEPROM.write(22, (P*100));   
             Menu_Mode_Selection = 0;
             }
             if (!Plus_Key_X) {
               P = P + 0.01;
               if (P > 10) P = 10;
               lcd.setCursor(0,1);
               lcd.print(F("      "));    // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print(F("P : "));
               lcd.print(P);
               Serial.print(F("Tracking PID P = :"));
               Serial.println(P);
               }
             if (!Minus_Key_X) {
               P = P - 0.01;
               if (P < 0) P = 0;
               lcd.setCursor(0,1);
               lcd.print(F("      "));   // Fully clear the number to stop reminants of a previous number from being left behind
               lcd.setCursor(0,1);
               lcd.print(F("P : "));
               lcd.print(P);
               Serial.print(F("Tracking PID P = :"));
               Serial.println(P);
               }
             
             }
     }


if (Menu_Mode_Selection == 2) {
       // Charging Station activated
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Charging Dock"));
       lcd.setCursor(0,1);
       lcd.print(F("ON/OFF"));
       delay(400);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Dock ON/OFF"));
       lcd.setCursor(0,1);
       lcd.print(F("Status : "));
       if (Use_Charging_Station == 1) lcd.print(F("ON "));
       if (Use_Charging_Station == 0) lcd.print(F("OFF"));
       
       Menu_Complete_Tracking = false;
       while (Menu_Complete_Tracking == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Charging Station Settings Saved"));
               Menu_Complete_Tracking = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Dock Saved"));
               Serial.print(F("Dock:"));
               Serial.println(Use_Charging_Station);
               delay(2000);
               lcd.clear();          
               EEPROM.write(47 , 1);
               EEPROM.write(48 , Use_Charging_Station);
               Menu_Mode_Selection = 0;
               
               }
             if (!Plus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Use_Charging_Station = 1;
               lcd.print(F("ON "));
               Serial.print(F("DOCK:"));
               Serial.println("Use_Charging_Station");
               delay(100);
               }
             if (!Minus_Key_X) {
               lcd.setCursor(0,1);
               lcd.print(F("Status : "));
               Use_Charging_Station = 0;
               lcd.print(F("OFF"));
               Serial.print(F("DOCK:"));
               Serial.println(Use_Charging_Station);
               delay(100);
               }
     }
     }


if (Menu_Mode_Selection == 3) {
       // Charging Station Diretion
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Tracking "));
       lcd.setCursor(0,1);
       lcd.print(F("Directions"));
       delay(400);
       lcd.clear();
       Menu_Mode_Selection = 0;
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Exit Dir: "));
       if (CW_Tracking_To_Start == 1)  lcd.print(F("CW"));
       if (CCW_Tracking_To_Start == 1) lcd.print(F("CCW"));
       lcd.setCursor(0,1);
       lcd.print(F("Dock Dir: "));
       if (CW_Tracking_To_Charge == 1)  lcd.print(F("CW"));
       if (CCW_Tracking_To_Charge == 1) lcd.print(F("CCW"));
       
       Menu_Complete_Tracking = false;
       while (Menu_Complete_Tracking == false) {
          Read_Membrane_Keys();
          delay(100);
          //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
               Serial.println(F("Charging Station Settings Saved"));
               Menu_Complete_Tracking = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Tracking"));
               lcd.setCursor(0,1);
               lcd.print(F("Direction Saved"));
               delay(2000);
               lcd.clear();          
               EEPROM.write(49 , 1);
               EEPROM.write(50 , CW_Tracking_To_Charge);                
               EEPROM.write(51 , 1);
               EEPROM.write(52 , CCW_Tracking_To_Charge);       
               EEPROM.write(53 , 1);
               EEPROM.write(54 , CW_Tracking_To_Start);                
               EEPROM.write(55 , 1);
               EEPROM.write(56 , CCW_Tracking_To_Start);   
               Menu_Mode_Selection = 0;
               }               
               
             if (!Plus_Key_X) {
               lcd.clear();
               CW_Tracking_To_Start   = 1;
               CCW_Tracking_To_Charge = 1;   
               CCW_Tracking_To_Start  = 0;
               CW_Tracking_To_Charge  = 0;  
               lcd.setCursor(0,0); 
               lcd.print(F("Exit Dir: "));               
               if (CW_Tracking_To_Start == 1)  lcd.print(F("CW"));
               if (CCW_Tracking_To_Start == 1) lcd.print(F("CCW"));
               lcd.setCursor(0,1);
               lcd.print(F("Dock Dir: "));
               if (CW_Tracking_To_Charge == 1)  lcd.print(F("CW"));
               if (CCW_Tracking_To_Charge == 1) lcd.print(F("CCW"));
               }


               
             if (!Minus_Key_X) {
               lcd.clear();
               CW_Tracking_To_Start   = 0;
               CCW_Tracking_To_Charge = 0;   
               CCW_Tracking_To_Start  = 1;
               CW_Tracking_To_Charge  = 1;  
               lcd.setCursor(0,0); 
               lcd.print(F("Exit Dir: "));               
               if (CW_Tracking_To_Start == 1)  lcd.print(F("CW"));
               if (CCW_Tracking_To_Start == 1) lcd.print(F("CCW"));
               lcd.setCursor(0,1);
               lcd.print(F("Dock Dir: "));
               if (CW_Tracking_To_Charge == 1)  lcd.print(F("CW"));
               if (CCW_Tracking_To_Charge == 1) lcd.print(F("CCW"));
               }
       }
     }

   if (Menu_Mode_Selection == 4) {
       // Sets the Exit point of Zone 1 and 2
       // Number of loop cycles when the mower is tracking the wire to Zone 1 and Zone 2

     
       int Set = 1;
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Tracking Zone"));
       lcd.setCursor(0,1);
       lcd.print(F("1&2 Cycles"));
       delay(400);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("> Zone1:"));
       lcd.print(Track_Wire_Zone_1_Cycles);
       lcd.setCursor(0,1);
       lcd.print(F("  Zone2:"));
       lcd.print(Track_Wire_Zone_2_Cycles);
       Menu_Complete_Tracking = false;
       while (Menu_Complete_Tracking == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 2) { 
               Serial.println(F("Settings Saved"));
               Menu_Complete_Tracking = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Z1&Z2 Track"));
               lcd.setCursor(0,1);
               lcd.print(F("Cycles SAVED"));
               delay(2000);
               lcd.clear();          
               EEPROM.write(43, 1);
               EEPROM.write(44, (Track_Wire_Zone_1_Cycles/100));
               EEPROM.write(45, 1);
               EEPROM.write(46, (Track_Wire_Zone_2_Cycles/100));
               Menu_Mode_Selection = 0;
               }
              
             if (Set == 1) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F("  Zone1:"));
                lcd.print(Track_Wire_Zone_1_Cycles);
                lcd.setCursor(0,1);
                lcd.print(F("> Zone2:"));
                lcd.print(Track_Wire_Zone_2_Cycles);   
                Set = Set + 1;          
                }


             }
             if (!Plus_Key_X) {
               if (Set == 1) {
                 Track_Wire_Zone_1_Cycles = Track_Wire_Zone_1_Cycles + 100;
                 lcd.setCursor(0,0);
                 lcd.setCursor(0,0);
                 lcd.print(F("> Zone1:"));
                 lcd.print(Track_Wire_Zone_1_Cycles);
                 lcd.setCursor(0,1);
                 lcd.print(F("  Zone2:"));
                 lcd.print(Track_Wire_Zone_2_Cycles);
                 }
               if (Set == 2) {
                 Track_Wire_Zone_2_Cycles = Track_Wire_Zone_2_Cycles + 100;
                 lcd.setCursor(0,0);
                 lcd.setCursor(0,0);
                 lcd.print(F("  Zone1:"));
                 lcd.print(Track_Wire_Zone_1_Cycles);
                 lcd.setCursor(0,1);
                 lcd.print(F("> Zone2:"));
                 lcd.print(Track_Wire_Zone_2_Cycles);          
                 }
               }
             if (!Minus_Key_X) {
               if (Set == 1) {
                 Track_Wire_Zone_1_Cycles = Track_Wire_Zone_1_Cycles - 100;
                   if (Track_Wire_Zone_1_Cycles < 1000) lcd.clear();     
                   if (Track_Wire_Zone_1_Cycles <= 0) {
                     lcd.clear();
                     Track_Wire_Zone_1_Cycles = 100;      // cant be less than zero.
                     }
                 lcd.setCursor(0,0);
                 lcd.print(F("> Zone1:"));
                 lcd.print(Track_Wire_Zone_1_Cycles);
                 lcd.setCursor(0,1);
                 lcd.print(F("  Zone2:"));
                 lcd.print(Track_Wire_Zone_2_Cycles);
                 }
               if (Set == 2) {
                 Track_Wire_Zone_2_Cycles = Track_Wire_Zone_2_Cycles - 100;
                 if (Track_Wire_Zone_2_Cycles < 1000) lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F("  Zone1:"));
                 lcd.print(Track_Wire_Zone_1_Cycles);
                 lcd.setCursor(0,1);
                 lcd.print(F("> Zone2:"));
                 lcd.print(Track_Wire_Zone_2_Cycles);          
                 }
               }
     
             
     }
     }


     if (Menu_Mode_Selection == 5) {
       // Forward and Reverse Tracking Distance to find wire
     
       int Set = 1;
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Set Wire Find"));
       lcd.setCursor(0,1);
       lcd.print(F("Distance/Cycles "));
       delay(400);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F(">Track Fwd: "));
       lcd.print(Max_Cycle_Wire_Find );
       lcd.setCursor(0,1);
       lcd.print(F(" Track Bck: "));
       lcd.print(Max_Cycle_Wire_Find_Back);
       Menu_Complete_Tracking = false;
       while (Menu_Complete_Tracking == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 2) { 
               Serial.println(F("Settings Saved"));
               Menu_Complete_Tracking = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Fwd/Bck Track"));
               lcd.setCursor(0,1);
               lcd.print(F("Cycles SAVED"));
               delay(2000);
               lcd.clear();          
               EEPROM.write(69, 1);
               EEPROM.write(70, (Max_Cycle_Wire_Find/10));
               EEPROM.write(71, 1);
               EEPROM.write(72, (Max_Cycle_Wire_Find_Back/10));
               Menu_Mode_Selection = 0;
               }
              
             if (Set == 1) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" Track Fwd: "));
                lcd.print(Max_Cycle_Wire_Find);
                lcd.setCursor(0,1);
                lcd.print(F(">Track Bck: "));
                lcd.print(Max_Cycle_Wire_Find_Back);
                Set = Set + 1;  
                delay(200);        
                }


             }
             if (!Plus_Key_X) {
               if (Set == 1) {
                 Max_Cycle_Wire_Find = Max_Cycle_Wire_Find + 10;
                 lcd.setCursor(0,0);
                 lcd.print(F(">Track Fwd: "));
                 lcd.print(Max_Cycle_Wire_Find );
                 lcd.setCursor(0,1);
                 lcd.print(F(" Track Bck: "));
                 lcd.print(Max_Cycle_Wire_Find_Back);
                 }
               if (Set == 2) {
                 Max_Cycle_Wire_Find_Back = Max_Cycle_Wire_Find_Back + 10;
                 lcd.setCursor(0,0);
                 lcd.print(F(" Track Fwd: "));
                 lcd.print(Max_Cycle_Wire_Find);
                 lcd.setCursor(0,1);
                 lcd.print(F(">Track Bck: "));
                 lcd.print(Max_Cycle_Wire_Find_Back);         
                 }
               }
             if (!Minus_Key_X) {
               if (Set == 1) {
                 Max_Cycle_Wire_Find = Max_Cycle_Wire_Find - 10;
                 if (Max_Cycle_Wire_Find <= 10) Max_Cycle_Wire_Find = 10;      // cant be less than zero.
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F(">Track Fwd: "));
                 lcd.print(Max_Cycle_Wire_Find);
                 lcd.setCursor(0,1);
                 lcd.print(F(" Track Bck: "));
                 lcd.print(Max_Cycle_Wire_Find_Back);  
                 }
               if (Set == 2) {
                 Max_Cycle_Wire_Find_Back = Max_Cycle_Wire_Find_Back - 10;
                 if (Max_Cycle_Wire_Find_Back <= 10) Max_Cycle_Wire_Find_Back = 10;      // cant be less than zero.
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F(" Track Fwd: "));
                 lcd.print(Max_Cycle_Wire_Find);
                 lcd.setCursor(0,1);
                 lcd.print(F(">Track Bck: "));
                 lcd.print(Max_Cycle_Wire_Find_Back);       
                 }
               }
     
             
     }
     }



     if (Menu_Mode_Selection == 6) {
       //Wheel Input Counter LH and RH
     
       int Set = 1;
       Menu_Mode_Selection = 0;
       lcd.clear();
       delay(100);
       lcd.setCursor(0,0);
       lcd.print(F("Wheel Input"));
       lcd.setCursor(0,1);
       lcd.print(F("Counter/Cycles "));
       delay(400);
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F(">RH Wheel: "));
       lcd.print(Max_Tracking_Turn_Right );
       lcd.setCursor(0,1);
       lcd.print(F(" LH Wheel: "));
       lcd.print(Max_Tracking_Turn_Left);
       Menu_Complete_Tracking = false;
       while (Menu_Complete_Tracking == false) {
             Read_Membrane_Keys();
             delay(100);
             //Enter Code Here to Cycle until stop key is pressed.
             if(!Start_Key_X){
             
             if (Set == 2) { 
               Serial.println(F("Settings Saved"));
               Menu_Complete_Tracking = true;
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print(F("Wheel Counter/"));
               lcd.setCursor(0,1);
               lcd.print(F("Cycles SAVED"));
               delay(2000);
               lcd.clear();          
               EEPROM.write(73, 1);
               EEPROM.write(74, (Max_Tracking_Turn_Right/10));
               EEPROM.write(75, 1);
               EEPROM.write(76, (Max_Tracking_Turn_Left/10));
               Menu_Mode_Selection = 0;
               }
              
             if (Set == 1) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print(F(" RH Wheel: "));
                lcd.print(Max_Tracking_Turn_Right);
                lcd.setCursor(0,1);
                lcd.print(F(">LH Wheel: "));
                lcd.print(Max_Tracking_Turn_Left);
                Set = Set + 1;  
                delay(200);        
                }


             }
             if (!Plus_Key_X) {
               if (Set == 1) {
                 Max_Tracking_Turn_Right = Max_Tracking_Turn_Right + 10;
                 lcd.setCursor(0,0);
                 lcd.print(F(">RH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Right );
                 lcd.setCursor(0,1);
                 lcd.print(F(" LH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Left);
                 }
               if (Set == 2) {
                 Max_Tracking_Turn_Left = Max_Tracking_Turn_Left + 10;
                 lcd.setCursor(0,0);
                 lcd.print(F(" RH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Right);
                 lcd.setCursor(0,1);
                 lcd.print(F(">LH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Left);         
                 }
               }
             if (!Minus_Key_X) {
               if (Set == 1) {
                 Max_Tracking_Turn_Right = Max_Tracking_Turn_Right - 10;
                 if (Max_Tracking_Turn_Right <= 100) Max_Tracking_Turn_Right = 100;      // cant be less than zero.
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F(">RH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Right);
                 lcd.setCursor(0,1);
                 lcd.print(F(" LH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Left);  
                 }
               if (Set == 2) {
                 Max_Tracking_Turn_Left = Max_Tracking_Turn_Left - 10;
                 if (Max_Tracking_Turn_Left <= 100) Max_Tracking_Turn_Left = 100;      // cant be less than zero.
                 lcd.clear();
                 lcd.setCursor(0,0);
                 lcd.print(F(" RH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Right);
                 lcd.setCursor(0,1);
                 lcd.print(F(">LH Wheel: "));
                 lcd.print(Max_Tracking_Turn_Left);       
                 }
               }
     
             
     }
     }

     
if (Menu_Complete_Tracking == true) Print_Membrane_Switch_Input_Tracking();   
}

#endif


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// LCD_Menu_Display
// Infornation to be printed to the Mower LCD screen
// to activate this part of the code make sure that the //#define LCD_KEYPAD
// is activated by removing the //.
// Otherwise these parts of the code will not be compiled to reduce the 
// overall size of the program.


void Setup_Run_LCD_Intro () {
  #if defined(LCD_KEYPAD)
  Serial.println(F("Setup LCD"));
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(F("ReP_AL Robot"));
  lcd.setCursor(0,1);
  lcd.print(Version);  
  if (WIFI_Enabled == 1) {
    lcd.setCursor(7,1);
    lcd.print(F("WIFI ON"));
    }
  delay(1000);
  lcd.clear();
  Serial.println(F("LCD Setup OK"));
  #endif
  }


void Print_Mower_Error() {
  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Mower Error"));
  if (Wire_Off > 5) {
    lcd.setCursor(0,1);
    lcd.print("Wire Off");
    }
  if (Wire_Refind_Tries > 4) {
    lcd.setCursor(0,1);
    lcd.print(F("Cant Refind Wire"));
    }
  #endif  
  }
  


void Print_LCD_Volt_Info() {
   #if defined(LCD_KEYPAD)
   lcd.setCursor(10, 0);
   lcd.print("V:");
   lcd.setCursor(12, 0);    
   lcd.print(Volts); 
   #endif 
   }
        

void Print_Charging_LCD()  {
  #if defined(LCD_KEYPAD)
  lcd.setCursor(0,0);
  if (Charge_Detected_MEGA == 1)  lcd.print("Charging");
  if ((Rain_Detected == 0) && (Charge_Detected_MEGA != 1) ) lcd.print("        ");
  #endif
  }

void Print_Raining_LCD()  {
  #if defined(LCD_KEYPAD)
  lcd.setCursor(4,0);
  if (Rain_Detected == 1) lcd.print("Rain");
  // See raining and charging clause if this is not displying correctly.
    if ((Rain_Detected == 0) && (Charging == 0) ) lcd.print("    ");
  #endif
  }

void Print_Recharge_LCD() {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1);
    lcd.print("Recharge Batt");
    #endif
    }

void Print_LCD_Wire()  {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0, 1);
    lcd.print("WIRE Detect ");     
    #endif
    }

void Print_LCD_Mowing() {
     
     #if defined(LCD_KEYPAD)
     
     if (Alarm_Timed_Mow_ON == 0) {
       lcd.setCursor(0, 1);
       lcd.print("Mowing..    ");
       }
     
     if ((Alarm_Timed_Mow_ON == 1) && ( Loop_Cycle_Mowing < 15)) {
       lcd.setCursor(0,1);
       lcd.print("Timer:");
       lcd.print(Alarm_Timed_Mow_Hour);
       lcd.print(":");
       if (Alarm_Timed_Mow_Minute < 10) lcd.print("0");
       lcd.print(Alarm_Timed_Mow_Minute);
       Mow_Time_Set = 1;
       }         
     
     #endif
     }

void Print_LCD_Spiral() {
     #if defined(LCD_KEYPAD)
     if (Alarm_Timed_Mow_ON == 0) {
       lcd.setCursor(0, 1);
       lcd.print("Spiral.. ");
       }
     if ((Alarm_Timed_Mow_ON == 1) && ( Loop_Cycle_Mowing < 15)) {
       lcd.setCursor(0,1);
       lcd.print("Timer:");
       lcd.print(Alarm_Timed_Mow_Hour);
       lcd.print(":");
       if (Alarm_Timed_Mow_Minute < 10) lcd.print("0");
       lcd.print(Alarm_Timed_Mow_Minute);
       Mow_Time_Set = 1;
       }          
     #endif
     }

void Print_LCD_Parallel() {
     #if defined(LCD_KEYPAD)
     if (Alarm_Timed_Mow_ON == 0) {
       lcd.setCursor(0, 1);
       lcd.print("Parallel");
       }
     if ((Alarm_Timed_Mow_ON == 1) && ( Loop_Cycle_Mowing < 15)) {
       lcd.setCursor(0,1);
       lcd.print("Timer:");
       lcd.print(Alarm_Timed_Mow_Hour);
       lcd.print(":");
       if (Alarm_Timed_Mow_Minute < 10) lcd.print("0");
       lcd.print(Alarm_Timed_Mow_Minute);
       Mow_Time_Set = 1;
       }         
     #endif
     }
     
void Print_LCD_Compass_Mowing() {
     #if defined(LCD_KEYPAD)
     if (Compass_Heading_Locked == 1) {
         if ( Loop_Cycle_Mowing > 15) {
             lcd.setCursor(0, 1);
             if (PWM_Right > PWM_Left)  lcd.print("<Mow         ");      
             if (PWM_Left > PWM_Right)  lcd.print(" Mow>        "); 
             if (PWM_Left == PWM_Right) lcd.print("|Mow|        ");
             }
     }
     if (Compass_Heading_Locked == 0) {
         lcd.setCursor(0, 1);
         lcd.print("           ");      
         }
     #endif
     }

void Print_LCD_Gyro_Mowing() {  // RVES added
    #if defined(LCD_KEYPAD)
    if (GYRO_Heading_Locked == 1) {
        if ( Loop_Cycle_Mowing > 15) {
            lcd.setCursor(0, 1);
            if (PWM_Right > PWM_Left)  lcd.print("<GMow         ");
            if (PWM_Left > PWM_Right)  lcd.print(" GMow>        ");
            if (PWM_Left == PWM_Right) lcd.print("|GMow|        ");
            }
    }
    if (GYRO_Heading_Locked == 0) {
        lcd.setCursor(0, 1);
        lcd.print("GcycleLow  ");
        }
    #endif
    }

void Print_LCD_Parallel_Mowing() {
     #if defined(LCD_KEYPAD)
     if (Compass_Heading_Locked == 1) {
         lcd.setCursor(0, 1);
         if (PWM_Right > PWM_Left)  lcd.print("< P Mow         ");      
         if (PWM_Left > PWM_Right)  lcd.print(" P Mow >        "); 
         if (PWM_Left == PWM_Right) lcd.print("| P Mow |       ");
         }
     if (Compass_Heading_Locked == 0) {
         lcd.setCursor(0, 1);
         lcd.print(" Parllel ");      
         }
     #endif
     }

void Print_LCD_Info_Parked() {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1);
    lcd.print("Parked");
    #endif
    }

void Print_LCD_Info_Manual() {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1);
    lcd.print("Manual");
    #endif
    }

void Print_LCD_Info_Docked() {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1);
    lcd.print("Docked");  
    #endif
    }

void Print_LCD_NO_Wire() {
    #if defined(LCD_KEYPAD)
    if ( (Mower_Docked == 1) || (Mower_Parked == 1) ) {
      lcd.setCursor(7,1);
      lcd.print(":WIRE OFF");  
      Wire_ON_Printed = 0;
      }
    if ( (Mower_Docked == 0) && (Mower_Parked == 0) ) {
      lcd.setCursor(0,1);
      lcd.print(":WIRE OFF        ");      
      Wire_ON_Printed = 0;
      }
    #endif
}

void Print_LCD_Wire_ON() {
    #if defined(LCD_KEYPAD)
    if ( ((Mower_Docked == 1) || (Mower_Parked == 1))  && (Wire_ON_Printed = 0)) {
      lcd.setCursor(7,1);
      lcd.print(":               ");  
      Wire_ON_Printed = 1; 
      }
    if ( (Mower_Docked == 0) && (Mower_Parked == 0) && (Wire_ON_Printed = 0) ) {
      lcd.setCursor(0,1);
      lcd.print(":               ");   
      Wire_ON_Printed = 1;    
      }
    #endif
}

void Print_Sonar_Hit()  {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1);
    lcd.print("Sonar Object ");  
    #endif
    }

void Print_LCD_Compass_Home() {
    #if defined(LCD_KEYPAD)
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Compass Home");
    if (Rain_Detected == 1) {
       lcd.setCursor(0,1);
       lcd.print("RAINING");
       }
    #endif
    }



void Print_Time_On_LCD() {
    #if defined(LCD_KEYPAD)
    if ((Charge_Detected_MEGA == 0) && (Mower_Running == 0) && (Rain_Detected == 0)) {      
      
      if (PCB == 1) {
          Display_DS3231_Time();  
          //Time_Hour = hour;
          //Time_Minute = minute;
          //Time_Second = second;     
          lcd.setCursor(0,0);
          lcd.print(Time_Hour);
          lcd.print(":");
          if (Time_Minute < 10) lcd.print ("0");
          lcd.print(Time_Minute);
          }
       if (PCB == 0) {
          lcd.setCursor(0,0);            // Spaces to the right & down
          Time t = rtc.time();
          lcd.print(t.hr);
          lcd.print(":");
          if (t.min < 10) lcd.print ("0");
          lcd.print(t.min);
          }
    }
    #endif
}


void Print_LCD_Wheel_Blocked() {
    #if defined(LCD_KEYPAD)
    lcd.setCursor(0,1);
    lcd.print("!Wheel_Blocked!");
    #endif
    }


void Print_LCD_Stop_Btn() {
    #if defined(LCD_KEYPAD)
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("STOP pressed");
    #endif
    }
    

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Manouvers
// Manouvers are a set of motion functions or motor actions that are regulary called
// e.g. Turn the mower around
// Ctrl + F = Find in Arduino   Highlight the sub-routine first and press Ctrl + F

// void Manouver_Mow_The_Grass()      normal grass cutting operation 
// void Manouver_Aerate_The_Grass()   aeration process
// void Manouver_Find_Wire_Track()    Finds the wire edge before tracking the wire
// void Manouver_Turn_Around()        How the mower eacts when corssing the wire
// void Manouver_Turn_Around_non_blocking ()
// void Manouver_Turn_Around_Sonar()  Hoe the mower avoids a sonar obstacle
// void Manouver_Turn_Around_Sonar_non_blocking()

// void Manouver_Manual_Mode()
// void Manouver_Start_Mower()
// void Manouver_Mower_Exit_Dock()
// void Manouver_Dock_The_Mower() 
// void Manouver_Park_The_Mower_Low_Batt()
// void Manouver_Park_The_Mower()
// void Manouver_Setup_Mode()
// void Manouver_Hibernate_Mower()

// void Manouver_Go_To_Charging_Station()     Starts the sequence to send the mower to the charge station
// void Manouver_Exit_To_Zone_X()             Brings the mower out of the dock and tracks the wire to the start zone



// Moves the Mower Forwards in the garden and activates compass features if enabled


void Manouver_Mow_Aerate_The_Grass() {
  if (Robot_Type == 1)  Manouver_Mow_The_Grass();
//  if (Robot_Type == 2)  Manouver_Aerate_The_Grass();
 }




void Manouver_Mow_The_Grass() {

    Motor_Action_Spin_Blades();
    SetPins_ToGoForwards();

    Loop_Cycle_Mowing = (Loop_Cycle_Mowing + 1);
    Serial.print(F("Loop:"));
    Serial.print(Loop_Cycle_Mowing);
    Serial.print(F("|"));
    
    #if defined(LCD_KEYPAD)
      lcd.setCursor(13, 1);
      lcd.print(Loop_Cycle_Mowing);
      #endif
  
  if (Loop_Cycle_Mowing < 50  )  {

    // If the GYRO is activated the compass will be ignored and the mower uses the GYRO instead.
    if ((Compass_Activate == 1) && (Compass_Heading_Hold_Enabled == 1) && (GYRO_Enabled == 0) ) {
      if (Loop_Cycle_Mowing % 10 == 0 ) {
      Serial.print(F("C-Lock:OFF"));
      Serial.print(F("|"));

      #if defined(LCD_KEYPAD)
        Print_LCD_Mowing();
        #endif

      Get_Compass_Reading();
      Motor_Action_Go_Mowing_Speed();
      Compass_Heading_Locked = 0;                           // Turn off the compass heading lock for the new cycles
      Compass_Steering_Status = 0;
      }
  }

    if (GYRO_Enabled == 1) {
        Serial.print(F("G-Lock:OFF"));
        Serial.print(F("|"));
        
        #if defined(LCD_KEYPAD)
          Print_LCD_Mowing();
          #endif
        
        Get_GYRO_Reading();
        Motor_Action_Go_Mowing_Speed();
        GYRO_Heading_Locked = 0;                           // Turn off the compass heading lock for the new cycles
        GYRO_Steering_Status = 0;
        }   
  }

  // On the 5th Mowing cycle options are chosen how to Mow based on the settings of pattern mow
  // Compass assistance etc.
  
  if (Loop_Cycle_Mowing == 50)   {

    // Normal Random Mowing
    if (Pattern_Mow == 0) {
        
        // Normal Random Mowing no compass Assistance
        if ((Compass_Heading_Hold_Enabled == 0) && (GYRO_Enabled == 0)) {
          Serial.print(F("C-Lock:OFF"));
          Serial.print(F("|"));
          
          #if defined(LCD_KEYPAD)
            Print_LCD_Mowing();
            #endif
          
          Motor_Action_Go_Mowing_Speed();
          Compass_Heading_Locked = 0;
          }

        // Normal Random Mowing no GYRO Assistance
        if (GYRO_Enabled == 0) {
          Serial.print(F("GYRO:OFF"));
          Serial.print(F("|"));
          Motor_Action_Go_Mowing_Speed();
          GYRO_Heading_Locked = 0;
          }          

        // Normal Random Mowing with Compass Assitnace turned on.
        // If the GYRO is turned on then the compass will be ignored and the angle will come from the GYRO
        if ((Compass_Heading_Hold_Enabled == 1) && (Compass_Activate == 1) && (GYRO_Enabled == 0) )  {
           Get_Compass_Reading();                                                      // Gets the latest compass reading
           Heading_Lock = Compass_Heading_Degrees;                                     // saves this compass reading to the heading lock
           Serial.print("Locked");
           Compass_Heading_Locked = 1;                                                 // Turns on the heading lock feature
           Motor_Action_Go_Mowing_Speed();
           }
        if (GYRO_Enabled == 1) {
           Get_GYRO_Reading();                                                      // Gets the latest compass reading
           GYRO_Heading_Locked = 1;                                                 // Turns on the heading lock feature
           Motor_Action_Go_Mowing_Speed();    
           }
        else {
          Motor_Action_Go_Mowing_Speed();
          }
      }

    if (Pattern_Mow == 1)  {                                  
        Motor_Action_Go_Mowing_Speed();
        Print_LCD_Parallel();

        Serial.print("Parallel:ON");
        Serial.print(F("|"));

        if ((Compass_Heading_Hold_Enabled == 1) && (Compass_Activate == 1))  {         // use the heading hold funtion for Parallel Mowing
           Get_Compass_Reading();                                                      // Gets the latest compass reading
           Heading_Lock = Compass_Heading_Degrees;                                     // saves this compass reading to the heading lock
           Compass_Heading_Locked = 1;                                                 // Turns on the heading lock feature
           Motor_Action_Go_Mowing_Speed();
           }
        else {
          Motor_Action_Go_Mowing_Speed();                                                // if the compass is not activated
          Serial.println("Compass not activated in the settings");
          }
        
        }

    // Sets up the variables so that a spiral mow pattern is activated.
    if (Pattern_Mow == 2)  {                                  
        Compass_Heading_Locked = 0;                   // Compass Lock is switched off                 
        
        #if defined(LCD_KEYPAD)
          Print_LCD_Spiral();
          #endif
        
        Serial.print("Spiral:ON");
        Serial.print(F("|"));
        Motor_Action_Go_Mowing_Speed();
        }

  }  // end of statements for == 5



  // Based on the settings above the Mower will continue to mow with the following actions
  if (Loop_Cycle_Mowing > 50) {
      
      // Normal Random Mowing
      if (Pattern_Mow == 0) {
         if ((Compass_Heading_Locked == 0) && (GYRO_Enabled == 0)) {
           Serial.print(F("C-Lock:OFF"));
           Serial.print(F("|"));

           #if defined(LCD_KEYPAD)
             lcd.setCursor(0, 1);  
             lcd.print("Mowing          ");
             #endif
           
           Motor_Action_Go_Mowing_Speed();
           Compass_Steering_Status = 0;                                             // TFT Information
           }
        if ((GYRO_Heading_Locked == 0) && (GYRO_Enabled == 0) && (Compass_Activate == 0)) {
           Serial.print(F("G-Lock:OFF"));
           Serial.print(F("|"));
           
           #if defined(LCD_KEYPAD)
             lcd.setCursor(0, 1);  
             lcd.print("Mowing          ");
             #endif
                      
           Motor_Action_Go_Mowing_Speed();
           Compass_Steering_Status = 0;
           }

        
        if ((Compass_Activate == 1) && (Compass_Heading_Hold_Enabled == 1) && (GYRO_Enabled == 0) )  {            // if the Mower is tracking using the compass steer here
            if ( (Loop_Cycle_Mowing % 20) == 0 ) {
                Get_Compass_Reading(); 
                Compass_Steering_Status = 1;        
                Calculate_Compass_Wheel_Compensation();
                Motor_Action_Dynamic_PWM_Steering();              // Removes the full speed function if the mower is trying to hold to the compass heading.
                Print_LCD_Compass_Mowing();
                Serial.print(F("C-Lock:ON_"));
                Serial.print("|");
                }
        }

        if (GYRO_Enabled == 1) {            // if the Mower is tracking using the compass steer here
      if ( (Loop_Cycle_Mowing % 20) == 0 ) {
          Get_GYRO_Reading(); 
          GYRO_Steering_Status = 1;        
          Calculate_GYRO_Wheel_Compensation();
          Motor_Action_Dynamic_PWM_Steering();              // Removes the full speed function if the mower is trying to hold to the compass heading.
          Print_LCD_Compass_Mowing();
          Serial.print(F("G-Lock:ON_"));
          Serial.print("|");
          }
        }
      
      }
        
        
       if (Pattern_Mow == 1) {
           Serial.print("||Mow");
           Pattern_Mow_Parallel();
           }
           
       if (Pattern_Mow == 2) {
          Serial.print("@Mow");
          Pattern_Mow_Spirals();                                // For pattern mow = 2 i.e. circular motion.
          }

       if (Pattern_Mow == 3) {
          Serial.print("Wire Mow");
          Pattern_Mow_Wire();                                // For pattern mow = 2 i.e. circular motion.
          }
    
      }

  // Max Cycles logic decides how far the Mower should run before turning around
  // This is useful is the mower gets hooked on soemthing and the wire and sonar sensors dont react.
  // The logic is now dependant on the type of mowing selected.  For Spiral mowing the max cycles are increased as the spiral mowing requires
  // more cycles to complete a decent sized spiral pattern, followed by a straight line connection leg to the next spiral
  
  long int Max_Cycles_Active;                                                                   // define a veriable to hold the max cycles
  if (Pattern_Mow != 2 ) Max_Cycles_Active                        = Max_Cycles_Straight;        // if normal straight line mowing is slected
  if ((Pattern_Mow == 2 ) && (Spiral_Mow == 3)) Max_Cycles_Active = Max_Cycles_Straight;        // if spiral is selected but its a straight line leg 
  if ((Pattern_Mow == 2 ) && (Spiral_Mow < 3))  Max_Cycles_Active = Max_Cycles_Spirals;         // if spiral is selected and its a LH or RH spiral
  
  if (Loop_Cycle_Mowing > Max_Cycles_Active) {                    // 150 the max length (Max_Cycles) for my garden. Adjust accordingly in the setup
     Serial.println(F(""));
     Serial.println(F("Loop Cycle at Max: Maximum Cycles = "));
     Serial.println(Max_Cycles_Active);
     Serial.println(F(""));
     Motor_Action_Stop_Spin_Blades();                    // Stop the blades from spinning
     Manouver_Turn_Around();                             // Turn around the mower
     Loop_Cycle_Mowing = 0;                              // Restes the loop cycle to start again.
     
     #if defined(LCD_KEYPAD)
       lcd.clear();
       #endif
     }
 }
 
 

void Manouver_Aerate_The_Grass() {

  // Sets the first Drill Target
  if (Loop_Cycle_Mowing == 5) {
    Next_Drill_Target = Loop_Cycle_Mowing + Drill_Spacing; 
    }

  SetPins_ToGoForwards();
  Loop_Cycle_Mowing = (Loop_Cycle_Mowing + 1);
  Serial.print(F("Loop:"));
  Serial.print(Loop_Cycle_Mowing);
  Serial.print(F("|"));

  if (Loop_Cycle_Mowing < Next_Drill_Target)  {
    Motor_Action_Go_Mowing_Speed();
  }
  
  if (Loop_Cycle_Mowing == Next_Drill_Target) {
    Motor_Action_Stop_Motors();
    delay(2000);

    //Insert any checks to make sure Drilling is ok.....
    if (Cutting_Blades_Activate == 1) {
//      Initiate_Drill_Cycle();
      Next_Drill_Target = Loop_Cycle_Mowing + Drill_Spacing;
      }
   else {
    Serial.println(F("Drill Disramed"));
    delay(3000);
    }
   } 

  if (Loop_Cycle_Mowing > Max_Cycles_Straight ) {                    // 150 the max length (Max_Cycles) for my garden. Adjust accordingly in the setup
     Serial.println(F(""));
     Serial.println(F("Loop Cycle at Max"));
     Serial.println(F(""));
     Manouver_Turn_Around();                             // Turn around the mower
     Loop_Cycle_Mowing = 0;                              // Restes the loop cycle to start again.
     }

}

void Manouver_Find_Wire_Track()  {

  Serial.println(F(""));
  Serial.println(F("Find Wire Function Activated"));

  bool Finished_Wire_Find = 0;
  
  #if defined(LCD_KEYPAD)
    lcd.clear();
    lcd.print("Finding Wire...  ");
    #endif
  
  Motor_Action_Stop_Spin_Blades();
  delay(5);
  Abort_Wire_Find = 0;
  No_Wire_Found_Fwd = 0;
  No_Wire_Found_Bck = 0;
  Mower_Running = 0;
  Running_Test_for_Boundary_Wire();                                                                   // Check to see that the wire is on.
  Loop_Cycle_Mowing = 0;
  int cycle = 0; 

  // If the Mower = Parked signal comes then stop the whole process
  while ((Mower_Error == 0) && (Mower_Parked == 0) && (Finished_Wire_Find == 0)) {
        
        
          for (int i = 0; i <= 1; i++) {
        
              Get_WIFI_Commands();
              Serial.print(F("Position Try = "));  
              Serial.println(i);
              ADCMan.run();
              UpdateWireSensor();
              delay(20);
              ADCMan.run();
              UpdateWireSensor();
              delay(20);
              PrintBoundaryWireStatus();
        
            // First go backwards if the mower is outside the wire
            if (( inside == false) && (Abort_Wire_Find == 0) ) {                                    // If the mower is outside the wire then run the following code.
              ADCMan.run();
              UpdateWireSensor();
              PrintBoundaryWireStatus();
              Motor_Action_Stop_Motors();                                                           // Stop all wheel motion
              delay(1000);
              SetPins_ToGoBackwards();                                                                // Set the mower to back up
              delay(100);
              
              #if defined(LCD_KEYPAD)
                lcd.clear();
                lcd.print("Backwards Try...  ");
                lcd.setCursor(0,1);
                lcd.print("Finding Wire  ");
                #endif
              
              delay(100);
              cycle = 0;
        
              // Run the mower backwards until the wire is detetced and the mower is inside the wire
              while (( inside != true) && (Abort_Wire_Find == 0) && (No_Wire_Found_Bck == 0)  ){          // While the mower is still outside the fence run this code
                Loop_Cycle_Mowing = 222;                                                                  // Display this number "222" in the APP under loop cycles.
                cycle = cycle + 1;                                                                        // adds one to the cycle count
                Motor_Action_Go_Full_Speed();                                                             // Go full speed (in this case backwards)
                UpdateWireSensor();                                                                       // Read the wire sensor and see of the mower is now  or outside the wire
                ADCMan.run();
                PrintBoundaryWireStatus();                                                                // Prints of the status of the wire sensor readings.
                Serial.println(F(""));
                if (Manual_Mode == 0) Get_WIFI_Commands(); 
                if (Mower_Parked == 1) {
                  Serial.println("Abort Wire Find");
                  Abort_Wire_Find = 1;
                  }
                if (cycle > Max_Cycle_Wire_Find_Back) {                                                   // Track forwards for Max_Cycle_Wire_Find_Back cycles
                    No_Wire_Found_Bck = 1;                                                                // if mower is still tracking after Max_Cycle_Wire_Find_Back cycles then cancel the find.
                    Serial.println("Max Backward Cycles reached");
                  }
                
                }
              
              Motor_Action_Stop_Motors();
              delay(5);
              }
        
            // Code to go forwards until the mower is outside/ON the wire
            if (( inside == true) && (Abort_Wire_Find == 0) && (No_Wire_Found_Fwd == 0) ) {             // If the Mower is situated  the wire then run the following code.
                ADCMan.run();
                UpdateWireSensor();
                Serial.println(F("CODE POSITION - MOTOR FORWARDS LOOP:  If statements"));
                PrintBoundaryWireStatus();
                Motor_Action_Stop_Motors();
                delay(1000);  
                SetPins_ToGoForwards();                                                                 // Set the motors to move the mower forwards
                delay(100);
                
                #if defined(LCD_KEYPAD)
                lcd.clear();
                lcd.print("Forward Try...  ");
                lcd.setCursor(0,1);
                lcd.print("Finding Wire  ");
                #endif
                
                delay(100);
                cycle = 0;                                                                              // resets the cycles
                
                // Move the mower forwards until the wire is detected and the mower is then outside the wire
                while (( inside != false) && (No_Wire_Found_Fwd == 0) && (Mower_Parked == 0) ) {             // Move the mower forward until mower is outisde/ON the wire fence or 500 cycles have passed
                  Loop_Cycle_Mowing = 111;                                                              // Displays 111 in the APP
                  cycle = cycle + 1;
                  
                  #if defined(LCD_KEYPAD)
                    lcd.setCursor(0,1);
                    lcd.print("Track -> Charge"); 
                    #endif
                  
                  Motor_Action_Go_Full_Speed();                                                         // Go full speed (in this case forwards)
//                  Check_Bumper();                                                                       // Check if the bumper is hit
                  UpdateWireSensor();                                                                   // Read the wire sensor and see of the mower is now  or outside the wire
                  ADCMan.run();
                  PrintBoundaryWireStatus();                                                            // Prints of the status of the wire sensor readings.
                  Serial.println(F(""));
                  if (Manual_Mode == 0) Get_WIFI_Commands(); 
                  if (Mower_Parked == 1) {
                    Serial.println("Abort Wire Find");
                    Abort_Wire_Find = 1;
                    }
//                  if (Bumper == true) {
//                    Motor_Action_Stop_Motors();
//                    SetPins_ToGoBackwards();
//                    Motor_Action_Go_Full_Speed();
//                    delay(Mower_Reverse_Delay);
//                    Motor_Action_Stop_Motors();
//                    Bumper = false; 
//                    delay(4000);
//                    cycle = Max_Cycle_Wire_Find;
//                    }
                  
                  if (cycle > Max_Cycle_Wire_Find) {                                                    // Track forwards for Max_Cycle_Wire_Find_Fwd cycles
                    No_Wire_Found_Fwd = 1;                                                              // if mower is still tracking after Max_Cycle_Wire_Find_Fwd cycles then cancel the find.
                    Serial.println("Max Forward Cycles reached");
                  }
               
                }
              Motor_Action_Stop_Motors();
              delay(5);
            }
          Motor_Action_Stop_Motors();
          delay(1000);
          }
          
        
          // Position the mower further over the wire so it has space to turn 90° onto the wire.
          if ( (Abort_Wire_Find == 0) && (No_Wire_Found_Fwd == 0) ) {
            SetPins_ToGoForwards();                                           
            delay(100);
            bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;
            Ramp_Motor_ON = 0;
            Motor_Action_Go_Full_Speed();
            if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);
            if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
            delay(500);
            Motor_Action_Stop_Motors();
            }
        
          // Sets the firection of spin depensing on if the mower is eciting or tracking home
        
          // Set pins to track home to charge.
          if (Mower_Track_To_Charge == 1) {
            
            #if defined(LCD_KEYPAD)
            lcd.setCursor(0,1);
            lcd.print("Track -> Charge"); 
            #endif
            
            delay(1000);
               if (CW_Tracking_To_Charge == 1) {
                  SetPins_ToTurnRight();                                                                // Track perimeter wire in a Clockwise Direction to the charging station
                  Serial.println(F("CW Tracking to Charger"));
               }
               if (CCW_Tracking_To_Charge == 1) {
                  SetPins_ToTurnLeft(); 
                  Serial.println(F("CCW Tracking toCharger")); 
               }
            }
        
          // Set pins to track to exit.
          if ((Mower_Track_To_Exit == 1) && (Mower_Parked == 0)) {
        
            #if defined(LCD_KEYPAD)
            lcd.setCursor(0,1);
            lcd.print("Track -> Exit"); 
            #endif
            
            delay(1000);
               if (CW_Tracking_To_Start == 1) {
                  SetPins_ToTurnRight();                                                                // Track perimeter wire in a Clockwise Direction to the charging station
                  Serial.println(F("CW Tracking to Exit"));
               }
               if (CCW_Tracking_To_Start == 1) {
                  SetPins_ToTurnLeft(); 
                  Serial.println(F("CCW Tracking to Exit")); 
               }
            }
        
          
          delay(20);
          // Pins are now set to turn from the above logic which way to turn onto the wire
          
          // Update the mowers position to the wire.
          ADCMan.run();
          UpdateWireSensor();
          ADCMan.run();
          PrintBoundaryWireStatus();                                                                  // Prints of the status of the wire sensor readings
          delay(20);
        
          int Max_Spin_Attempts = 300;
          int Spin_Attempts = 0;
          int WIFI_Check_Up;
        
          #if defined(LCD_KEYPAD)
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Spin To Wire");
              #endif
        
          Serial.println(F(""));
          Serial.print(F("Turning onto Wire:"));
          Serial.println(F(""));
        
          // Spins the mower over the wire in the driection of tracking
          while (( inside == false)  && (Abort_Wire_Find == 0) && (No_Wire_Found_Fwd == 0)  && (Spin_Attempts < Max_Spin_Attempts )) {
                while (( inside != true) && (Spin_Attempts < Max_Spin_Attempts )) {                                                             // Do this loop until mower is back  the wire fence
                Serial.println(F(""));
                Serial.print(F("Turn on Wire Attempts:"));
                Serial.print(Spin_Attempts);
                Serial.print(F("|:"));
                  #if defined(LCD_KEYPAD)
                  lcd.setCursor(0,1);       
                  lcd.print(Spin_Attempts);
                  #endif
                bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
                Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
                Motor_Action_Go_Full_Speed();
                if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
                if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
                UpdateWireSensor();                                                                   // Read the wire sensor and see of the mower is now  or outside the wire
                ADCMan.run();
                PrintBoundaryWireStatus();                                                            // Prints of the status of the wire sensor readings.
                Spin_Attempts = Spin_Attempts + 1;                                                    // checks that the mower is not blocked trying to spin on the wire and gets stuck in this loop
                WIFI_Check_Up = WIFI_Check_Up + 1;
                if (WIFI_Check_Up == 20) {
                  if (Manual_Mode == 0) Get_WIFI_Commands(); 
                  WIFI_Check_Up = 0;
                  }
                              
                }
            Motor_Action_Stop_Motors();                                                                 // Stop the mower on the wire facing the correct direction.
            }
          
          Motor_Action_Stop_Motors();
          if ((Abort_Wire_Find == 0) && (Spin_Attempts < Max_Spin_Attempts )) {
            Serial.println(F("Track Wire Function Complete - ON WIRE??"));
            
            #if defined(LCD_KEYPAD)
              lcd.clear();
              lcd.print("Wire Found...");
              #endif
            
            delay(2000);                                                                                // 2 second pause to check result
            SetPins_ToGoForwards();                                           
            delay(100);
        
            #if defined(LCD_KEYPAD)
              lcd.clear();
              lcd.setCursor(0,1);
              lcd.print("Track -> Charge"); 
              #endif
            }
        
          // if an abort has been received or the mower is not spinning to the right diection on the the wire then park it.
          if ((Abort_Wire_Find == 1) || (Spin_Attempts >= Max_Spin_Attempts))  {
            Serial.println("Wire Find Aborted");
            #if defined(LCD_KEYPAD)
            lcd.clear();
            lcd.print("Wire Find ABORT!!");
            #endif
            
            delay(2000);       
            Abort_Wire_Find = 0;
            SetPins_ToGoForwards();
            Manouver_Park_The_Mower();
            }
        
          if (No_Wire_Found_Fwd == 1) {
            Serial.println("Re-starting wire find");
            SetPins_ToGoForwards();
            }
  Finished_Wire_Find = 1;
  }

Motor_Action_Stop_Motors(); 

if (Mower_Error == 1) {
  Serial.println(F(""));
  Serial.println(F("Mower in Error Mode"));
  Serial.println(F(""));
  }

}


    

// Turn Around defines how the mower should react when a wire or sonar sensor is activated.
// Now with the spiral pattern mowing is also decides the logic of which Spiral shape will be next

void Manouver_Turn_Around() {
#if not defined(NODELAY_BACKWARD)

    Motor_Action_Stop_Motors(); 
    if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
    delay(100);
    
    Serial.println(F(""));
    Serial.println(F(""));
    if (Outside_Wire == 1)                                                  Serial.println(F("Mower is Outside the Wire"));
    if ((GPS_Enabled == 1) && (GPS_Type == 1) && (GPS_Inside_Fence == 0))   Serial.println(F("Mower is Outside the GPS Fence"));
    if (Wheel_Blocked == 4)                                                 Serial.println(F("Mower Wheels are jammed"));
    
    if (TFT_Screen_Menu == 1) {
        if (Robot_Type == 1) Send_Mower_Running_Data();    
//        if (Robot_Type == 2) Send_Aerator_Running_Data();         // Update TFT Screen
        }

    Serial.println(F("Mower is Turning"));
    Serial.println(F(""));
    delay(80);

    // Back up the mower
    SetPins_ToGoBackwards();
    Motor_Action_Go_Full_Speed();
    delay(Mower_Reverse_Delay);
    Motor_Action_Stop_Motors(); 

    // Randomly decide if the mower should turn left or right depending on if the loop cycle is odd or even
    if ( (Loop_Cycle_Mowing % 2) == 0 ) {
     SetPins_ToTurnRight(); 
    }
    else SetPins_ToTurnLeft();

    // Randomly turns the mower to a new heading depending on the delay Min or Delay Max from the settings
    Motor_Action_Turn_Speed();
    delay (random(Mower_Turn_Delay_Min, Mower_Turn_Delay_Max));

    // If Spiral Mode is activated
    // Advances the type of movement in Pattern Spiral Mode from : 1 RH Spiral | 2 LH Spiral | 3 Straight line.
    if (Pattern_Mow == 2) {
    Spiral_Mow = (Spiral_Mow + 1);
    if (Spiral_Mow > 3) Spiral_Mow = 1;
    }
    //Spiral (random(1,3));  Activate this to make a true random pattern.

    Motor_Action_Stop_Motors();
    
    // If the perimeter wire is enabled
    // Check that the mower has turned and moved back inside the boundary wire.
    if (Perimeter_Wire_Enabled == 1) {
        Check_Wire_In_Out();
    
        if (Outside_Wire == 1) { 
          Serial.println(F("Outside Wire 1"));
          Running_Test_for_Boundary_Wire(); 
          delay(100);
          UpdateWireSensor();
          Check_Wire_In_Out();
              if (Outside_Wire == 1) { 
                Serial.println(F("Outside Wire 2"));
                //SetPins_ToGoBackwards();
                //Motor_Action_Go_Full_Speed();
                //delay(300);
                Motor_Action_Stop_Motors();
                delay(20);
                Running_Test_for_Boundary_Wire(); 
                delay(80);
                UpdateWireSensor();
                Check_Wire_In_Out();
                if (Outside_Wire == 1) { 
                      Serial.println(F("Outside Wire = 3 - Must be really outside...."));
                      //SetPins_ToGoForwards();
                      //Motor_Action_Go_Full_Speed();
                      //delay(300);
                      Motor_Action_Stop_Motors();
                      if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
                      delay(20);
                      Running_Test_for_Boundary_Wire(); 
                      delay(80);
                      UpdateWireSensor();
                      Check_Wire_In_Out();
                      }
                }
    
         }
        }

    // If the GPS Fencing is enabled
    // Check that the mower has turned and moved back inside the GPS Fence.
    if ((GPS_Enabled == 1) && (GPS_Type == 1)) {
        Check_GPS_In_Out();       
    
        if (GPS_Inside_Fence == 0) {                                  // Mower is still outside the fence
          Serial.println(F("Outside GPS Fence 1"));
          delay(100);
          Check_GPS_In_Out();                                         // Check the GPS signal again
              if (GPS_Inside_Fence == 0) { 
                Serial.println(F("Outside GPS Fence 2"));
                delay(1000);
                Check_GPS_In_Out(); 
                if (GPS_Inside_Fence == 0) { 
                      Serial.println(F("Outside GPS Fence = 3 - Must be really outside...."));
                      }
                }
    
         }
        }
    
    // Reset the sonar 
    Sonar_Status = 0;
    Sonar_Hit = 0;                                                        // Reset Sonar
    Sonar_Hit_1_Total = 0;
    Sonar_Hit_2_Total = 0;
    Sonar_Hit_3_Total = 0;
    Skip_Sonar_Turn = 1;
    distance1 = 999;
    distance2 = 999;
    distance3 = 999;

//    Bumper = false;                                                           // Reset Bumper
    Loop_Cycle_Mowing = 1;                                                // Rest Loop Cycle
    Compass_Heading_Locked = 0;                                           // Reset Compass Heading Lock
    
    #if defined(LCD_KEYPAD)
    lcd.clear();
    #endif
    
    if (TFT_Screen_Menu == 1) {
      if (Robot_Type == 1) Send_Mower_Running_Data();
//      if (Robot_Type == 2) Send_Aerator_Running_Data();
      }
    Serial.println(F(""));
    Serial.println(F("Mower Turned Around - Wire/GPS"));
    Serial.println(F(""));

#else
  if(Mower_RunBack == 0) {
      Manouver_Turn_Around_Phase = 0;
      Manouver_Turn_Around_Sonar_Phase = 0; // just for sure
      Mower_RunBack = 1;
    }
#endif
}


void Manouver_Turn_Around_non_blocking() {
#if defined(NODELAY_BACKWARD)
  Serial.print(F("RunBackPhase:"));
  Serial.print(Manouver_Turn_Around_Phase);

  switch (Manouver_Turn_Around_Phase)
  {
  case 0:
    if (Mower_RunBack == 1) {
      Manouver_Turn_Around_Phase++;
    }
    break;

  case 1:
    Serial.println(F("Manouver_Turn_Around starting.."));
    Motor_Action_Stop_Motors();
    if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU

    Serial.println(F(""));
    Serial.println(F(""));
        if (Outside_Wire == 1)                                                  Serial.println(F("Mower is Outside the Wire"));
        if ((GPS_Enabled == 1) && (GPS_Type == 1) && (GPS_Inside_Fence == 0))   Serial.println(F("Mower is Outside the GPS Fence"));
        if (Wheel_Blocked == 4)                                                 Serial.println(F("Mower Wheels are jammed"));
//        if (Bumper == true)                                                       Serial.println(F("Mower bumper hit object"));

    if (TFT_Screen_Menu == 1) {
      if (Robot_Type == 1) Send_Mower_Running_Data();
//      if (Robot_Type == 2) Send_Aerator_Running_Data();         // Update TFT Screen
      }

    Serial.println(F("Mower is Turning"));
    Serial.println(F(""));
    Manouver_Turn_Around_Phase++;
    break;

  case 2:
    if (TimerDelayOn(T_MTA_ph2, 500)) Manouver_Turn_Around_Phase++;
    break;

  case 3:
    // Back up the mower
    SetPins_ToGoBackwards();
    Motor_Action_Go_Full_Speed();

    Manouver_Turn_Around_Phase++;
    break;

  case 4:
    if (TimerDelayOn(T_MTA_ph4, Mower_Reverse_Delay)) Manouver_Turn_Around_Phase++;
    break;

  case 5:
    Motor_Action_Stop_Motors();
    if (TimerDelayOn(T_MTA_ph5, 500)) Manouver_Turn_Around_Phase++;
    break;

  case 6:
    // Randomly decide if the mower should turn left or right depending on if the loop cycle is odd or even
    if ( (Loop_Cycle_Mowing % 2) == 0 ) {
     SetPins_ToTurnRight();
    }
    else SetPins_ToTurnLeft();

    // Randomly turns the mower to a new heading depending on the delay Min or Delay Max from the settings
    Motor_Action_Turn_Speed();

    Manouver_Turn_Around_Phase++;
    break;

  case 7:
    if (TimerDelayOn(T_MTA_ph7, random(Mower_Turn_Delay_Min, Mower_Turn_Delay_Max))) Manouver_Turn_Around_Phase++;
    break;

  case 8:
    // If Spiral Mode is activated
    // Advances the type of movement in Pattern Spiral Mode from : 1 RH Spiral | 2 LH Spiral | 3 Straight line.
    if (Pattern_Mow == 2) {
    Spiral_Mow = (Spiral_Mow + 1);
    if (Spiral_Mow > 3) Spiral_Mow = 1;
    }
    //Spiral (random(1,3));  Activate this to make a true random pattern.

    Motor_Action_Stop_Motors();

    Manouver_Turn_Around_Phase++;
    break;

  case 9:
    // If the perimeter wire is enabled
    // Check that the mower has turned and moved back inside the boundary wire.
    if (Perimeter_Wire_Enabled == 1) {
      Check_Wire_In_Out();

      if (Outside_Wire == 1) {
        Serial.println(F("Outside Wire 1"));
        Running_Test_for_Boundary_Wire();
        delay(100);
        UpdateWireSensor();
        Check_Wire_In_Out();
          if (Outside_Wire == 1) {
          Serial.println(F("Outside Wire 2"));
          //SetPins_ToGoBackwards();
          //Motor_Action_Go_Full_Speed();
          //delay(300);
          Motor_Action_Stop_Motors();
          delay(20);
          Running_Test_for_Boundary_Wire();
          delay(80);
          UpdateWireSensor();
          Check_Wire_In_Out();
          if (Outside_Wire == 1) {
              Serial.println(F("Outside Wire = 3 - Must be really outside...."));
              //SetPins_ToGoForwards();
              //Motor_Action_Go_Full_Speed();
              //delay(300);
              Motor_Action_Stop_Motors();
              if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
              delay(20);
              Running_Test_for_Boundary_Wire();
              delay(80);
              UpdateWireSensor();
              Check_Wire_In_Out();
              }
          }

       }
      }

    // If the GPS Fencing is enabled
    // Check that the mower has turned and moved back inside the GPS Fence.
    if ((GPS_Enabled == 1) && (GPS_Type == 1)) {
      Check_GPS_In_Out();

      if (GPS_Inside_Fence == 0) {                                  // Mower is still outside the fence
        Serial.println(F("Outside GPS Fence 1"));
        delay(100);
        Check_GPS_In_Out();                                         // Check the GPS signal again
          if (GPS_Inside_Fence == 0) {
          Serial.println(F("Outside GPS Fence 2"));
          delay(1000);
          Check_GPS_In_Out();
          if (GPS_Inside_Fence == 0) {
              Serial.println(F("Outside GPS Fence = 3 - Must be really outside...."));
              }
          }

       }
      }

    // Reset the sonar
    Sonar_Status = 0;
    Sonar_Hit = 0;                                                        // Reset Sonar
    Sonar_Hit_1_Total = 0;
    Sonar_Hit_2_Total = 0;
    Sonar_Hit_3_Total = 0;
    Skip_Sonar_Turn = 1;
    distance1 = 999;
    distance2 = 999;
    distance3 = 999;

//    Bumper = false;                                                           // Reset Bumper
    Loop_Cycle_Mowing = 1;                                                // Rest Loop Cycle
    Compass_Heading_Locked = 0;                                           // Reset Compass Heading Lock
    //Wheel_Blocked = 0;

    #if defined(LCD_KEYPAD)
    lcd.clear();
    #endif

    if (TFT_Screen_Menu == 1) {
      if (Robot_Type == 1) Send_Mower_Running_Data();
//      if (Robot_Type == 2) Send_Aerator_Running_Data();
      }
    Serial.println(F(""));
    Serial.println(F("Mower Turned Around - Wire/GPS/Bumper/Wheels jammed"));
    Serial.println(F(""));

    Manouver_Turn_Around_Phase = 0;
    Mower_RunBack = 0;
    break;
  }
#endif
}
  

void Manouver_Turn_Around_Sonar() {
#if not defined(NODELAY_BACKWARD)
  Sonar_Status = 1;
  Serial.println(F("Mower is Turning - Sonar"));
  Motor_Action_Stop_Motors(); 
  
  if (TFT_Screen_Menu == 1) {
     if (Robot_Type == 1) Send_Mower_Running_Data();
//     if (Robot_Type == 2) Send_Aerator_Running_Data();   // Update TFT Screen
     }
     
  delay(500);
  SetPins_ToGoBackwards();
  Motor_Action_Go_Full_Speed();
  delay(Mower_Reverse_Delay);
  Motor_Action_Stop_Motors(); 

  if ( ( (distance1 < maxdistancesonar) ||  (distance2 < maxdistancesonar) ) && (Skip_Sonar_Turn == 0)) {
      #if defined(LCD_KEYPAD)
        lcd.setCursor(0, 8);
        lcd.print("Go Right -->       ");
        #endif
      
      Serial.println(F(""));
      Serial.println(F("Sonar Turning Right"));
      if (distance1 < maxdistancesonar) Serial.println(F("Distance 1 Triggered"));
      if (distance2 < maxdistancesonar) Serial.println(F("Distance 2 Triggered"));
      Serial.println(F(""));
      SetPins_ToTurnRight();
      Motor_Action_Turn_Speed();
      delay(Mower_Turn_Delay_Max);
      Skip_Sonar_Turn = 1;                                      // cancels the distance 3 turn if it ia also activated.
      }

  if ((distance3 < maxdistancesonar) && (Skip_Sonar_Turn == 0)) {
     
      #if defined(LCD_KEYPAD)
        lcd.setCursor(0, 8);
        lcd.print("<-- Go Left      ");
        #endif

      Serial.println(F("Sonar Turning Left"));
      if (distance1 < maxdistancesonar) Serial.println(F("Distance 3 Triggered"));
      Serial.println(F(""));
      SetPins_ToTurnLeft();
      Motor_Action_Turn_Speed();
      delay(Mower_Turn_Delay_Max);
      Skip_Sonar_Turn = 1;
      }
  
  Motor_Action_Stop_Motors();
  if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
  Compass_Heading_Locked = 0;
  Sonar_Hit = 0;
  Loop_Cycle_Mowing = 0;
  Sonar_Status = 0;
  Wheel_Blocked = 0;
  if (TFT_Screen_Menu == 1) {
    if (Robot_Type == 1) Send_Mower_Running_Data();
//    if (Robot_Type == 2) Send_Aerator_Running_Data();
    }
  Serial.println(F("Mower Turned Around from Sonar Hit"));
  Serial.println(F(""));
  Skip_Sonar_Turn = 0;

#else
  if(Mower_RunBack == 0) {
    Manouver_Turn_Around_Sonar_Phase = 0;
    Manouver_Turn_Around_Phase = 0; // just for sure
    Mower_RunBack = 2;
  }
#endif
  }


void Manouver_Turn_Around_Sonar_non_blocking() {
#if defined(NODELAY_BACKWARD)
  Serial.print(F("RunBackPhaseS:"));
  Serial.print(Manouver_Turn_Around_Sonar_Phase);

  switch (Manouver_Turn_Around_Sonar_Phase)
  {
  case 0:
    if (Mower_RunBack == 2) {
      Manouver_Turn_Around_Sonar_Phase++;
    }
    break;

  case 1:
    Sonar_Status = 1;
    Serial.println(F("Mower is Turning - Sonar"));
    Motor_Action_Stop_Motors();

    if (TFT_Screen_Menu == 1) {
     if (Robot_Type == 1) Send_Mower_Running_Data();
//     if (Robot_Type == 2) Send_Aerator_Running_Data();   // Update TFT Screen
     }
    Manouver_Turn_Around_Sonar_Phase++;
    break;

  case 2:
    if (TimerDelayOn(T_MTAS_ph2, 500) || Wheel_Blocked == 4) Manouver_Turn_Around_Sonar_Phase++;
    break;

  case 3:
    SetPins_ToGoBackwards();
    Motor_Action_Go_Full_Speed();

    Manouver_Turn_Around_Sonar_Phase++;
    break;

  case 4:
    if (TimerDelayOn(T_MTAS_ph4, Mower_Reverse_Delay) || Wheel_Blocked == 4) Manouver_Turn_Around_Sonar_Phase++;
    break;

  case 5:
    Motor_Action_Stop_Motors();
    if (TimerDelayOn(T_MTAS_ph5, 500)) Manouver_Turn_Around_Sonar_Phase++;
    break;

  case 6:
    if ( ( (distance1 < maxdistancesonar) ||  (distance2 < maxdistancesonar) ) && (Skip_Sonar_Turn == 0)) {
      #if defined(LCD_KEYPAD)
      lcd.setCursor(0, 8);
      lcd.print("Go Right -->       ");
      #endif

      Serial.println(F(""));
      Serial.println(F("Sonar Turning Right"));
      if (distance1 < maxdistancesonar) Serial.println(F("Distance 1 Triggered"));
      if (distance2 < maxdistancesonar) Serial.println(F("Distance 2 Triggered"));
      Serial.println(F(""));
      SetPins_ToTurnRight();
    }

    if ((distance3 < maxdistancesonar) && (Skip_Sonar_Turn == 0)) {

      #if defined(LCD_KEYPAD)
      lcd.setCursor(0, 8);
      lcd.print("<-- Go Left      ");
      #endif

      Serial.println(F("Sonar Turning Left"));
      if (distance1 < maxdistancesonar) Serial.println(F("Distance 3 Triggered"));
      Serial.println(F(""));
      SetPins_ToTurnLeft();
    }

      Motor_Action_Turn_Speed();
      Skip_Sonar_Turn = 1;                                      // cancels the distance 3 turn if it ia also activated.

      Manouver_Turn_Around_Sonar_Phase++;
      break;

  case 7:
      if (TimerDelayOn(T_MTAS_ph7, Mower_Turn_Delay_Max) || Wheel_Blocked == 4) Manouver_Turn_Around_Sonar_Phase++;
      break;

  case 8:
      Motor_Action_Stop_Motors();
      if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
      Compass_Heading_Locked = 0;
      Sonar_Hit = 0;
      Loop_Cycle_Mowing = 0;
      Sonar_Status = 0;
      Wheel_Blocked = 0;
      if (TFT_Screen_Menu == 1) {
      if (Robot_Type == 1) Send_Mower_Running_Data();
//      if (Robot_Type == 2) Send_Aerator_Running_Data();
      }
      Serial.println(F("Mower Turned Around from Sonar Hit"));
      Serial.println(F(""));
      Skip_Sonar_Turn = 0;

      Manouver_Turn_Around_Sonar_Phase = 0;
      Mower_RunBack = 0;
      break;
  }
#endif
  }


void Manouver_Manual_Mode() {
  Mower_Docked          = 0;
  Mower_Parked          = 0;
  Mower_Running         = 0;
  Mower_Parked_Low_Batt = 0;
  Mower_Track_To_Exit   = 0;
  Mower_Track_To_Charge = 0;
  Exiting_Dock          = 0;
  Mower_Error           = 0;
  Manual_Mode           = 1;
  Mower_Setup_Mode      = 0;
  Loop_Cycle_Mowing     = 0;
  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades();

  Serial.println(F("Sending Manual Status to TFT"));
  Serial.println(F(""));  
  
  
  
  
  Robot_Status_Value = 6;
  Send_Mower_Docked_Data();         // Send docked numbers to break out of the cycle and change it to Mower Setup Mode

  
}


void Manouver_Setup_Mode() {
  
  
  Mower_Docked          = 0;
  Mower_Parked          = 0;
  Mower_Running         = 0;
  Mower_Parked_Low_Batt = 0;
  Mower_Track_To_Exit   = 0;
  Mower_Track_To_Charge = 0;
  Exiting_Dock          = 0;
  Mower_Error           = 0;
  Manual_Mode           = 0;
  Mower_Setup_Mode      = 1;
  Loop_Cycle_Mowing     = 0;

  Serial.println(F("Sending Setup Status to TFT"));
  Serial.println(F("")); 
  Robot_Status_Value = 3;
  Send_Mower_Docked_Data();         // Send docked numbers to break out of the cycle and change it to Mower Setup Mode
  
  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades(); 
  }


void Manouver_Start_Mower() {

  Serial.println(F(""));
  Serial.println(F("Starting to Mow!! "));   
  Serial.println(F(""));
  
  if ((Mower_Track_To_Exit == 1) && (TFT_Screen_Menu == 1)) {
    Mower_Track_To_Exit   = 0;
    Exiting_Dock          = 0;
    Mower_Running         = 1;
    Calculate_TFT_Robot_Status_Value(); 
    Send_Mower_Tracking_Data();
    }

  // If wire is not detected when exiting the dock do the following
  if ((Mower_Docked == 1) && (Exiting_Dock == 1))   {                  
        Serial.println(F(""));
        Serial.println(F("Mower = Docked | Quick Start"));
        Mower_Track_To_Exit   = 0;
        Exiting_Dock          = 0;
        Mower_Parked          = 0;
        Mower_Running         = 1;
        Send_Mower_Docked_Data();        
        } 

   
  Mower_Docked          = 0;
  Mower_Parked          = 0;
  Mower_Setup_Mode      = 0;
  Mower_Running         = 1;
  Transmit_All_To_NODEMCU();
  Mower_Parked_Low_Batt = 0;
  Mower_Track_To_Charge = 0;
  Rain_Hit_Detected     = 0;
  Mower_Error           = 0;
  Exiting_Dock          = 0;
  Loop_Cycle_Mowing     = 0;
  Manual_Mode           = 0;
  Wire_Refind_Tries     = 0;
  Calculate_TFT_Robot_Status_Value(); 
  Transmit_All_To_NODEMCU();
  Turn_On_Relay();
  Y_Tilt = 0;
//  if (Robot_Type == 2) Ensure_Drills_Are_Restarcted();
  Run_Initial_Boundary_Wire_Test();             // Run the boundary wire test row.
 
  
  // If wire is not detected when exiting the dock do the following
  if ((Wire_Detected == 0) && ((Exiting_Dock == 1) || (Mower_Running == 1)) )   {                  
        Serial.println(F(""));
        Serial.println(F("Mower = Running | Exit Dock = 1"));
        Serial.println(F("Perimeter Wire Still not detected"));
        //Manouver_Park_The_Mower();
        Serial.println(F("Sending Tracking Data in Wire detect = 0"));
        Robot_Status_Value = 4;         // Sends the mower status value to the TFT Screen 4 = Error Status.
        Mower_Error_Value = 1;          // Describes the error 1 = No Wire, 
        if (Mower_Running == 1) {
            if (TFT_Screen_Menu == 1) {
              if (Robot_Type == 1) Send_Mower_Running_Data();
//              if (Robot_Type == 2) Send_Aerator_Running_Data();
              }    
        }
        if (Exiting_Dock == 1)  Send_Mower_Tracking_Data();
        delay(5000);
        Manouver_Park_The_Mower();
        } 
  
  // If wire is not detected when the mower is running then send the mower into an error state.
  if ((Perimeter_Wire_Enabled == 0) && (Mower_Running == 1)) {                     
        Serial.println(F(""));
        Serial.println(F("Perimeter Wire Not Activated"));
        Calculate_TFT_Robot_Status_Value(); 
        Send_Mower_Tracking_Data();       
        }
  
  }

void Manouver_Mower_Exit_Dock() {
  
  Mower_Docked          = 0;
  Mower_Parked          = 0;
  Mower_Running         = 0;
  Mower_Parked_Low_Batt = 0;
  Rain_Hit_Detected     = 0;
  Mower_Error           = 0;
  Manual_Mode           = 0;
  Mower_Setup_Mode      = 0;
  Tracking_Wire         = 0;
  Exiting_Dock          = 1;  
  Robot_Status_Value    = 9;

  
  Serial.println(F(""));
  Serial.println(F("Updating TFT with Exit Dock Information"));
  Serial.println(F(""));
  if ((Mower_Docked == 1) || (Mower_Parked == 1))  Send_Mower_Docked_Data();
  else Send_Mower_Tracking_Data();               // Send docked numbers to break out of the cycle and change it to mower exiting dock mode.    
  Serial.println(F(""));                    // Send Command to the TFT
  Get_WIFI_Commands();                      // Command WIFI
  }




void Manouver_Dock_The_Mower() {
  if (TFT_Screen_Menu == 1) {
    Turn_To_Home              = 0;
    Find_Wire_Track           = 0;
    Go_To_Charging_Station    = 0;
    Mower_Docked              = 1;
    Tracking_Wire             = 0;
    Mower_Track_To_Charge     = 0;
    Calculate_TFT_Robot_Status_Value();
    Send_Mower_Tracking_Data();
    }
  
  Mower_Docked          = 1;
  Mower_Parked          = 0;
  Mower_Setup_Mode      = 0;
  Mower_Running         = 0;
  Mower_Parked_Low_Batt = 0;
  Mower_Track_To_Exit   = 0;
  Mower_Track_To_Charge = 0;
  Mower_Error           = 0;
  Loop_Cycle_Mowing     = 0;
  Manual_Mode           = 0;
  Exiting_Dock          = 0;
#if defined(NODELAY_BACKWARD)
  Mower_RunBack     = 0;
#endif // -(NODELAY_BACKWARD)-
  if (TFT_Screen_Menu == 0) {
    Calculate_TFT_Robot_Status_Value();
  }
  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades();
  Turn_Off_Relay();
  Print_LCD_Info_Docked();
  Charge_Detected_MEGA = 0;
  //Setup Alarms 
  Alarm_Timed_Mow_ON = 0;                                           // Turns off the 1 hr Alarm
 
}

// Mower is a parked position and needs manual charging
void Manouver_Park_The_Mower_Low_Batt() {

  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades();
 
  if ((Exiting_Dock == 1) && (TFT_Screen_Menu == 1))  {
        Serial.println(F("Sending TFT Tracking Data"));
        Mower_Parked_Low_Batt = 1;
        Mower_Error           = 1;
        Exiting_Dock          = 0;
        Calculate_TFT_Robot_Status_Value();               // Updates the Mower status Value
        Send_Mower_Tracking_Data();
        }
  if ((Mower_Running == 1) &&  (TFT_Screen_Menu == 1)) {
        Serial.println(F("Sending TFT Running Data"));
        Mower_Parked_Low_Batt = 1;
        Mower_Error           = 1;
        Mower_Running         = 0;
        Calculate_TFT_Robot_Status_Value();               // Updates the Mower status Value
       if (Robot_Type == 1) Send_Mower_Running_Data();
//       if (Robot_Type == 2) Send_Aerator_Running_Data();
    }
        
  
  Mower_Parked_Low_Batt = 1;
  Mower_Docked          = 0;
  Mower_Parked          = 0;
  Mower_Running         = 0;
  Mower_Setup_Mode      = 0;
  Mower_Track_To_Charge = 0;
  Mower_Error           = 0;
  Loop_Cycle_Mowing     = 0;
  Manual_Mode           = 0;


}


// Mower is in a parked or paused potion ready to restart
void Manouver_Park_The_Mower() {

  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades();

  #if defined(LCD_KEYPAD) 
    if (Mower_Parked == 0 ) lcd.clear();
  #endif

  // if the mower is tracking when the park button is pressed send 
  // information to the TFT via the "Tracking_Data"
  if ((TFT_Screen_Menu == 1) && (Mower_Running == 0) && (Mower_Error_Value == 0) && ((Tracking_Wire == 1) || (Go_To_Charging_Station == 1) || (Mower_Track_To_Exit == 1))  ) {
        Turn_To_Home              = 0;
        Find_Wire_Track           = 0;
        Go_To_Charging_Station    = 0;
        Mower_Docked              = 0;
        Tracking_Wire             = 0;
        Mower_Track_To_Charge     = 0;
        Mower_Track_To_Exit       = 0;
        Mower_Parked              = 1;
        Mower_Setup_Mode          = 0;
        Mower_Error               = 0;
        Mower_Running             = 0;
        Exiting_Dock              = 0;
       
        //Calculate_TFT_Robot_Status_Value();   // gives wrong value dont use!!
        Robot_Status_Value = 2;
        Serial.print("");
        Serial.print(F("Parking Status 1 - Mower Status Value Sent = "));
        Serial.print(Robot_Status_Value);
        Serial.print("");
        Send_Mower_Tracking_Data();
        }

        
  if ((TFT_Screen_Menu == 1) && (Mower_Running == 1))  {
        Turn_To_Home              = 0;
        Find_Wire_Track           = 0;
        Go_To_Charging_Station    = 0;
        Mower_Docked              = 0;
        Tracking_Wire             = 0;
        Mower_Track_To_Charge     = 0;
        Mower_Track_To_Exit       = 0;
        Mower_Parked              = 1;
        Mower_Setup_Mode          = 0;
        Mower_Error               = 0;
        Mower_Running             = 0;
        Exiting_Dock              = 0;
               
        Calculate_TFT_Robot_Status_Value();     
        Serial.print(F("Parking Status 2-  Mower Status Value Sent = "));
        Serial.print(Robot_Status_Value);
        Serial.print("");
        if (Robot_Type == 1) Send_Mower_Running_Data();
//        if (Robot_Type == 2) Send_Aerator_Running_Data();
        }

  if ((TFT_Screen_Menu == 1) && (Robot_Status_Value == 4))  {
        Turn_To_Home              = 0;
        Find_Wire_Track           = 0;
        Go_To_Charging_Station    = 0;
        Mower_Docked              = 0;
        Tracking_Wire             = 0;
        Mower_Track_To_Charge     = 0;
        Mower_Track_To_Exit       = 0;
        Mower_Parked              = 1;
        Mower_Error               = 0;
        Mower_Setup_Mode          = 0;
        Mower_Running             = 0;
        Exiting_Dock              = 0;              
        Robot_Status_Value        = 2;   
        Mower_Error_Value         = 0;
        Serial.println("");
        Serial.print(F("Parking Status 3 Error -  Mower Status Value Sent = "));
        Serial.print(Robot_Status_Value);
        Serial.println("");
        Send_Mower_Error_Data();
        }

  if (TFT_Screen_Menu == 0) {
    Calculate_TFT_Robot_Status_Value();
  }

 
  Mower_Docked          = 0;
  Mower_Parked          = 1;
  Mower_Running         = 0;
  Mower_Parked_Low_Batt = 0;
  Mower_Track_To_Charge = 0;
  Tracking_Wire         = 0;
  Mower_Track_To_Exit   = 0;
  Exiting_Dock          = 0;
  Mower_Error           = 0;
  Loop_Cycle_Mowing     = 0;
  Manual_Mode           = 0;
  Mower_Setup_Mode      = 0;
#if defined(NODELAY_BACKWARD)
  Mower_RunBack     = 0;
#endif // -(NODELAY_BACKWARD)-
  Turn_Off_Relay();

  Alarm_Timed_Mow_ON = 0;                                           // Turns off the 1 hr Alarm
  //if (Alarm_1_Repeat == 0) Alarm_1_ON = 0;
  //if (Alarm_2_Repeat == 0) Alarm_2_ON = 0;
  //if (Alarm_3_Repeat == 0) Alarm_3_ON = 0;
  }


// Puts the mower to sleep - normally due to an error being found 
void Manouver_Hibernate_Mower() {

  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades();
  Mower_Error           = 1;
  
  // This function requires 2 different strategies to send the correct information to the TFT based on if the mower 
  // is running or exiting the dock.  The Error function is therefore transmitted diffrently depending on the expected
  // Package of daat the TFT is waiting for.  i.e. in Exit Dock Mode the TFT expects the Exit Dock TFT Data Package
  // and will hang if it recieves the running data package.

  Serial.println(F("Hibernate Mower"));
  
  if ((Exiting_Dock == 1) && (TFT_Screen_Menu == 1))  {
        Serial.println(F("Sending TFT Tracking Data"));
        Mower_Error           = 1;
        Exiting_Dock          = 0;
        Mower_Parked          = 0;  
        Calculate_TFT_Robot_Status_Value();               // Updates the Mower status Value
        Send_Mower_Tracking_Data();
        }
  if ((Mower_Running == 1) &&  (TFT_Screen_Menu == 1)) {
        Serial.println(F("Sending TFT Running Data"));        
        Mower_Error           = 1;
        Mower_Running         = 0;
        if (Tilt_Orientation_Sensed == 1)  {
          Serial.println("Tilt Used");
          Mower_Error           = 3;
          Mower_Running         = 0;
          }
        Serial.print(F("Mower Error before calc="));
        Serial.println(Mower_Error);  
        Calculate_TFT_Robot_Status_Value();               // Updates the Mower status Value
        Serial.print(F("Mower Error after calc="));
        Serial.println(Mower_Error);  
        if (Robot_Type == 1) Send_Mower_Running_Data();
//        if (Robot_Type == 2) Send_Aerator_Running_Data();
        }

  if ((Mower_Parked == 1) &&  (TFT_Screen_Menu == 1) && (Tilt_Orientation_Sensed == 0)) {
        Serial.println(F("Sending TFT Parked Data"));
        Mower_Error           = 1;
        Mower_Parked          = 1;
        Calculate_TFT_Robot_Status_Value();               // Updates the Mower status Value
        Send_Mower_Docked_Data();
        }

        Mower_Docked          = 0;  
        Mower_Parked_Low_Batt = 0;
        Mower_Track_To_Charge = 0;
        Tracking_Wire         = 0;
        Mower_Track_To_Exit   = 0;
        Loop_Cycle_Mowing     = 0;
        Manual_Mode           = 0;
 
  // Powers down the mower motors and cuts the main power via the relay. 

  Turn_Off_Relay();
  }



// Mower is sent to the charging station after low volts are detected or mebrane key input.
// Or via the WIFI APP Go to Dock.

void Manouver_Go_To_Charging_Station() {

  Motor_Action_Stop_Motors();
  Motor_Action_Stop_Spin_Blades();

  Serial.println(F("Funtion in Go To Charging Station"));
  Check_Mower_Status();
  

  if (Mower_Docked == 1) {
    Serial.println(F(""));
    Serial.println(F("Mower is already docked"));
    Serial.println(F(""));
  }
  // Only activate this code if the mower is not already docked.
  if (Mower_Docked == 0) {

      // Update the TFT Screen with information
      if ((TFT_Screen_Menu == 1) && (Mower_Parked == 1)) {
          Mower_Docked          = 0;
          Mower_Parked          = 0;
          Mower_Track_To_Charge = 1;
          Tracking_Wire         = 1;
          Serial.println(F("Updating TFT with Exit Dock Information"));
          Calculate_TFT_Robot_Status_Value();
          Send_Mower_Docked_Data();         // Send docked numbers to break out of the cycle and change it to mower exiting dock mode.
          delay(4000);  
          }  

      // Update the TFT Screen with information
      if ((TFT_Screen_Menu == 1) && (Mower_Running == 1)) {
          Mower_Docked          = 0;
          Mower_Parked          = 0;
          Mower_Track_To_Charge = 1;
          Tracking_Wire         = 1;
          Serial.println(F("Updating TFT with Exit Dock Information"));
          Calculate_TFT_Robot_Status_Value();
          if (Robot_Type == 1) Send_Mower_Running_Data();
//         if (Robot_Type == 2) Send_Aerator_Running_Data();
          delay(4000);  
          } 

    
      // Reset all variables to the correct status
      Mower_Docked          = 0;
      Mower_Parked          = 0;
      Mower_Running         = 0;
      Mower_Parked_Low_Batt = 0;
      Mower_Track_To_Charge = 1;
      Tracking_Wire         = 1;
      Mower_Track_To_Exit   = 0;
      Mower_Error           = 0;
      Loop_Cycle_Mowing     = 0;
      Manual_Mode           = 0;
      No_Wire_Found_Fwd     = 0;
      No_Wire_Found_Bck     = 0;
      Manage_Alarms();                                              // Switches on or off the Alarms depending on the setup

#if defined(WDT)
      MEGA_WDT(); // disable WDT because of a lot of delay() used or reset WDT after each delay() ???
#endif // -(WDT)-
    
  
   // if the stop/pause button is pressed the whole process is stopped.
   // Process is halted when the docked = 1 is given by detecting a charge
   while ( (Mower_Parked == 0) && (Mower_Docked == 0)) {    
      Get_WIFI_Commands();
      delay(2000);
      Turn_On_Relay();
      delay(500);
      
      
      // STEP 1
      // Turns the Mower in the Home Compass Direction and send the info to the TFT and WIFI
      // Function is cancelled if a Mower Parked = 1
      // This is created when the Pause Button is pressed.
      if (Compass_Activate == 1)  {
        Get_WIFI_Commands();
        if (TFT_Screen_Menu == 1) {                                     // send Commands to the TFT;
          Turn_To_Home = 1;
          Find_Wire_Track = 0;
          Go_To_Charging_Station = 0;
          Send_Mower_Tracking_Data();
          }  
      if (Mower_Parked == 0) Compass_Turn_Mower_To_Home_Direction();
      }
      
      
      // STEP 2
      // Find the Wire and send the info to the TFT and WIFI  
      // Function is cancelled if a Mower Parked = 1
      // This is created when the Pause Button is pressed.
      if (Perimeter_Wire_Enabled == 1) {  
        Get_WIFI_Commands();
        if (TFT_Screen_Menu == 1) {
          Turn_To_Home = 0;
          Find_Wire_Track = 1;
          Go_To_Charging_Station = 0;
          Send_Mower_Tracking_Data();
          }
      if ((Mower_Parked == 0) && (Fake_All_Settings == 0)) Manouver_Find_Wire_Track();
      }
        
      
      
      // STEP 3
      // Track the Wire and send the info to the TFT and WIFI
      if ((Perimeter_Wire_Enabled == 1) && (No_Wire_Found_Fwd == 0)) {   
        Get_WIFI_Commands();
        if (TFT_Screen_Menu == 1) {
              Turn_To_Home = 0;
              Find_Wire_Track = 0;
              Go_To_Charging_Station = 1;
              Send_Mower_Tracking_Data();
              }
        if (Mower_Parked == 0) Track_Perimeter_Wire_To_Dock();
        }
      
      
      // REPEAT
      // Restarts the process if no wire is found and no pause has been pressed (Parked),
      // and if the mower has not achievd a Docked Status
      if ((Mower_Docked == 0) && (Mower_Parked == 0)) {
          Get_WIFI_Commands();
          if (TFT_Screen_Menu == 1) Send_Mower_Tracking_Data();
          if (No_Wire_Found_Fwd == 1) Manouver_Go_To_Charging_Station();    // Restart.
          Get_WIFI_Commands();
          }
    
      }
   }
}  

void Manouver_Exit_To_Zone_X() {
   // Zone 1 or Zone 2 and the Wire itterations are set on the Membrane Buttons.
   // These values are then crried into the following functions.     
   
   // Activated Auto Wire_ON
   Mower_Running  = 1;                                          // Set status to 2 for NodeMCU Wire ON Function (with Filter)
   Mower_Docked   = 0;    
   Transmit_All_To_NODEMCU();
   
   Mower_Running  = 0;
   Mower_Docked   = 1;  
   Mower_Error_Value = 0;
   delay(400);   
   Turn_On_Relay();
   delay(1000);
   Serial.println(F("Sending Tracking Data in Exit To Zone X"));
   delay(200);   

   Serial.println("In Exit Function");
   Check_Mower_Status();
   
   if ((Mower_Parked == 0) || (Mower_Docked == 1)) Manouver_Mower_Exit_Dock();          // Carry out the Exit Dock Manouver   
   if (Exiting_Dock == 1) Manouver_Exit_From_Docking_Station();                         // Move the Mower into position backing out of the docking station
        
      Wire_Off = 0;                                             // Reset Wire OFF Count before checiking wire status
      Run_Initial_Boundary_Wire_Test();                         // Run the intial wire test again to give time for the auto wire to activate and be detected.
 
      Serial.println("After Wire Test");
      Check_Mower_Status();
         
      if ((Wire_Detected == 0) && (Perimeter_Wire_Enabled == 1)) {
                Serial.println(F(""));
                Serial.println("Perimeter Wire Still not detected");
                Robot_Status_Value = 4;         // Sends the mower status value to the TFT Screen 4 = Error Status.
                Mower_Error_Value = 1;          // Describes the error 1 = No Wire, 
                Serial.println(F("Sending Tracking Data in Wire detect = 0"));
                Send_Mower_Tracking_Data();
                delay(5000);
                Manouver_Park_The_Mower();
                }
    
      if ((Wire_Detected == 1) && (Perimeter_Wire_Enabled == 1 ) && (Mower_Error_Value == 0) ) {
          Serial.println(F("Boundary Wire - Detected - Lets GO!"));
          }


     while ( (Mower_Parked == 0) && (Mower_Running == 0) ) {     
            
          Get_WIFI_Commands();
          if ((Perimeter_Wire_Enabled == 1) && (Fake_All_Settings == 0)) Manouver_Find_Wire_Track();                                   // Located the boundary wire           
          Get_WIFI_Commands();
          if (Mower_Parked == 0) Track_Wire_From_Dock_to_Zone_X();
          Get_WIFI_Commands();            
          if (Mower_Parked == 0) Special_Move_Into_Garden_Zone_X();
          Get_WIFI_Commands();
          if (Mower_Parked == 0) Manouver_Start_Mower();
          Get_WIFI_Commands();   
          }
        
      if (Perimeter_Wire_Enabled == 0){
           Serial.println(F(""));
           Serial.println("Perimeter Wire not activated in settings");
           //Manouver_Park_The_Mower();
           Manouver_Start_Mower();
           }  
    }
 


// Function to re-find the wire if the mower looses the wire while mowing
// 3 outside the wires ativates this function. Sonar and wire function is then used to re-find the wire.
void Manouver_Outside_Wire_ReFind_Function(){
    Motor_Action_Stop_Spin_Blades(); 
    
    #if defined(LCD_KEYPAD)
    lcd.clear();
    lcd.print("Trying to find");
    lcd.setCursor(0,1);
    lcd.print("Wire Again...");
    #endif
    
    ADCMan.run();
    UpdateWireSensor();
    delay(20);
    PrintBoundaryWireStatus();
    ADCMan.run();
    UpdateWireSensor();
    delay(20);
    PrintBoundaryWireStatus();
  while (inside == false) {                                              // If the mower is outside the wire then run the following code.
     ADCMan.run();
     UpdateWireSensor();
     PrintBoundaryWireStatus();
     //Check_Wire_In_Out();
     delay(500);
     distance_blockage = PingSonarX(trigPin1, echoPin1, 1, 1, 1, 4, 0);
     delay(500);
     Serial.print(F("Distance measured from sonar :"));
     Serial.println(distance_blockage);
     
     // if the sonar is measuring an opening as the distance is greater than 300cm then move forward in that direction.
     if (distance_blockage > 400) {
      while ( (inside == false) && (distance_blockage > 400) ){
        SetPins_ToGoForwards();
        Motor_Action_Go_Full_Speed();
        delay(500);
        ADCMan.run();
        UpdateWireSensor();
        PrintBoundaryWireStatus();
        distance_blockage = PingSonarX(trigPin1, echoPin1, 1, 1, 1, 4, 0); 
        delay(10);
        //Check_Wire_In_Out();
      }
     Motor_Action_Stop_Motors();
     ADCMan.run();
     UpdateWireSensor();
     PrintBoundaryWireStatus();
     }

     // if the sonar is measuring something less than 300cm then turn to the left and measure again
     // keep turning 'while function' until a path is open.
     while (distance_blockage < 300) {
       SetPins_ToTurnLeft();
       delay(200);
       Motor_Action_Turn_Speed();                                                             
       delay(100); 
       distance_blockage = PingSonarX(trigPin1, echoPin1, 1, 1, 1, 4, 0); 
       delay(10);
     }
     Motor_Action_Stop_Motors();
     ADCMan.run();
     UpdateWireSensor();
     PrintBoundaryWireStatus();
  }

   Serial.println("Mower is now back inside the wire......?");

   #if defined(LCD_KEYPAD)
   lcd.clear();
   lcd.print("Mower now");
   lcd.setCursor(0,1);
   lcd.print("Inside Wire?");
   #endif
   
   Outside_Wire_Count = 0;
   //FindWireTrack();
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Motor_Action_Mower
void Motor_Action_Go_Mowing_Speed() {

  // No Mag speed adjustment active just go full speed
  if (MAG_Speed_Adjustment == 0) Motor_Action_Go_Full_Speed();

  // Adjust wheel speed according to the MAG level
  if (MAG_Speed_Adjustment == 1) {
    if (MAG_Now >= Slow_Speed_MAG) {
      Motor_Action_Go_Full_Speed();
    }

    if (MAG_Now < Slow_Speed_MAG)  {
      Motor_Action_Go_Slow_Speed();
    }
  }
}

void Motor_Action_Go_Full_Speed()     {

  // Full straighgt speed no motr speed ramp up.
  if (Ramp_Motor_ON == 0) {
#if defined(ROBOT_MOWER)
    if (Wheels_Activate) {
    analogWrite(ENAPin, PWM_MaxSpeed_RH);                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
    analogWrite(ENBPin, PWM_MaxSpeed_LH);                       // Anaolgwirte sends PWM signals Speed = 0-255  (255 is max speed)
    }
#endif


#if defined(ROBOT_AERATOR)
    analogWrite(ENAPin, PWM_MaxSpeed_RH);                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
    analogWrite(ENBPin, PWM_MaxSpeed_LH);
    analogWrite(ENCPin, PWM_MaxSpeed_RH);
    analogWrite(ENDPin, PWM_MaxSpeed_LH);
#endif

    Wheel_Status_Value = 1;
    Serial.print(F("Wheel:FULL|"));
  }

  // If Ramp up is achieved just go full speed
  if ((Ramp_Motor_ON == 1) && (Full_Speed_Achieved == 1)) {

#if defined(ROBOT_MOWER)
    if (Wheels_Activate) {
    analogWrite(ENAPin, PWM_MaxSpeed_RH);                              // Ramp up the motor speed
    analogWrite(ENBPin, PWM_MaxSpeed_LH);                              // Ramp up the motor speed
    }
#endif

#if defined(ROBOT_AERATOR)
    analogWrite(ENAPin, PWM_MaxSpeed_RH);                              // Ramp up the motor speed
    analogWrite(ENBPin, PWM_MaxSpeed_LH);
    analogWrite(ENCPin, PWM_MaxSpeed_RH);
    analogWrite(ENDPin, PWM_MaxSpeed_LH);
#endif

    Wheel_Status_Value = 2;
    Serial.print(F("Wheel:R-FULL|"));
  }

  // Ramp motor option
  if ((Ramp_Motor_ON == 1) && (Full_Speed_Achieved == 0)) {

    Serial.print(F("|WRamp: "));
    int Motor_Step = 150;

    while (Motor_Step > 1) {

#if defined(ROBOT_MOWER)
    if (Wheels_Activate) {
      analogWrite(ENAPin, (PWM_MaxSpeed_RH - Motor_Step));                       // Ramp up the motor speed
      analogWrite(ENBPin, (PWM_MaxSpeed_LH - Motor_Step));                       // Ramp up the motor speed
    }
#endif

#if defined(ROBOT_AERATOR)
      analogWrite(ENAPin, (PWM_MaxSpeed_RH - Motor_Step));                       // Ramp up the motor speed
      analogWrite(ENBPin, (PWM_MaxSpeed_LH - Motor_Step));                       // Ramp up the motor speed
      analogWrite(ENCPin, (PWM_MaxSpeed_RH  - Motor_Step));
      analogWrite(ENDPin, (PWM_MaxSpeed_LH  - Motor_Step));
#endif

      delay(60);
      //Serial.print(F("Step: "));
      //Serial.println(Motor_Step);
      Motor_Step = Motor_Step - 10;
    }

    Wheel_Status_Value = 3;
    Serial.print(F("Wheel:S-FULL|"));
    Full_Speed_Achieved = 1;
  }


}

void Motor_Action_Go_Slow_Speed()     {
#if defined(ROBOT_MOWER)
  if (Wheels_Activate) {
  analogWrite(ENAPin,  PWM_Slow_Speed_RH);                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
  analogWrite(ENBPin,  PWM_Slow_Speed_LH);                       // Anaolgwirte sends PWM signals Speed = 0-255  (255 is max speed)
  }
#endif


#if defined(ROBOT_AERATOR)
  analogWrite(ENAPin,  PWM_Slow_Speed_RH);                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
  analogWrite(ENBPin,  PWM_Slow_Speed_LH);                       // Anaolgwirte sends PWM signals Speed = 0-255  (255 is max speed)
  analogWrite(ENCPin,  PWM_Slow_Speed_RH);                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
  analogWrite(ENDPin,  PWM_Slow_Speed_LH);
#endif

  Wheel_Status_Value = 4;
  Serial.print(F("Wheel:SLOW|"));
}

void Motor_Action_GoFullSpeed_Out_Garage()     {
  //Speeds can be changed to give the mower a slight curve when exiting the Garage.

  PWM_MaxSpeed_LH = PWM_MaxSpeed_LH + 20;
  if (PWM_MaxSpeed_LH > 255)  PWM_MaxSpeed_LH = 255;
  if (PWM_MaxSpeed_RH > 255)  PWM_MaxSpeed_RH = 255;

#if defined(ROBOT_MOWER)
  if (Wheels_Activate) {
  analogWrite(ENAPin, PWM_MaxSpeed_RH);                                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
  analogWrite(ENBPin, PWM_MaxSpeed_LH);
  }
#endif

#if defined(ROBOT_AERATOR)
  analogWrite(ENAPin, PWM_MaxSpeed_RH);                                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
  analogWrite(ENBPin, PWM_MaxSpeed_LH);
  analogWrite(ENCPin, PWM_MaxSpeed_RH);                       // Speed = 0-255  (255 is max speed). Speed is set in the settings
  analogWrite(ENDPin, PWM_MaxSpeed_LH);
#endif

  Wheel_Status_Value = 1;
  Serial.print(F("Wheel:FULL|"));
}


void SetPins_ToGoForwards()     {                                 // Motor Bridge pins are set for both motors to move forwards.

#if defined(ROBOT_MOWER)
  digitalWrite(IN1Pin, LOW);                                      // Motor Birdge pins are set to high or low to set the direction of movement
  digitalWrite(IN2Pin, HIGH);
  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, HIGH);
#endif

#if defined(ROBOT_AERATOR)
  digitalWrite(IN1Pin, LOW);                                      //
  digitalWrite(IN2Pin, HIGH);

  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, HIGH);

  digitalWrite(IN5Pin, LOW);
  digitalWrite(IN6Pin, HIGH);

  digitalWrite(IN7Pin, LOW);
  digitalWrite(IN8Pin, HIGH);
#endif

  Wheel_Status_Value = 5;
  Serial.print(F("|Wheel:For|"));
}


void SetPins_ToGoBackwards()      {                               // Motor Bridge pins are set for both motors to move Backwards
#if defined(ROBOT_MOWER)
  digitalWrite(IN1Pin, HIGH);                                     // Motor 1
  digitalWrite(IN2Pin, LOW);
  digitalWrite(IN3Pin, HIGH);                                     // Motor 2
  digitalWrite(IN4Pin, LOW);
#endif

#if defined(ROBOT_AERATOR)
  digitalWrite(IN1Pin, HIGH);                                     // Motor 1
  digitalWrite(IN2Pin, LOW);

  digitalWrite(IN3Pin, HIGH);                                     // Motor 2
  digitalWrite(IN4Pin, LOW);

  digitalWrite(IN5Pin, HIGH);
  digitalWrite(IN6Pin, LOW);

  digitalWrite(IN7Pin, HIGH);
  digitalWrite(IN8Pin, LOW);
#endif

  Wheel_Status_Value = 6;
  Serial.print(F("|Wheel:Rev|"));
#if not defined(NODELAY_BACKWARD)
  delay(20);
#endif
}


void Motor_Action_Stop_Motors()  {                                     // Motor Bridge pins are set for both motors to stop
#if defined(ROBOT_MOWER)
  digitalWrite(ENAPin, 0);
  digitalWrite(IN1Pin, LOW);                                    //Motor 1
  digitalWrite(IN2Pin, LOW);
  digitalWrite(ENBPin, 0);                                      //Motor 2
  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, LOW);
#endif

#if defined(ROBOT_AERATOR)
  digitalWrite(ENAPin, 0);
  digitalWrite(IN1Pin, LOW);                                    //Motor 1
  digitalWrite(IN2Pin, LOW);
  digitalWrite(ENBPin, 0);                                      //Motor 2
  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, LOW);
  digitalWrite(ENCPin, 0);
  digitalWrite(IN5Pin, LOW);
  digitalWrite(IN6Pin, LOW);
  digitalWrite(ENDPin, 0);
  digitalWrite(IN7Pin, LOW);
  digitalWrite(IN8Pin, LOW);
#endif

  Wheel_Status_Value = 7;
  Serial.print(F("Wheel:0FF|"));

  Full_Speed_Achieved = 0;
}


void SetPins_ToTurnLeft()       {                              // Pins are set so that Motors drive in opposite directions
#if defined(ROBOT_MOWER)
  digitalWrite(IN1Pin, LOW);                                   // Motor 1
  digitalWrite(IN2Pin, HIGH);
  digitalWrite(IN3Pin, HIGH);                                  // Motor 2
  digitalWrite(IN4Pin, LOW);
#endif

#if defined(ROBOT_AERATOR)
  digitalWrite(IN1Pin, HIGH);                                   // Motor 1
  digitalWrite(IN2Pin, LOW);

  digitalWrite(IN3Pin, LOW);                                  // Motor 2
  digitalWrite(IN4Pin, HIGH);

  digitalWrite(IN5Pin, LOW);
  digitalWrite(IN6Pin, HIGH);

  digitalWrite(IN7Pin, HIGH);
  digitalWrite(IN8Pin, LOW);
#endif

  Wheel_Status_Value = 8;
  Serial.print(F("Wheel:TL_|"));
}


void SetPins_ToTurnRight() {                                    // Pins are set so that Motors drive in opposite directions
#if defined(ROBOT_MOWER)
  digitalWrite(IN1Pin, HIGH);                                   // Motor 1
  digitalWrite(IN2Pin, LOW);
  digitalWrite(IN3Pin, LOW);                                    //Motor 2
  digitalWrite(IN4Pin, HIGH);
#endif

#if defined(ROBOT_AERATOR)
  digitalWrite(IN1Pin, LOW);                                   // Motor 1
  digitalWrite(IN2Pin, HIGH);

  digitalWrite(IN3Pin, HIGH);                                    //Motor 2
  digitalWrite(IN4Pin, LOW);

  digitalWrite(IN5Pin, HIGH);
  digitalWrite(IN6Pin, LOW);

  digitalWrite(IN7Pin, LOW);
  digitalWrite(IN8Pin, HIGH);
#endif

  Wheel_Status_Value = 9;
  Serial.print(F("Wheel:TR_|"));
}


// USed to turn the mower at a set speed.
void Motor_Action_Turn_Speed() {
#if defined(ROBOT_MOWER)
  if (Wheels_Activate) {
  analogWrite(ENAPin, (PWM_MaxSpeed_RH - Turn_Adjust) );                                  // Change the 0 value to 10 or 20 to recuce the speed
  analogWrite(ENBPin, (PWM_MaxSpeed_LH - Turn_Adjust) );                                  // Change the 0 value to 10 or 20 to recuce the speed
  }
#endif

#if defined(ROBOT_AERATOR)
  analogWrite(ENAPin, (PWM_MaxSpeed_RH - Turn_Adjust) );                                  // Change the 0 value to 10 or 20 to recuce the speed
  analogWrite(ENBPin, (PWM_MaxSpeed_LH - Turn_Adjust) );                                  // Change the 0 value to 10 or 20 to recuce the speed
  analogWrite(ENCPin, (PWM_MaxSpeed_LH - Turn_Adjust) );                                  // Change the 0 value to 10 or 20 to recuce the speed
  analogWrite(ENDPin, (PWM_MaxSpeed_RH - Turn_Adjust) );                                  // Change the 0 value to 10 or 20 to recuce the speed
#endif
}


// Turns the mowing blades on
void Motor_Action_Spin_Blades()  {

  if (Robot_Type == 1) {
    if (Cutting_Blades_Activate == 1) {                                       // Blades are turn ON in settings and will spin!

  #if defined(ROBOT_MOWER)
    digitalWrite(Relay_Blades_Brake_Resistor, LOW);
    if (!Blade_flagRun) {                // added delay only for first start up of blades because of spikes when relay switched on - reason of stop button interrupt activation
      analogWrite(RPWM, 0);
      delay(100);
      Blade_flagRun = true;
    }
  #endif // -(ROBOT_MOWER)-

      digitalWrite(R_EN, HIGH); // NOT Inverted
//      digitalWrite(L_EN, HIGH); // NOT Inverted
      analogWrite(RPWM, PWM_Blade_Speed);
      Serial.print(F("Blades:ON_|"));
    }

    if (Cutting_Blades_Activate == 0) {                                     // Blades are turn off in settings and will not spin!
      void StopSpinBlades();

    }
  }
}


void Motor_Action_Stop_Spin_Blades()  {

  if (Robot_Type == 1) {
    digitalWrite(R_EN, LOW); // NOT Inverted
//    digitalWrite(L_EN, LOW); // NOT Inverted
  #if defined(ROBOT_MOWER)
    digitalWrite(Relay_Blades_Brake_Resistor, HIGH);
    Blade_flagRun = false;
  #endif // -(ROBOT_MOWER)-
    Serial.print(F("Blades:0FF|"));
  }
}


//Steers the Mower depending on the PID input from the Algorythm
void Motor_Action_Dynamic_PWM_Steering() {
#if defined(ROBOT_MOWER)
  if (Wheels_Activate) {
  analogWrite(ENAPin, PWM_Right);                             // ENA low = Right Swerve   ENB low = Left Swerve
  analogWrite(ENBPin, PWM_Left);
  }
#endif

#if defined(ROBOT_AERATOR)
  analogWrite(ENAPin, PWM_Right);                             // ENA low = Right Swerve   ENB low = Left Swerve
  analogWrite(ENBPin, PWM_Left);
  analogWrite(ENCPin, PWM_Right);                              // ENA low = Right Swerve   ENB low = Left Swerve
  analogWrite(ENDPin, PWM_Left);
#endif

  Serial.print(F("PWM_L:"));
  Serial.print(PWM_Left);
  Serial.print(F("|"));
  Serial.print(F("PWM_R:"));
  Serial.print(PWM_Right);
  Serial.print(F("|"));

}


void Stop_Button_Pressed() {
  #if defined(STOP_BTN)
  if  (Mower_Running == 1) {
    Motor_Action_Stop_Spin_Blades();
    Motor_Action_Stop_Motors();
    Stop_Button_Activated = true;
  }
  #endif // -(STOP_BTN)-
}


void Stop_Button_Action() {
  #if defined(STOP_BTN)
  // All communication commands should be out of function called by interrupt
  if (Stop_Button_Activated) {
    Manouver_Park_The_Mower();
    Print_LCD_Stop_Btn();
    Serial.println(F(""));
    Serial.println(F("!!! STOP button pressed !!!"));
    Stop_Button_Activated = false;
  }
  #endif // -(STOP_BTN)-
}


void Set_Mower_Forwards_Right() {
  digitalWrite(IN1Pin, LOW);                                      // Motor Birdge pins are set to high or low to set the direction of movement
  digitalWrite(IN2Pin, HIGH);
  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, LOW);
  }


void Set_Mower_Forwards_Left() {
  digitalWrite(IN1Pin, LOW);                                      // Motor Birdge pins are set to high or low to set the direction of movement
  digitalWrite(IN2Pin, LOW);
  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, HIGH);
  }

void Set_Mower_Backwards_Right() {
  digitalWrite(IN1Pin, HIGH);                                      // Motor Birdge pins are set to high or low to set the direction of movement
  digitalWrite(IN2Pin, LOW);
  digitalWrite(IN3Pin, LOW);
  digitalWrite(IN4Pin, LOW);
  }


void Set_Mower_Backwards_Left() {
  digitalWrite(IN1Pin, LOW);                                      // Motor Birdge pins are set to high or low to set the direction of movement
  digitalWrite(IN2Pin, LOW);
  digitalWrite(IN3Pin, HIGH);
  digitalWrite(IN4Pin, LOW);
  }


void Set_Mecanum_Forwards_Left_Front() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN7Pin, LOW);                                      //
  digitalWrite(IN8Pin, HIGH);
  #endif
  }

void Set_Mecanum_Backwards_Left_Front() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN7Pin, HIGH);                                      //
  digitalWrite(IN8Pin, LOW);
  #endif
  }

void Set_Mecanum_Forwards_Right_Front() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN5Pin, LOW);                                      //
  digitalWrite(IN6Pin, HIGH);
  #endif
  }

void Set_Mecanum_Backwards_Right_Front() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN5Pin, HIGH);                                      //
  digitalWrite(IN6Pin, LOW);
  #endif
  }


void Set_Mecanum_Forwards_Left_Rear() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN1Pin, LOW);                                      //
  digitalWrite(IN2Pin, HIGH);
  #endif
  }

void Set_Mecanum_Backwards_Left_Rear() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN1Pin, HIGH);                                      //
  digitalWrite(IN2Pin, LOW);
  #endif
  }

void Set_Mecanum_Forwards_Right_Rear() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN3Pin, LOW);                                      //
  digitalWrite(IN4Pin, HIGH);
  #endif
  }

void Set_Mecanum_Backwards_Right_Rear() {
  #if defined(ROBOT_AERATOR)
  digitalWrite(IN3Pin, HIGH);                                      //
  digitalWrite(IN4Pin, LOW);
  #endif
  }

void Full_Speed_Mecanum_Left_Rear() {
  #if defined(ROBOT_AERATOR)
  analogWrite(ENAPin, PWM_MaxSpeed_RH); 
  #endif
  }

void Full_Speed_Mecanum_Right_Rear() {
  #if defined(ROBOT_AERATOR)
  analogWrite(ENBPin, PWM_MaxSpeed_RH); 
  #endif
  }

void Full_Speed_Mecanum_Left_Front() {
  #if defined(ROBOT_AERATOR)
  analogWrite(ENDPin, PWM_MaxSpeed_RH); 
  #endif
  }

void Full_Speed_Mecanum_Right_Front() {
  #if defined(ROBOT_AERATOR)
  analogWrite(ENCPin, PWM_MaxSpeed_RH); 
  #endif
  }

void Mecanum_Side_Movement_Left() {
  #if defined(ROBOT_AERATOR)
  Set_Mecanum_Backwards_Left_Front();
  Set_Mecanum_Forwards_Right_Front();
  Set_Mecanum_Forwards_Left_Rear();
  Set_Mecanum_Backwards_Right_Rear();
  #endif
  }


void Mecanum_Side_Movement_Right() {
  #if defined(ROBOT_AERATOR)
  Set_Mecanum_Forwards_Left_Front();
  Set_Mecanum_Backwards_Right_Front();
  Set_Mecanum_Backwards_Left_Rear();
  Set_Mecanum_Forwards_Right_Rear();
  #endif
  }

void Mecanum_Diagonal_Movement_Left() {
  #if defined(ROBOT_AERATOR)
  Motor_Action_Stop_Motors();
  Set_Mecanum_Forwards_Right_Front();
  Set_Mecanum_Forwards_Left_Rear();
  #endif
  }


void Mecanum_Diagonal_Movement_Right() {
  #if defined(ROBOT_AERATOR)
  Motor_Action_Stop_Motors();
  Set_Mecanum_Forwards_Left_Front();
  Set_Mecanum_Forwards_Right_Rear();
  #endif
  }

void Mecanum_Rear_Steer_Right() {
  #if defined(ROBOT_AERATOR)
  Motor_Action_Stop_Motors();
  Set_Mecanum_Forwards_Left_Front();
  Set_Mecanum_Backwards_Right_Front();
  #endif
  }


void Mecanum_Rear_Steer_Left() {
  #if defined(ROBOT_AERATOR)
  Motor_Action_Stop_Motors();
  Set_Mecanum_Backwards_Left_Front();
  Set_Mecanum_Forwards_Right_Front();
  #endif
  }
  

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Pattern_Mow
void Pattern_Mow_Spirals() {
      //analogWrite(ENAPin, PWM_MaxSpeed_RH - (100 - Loop_Cycle_Mowing));             // 255 - 200 = 55 up to 255 again.  (255 is max speed). Speed is set in the settings                        
      
      int PWM_1 = 70;         // diameter 0.4m    
      int PWM_2 = 95;         //                    
      int PWM_3 = 110;
      int PWM_4 = 120;
      int PWM_5 = 127;        //
      int PWM_6 = 135;
      int PWM_7 = 145;        // (diameter 3-4m)

      int Spiral_Step = 200;

      int End_Linking                     = 200;      // straight line section between spirals  200 is approx 3-4m
      int End_Spiral_1   = End_Linking    + Spiral_Step;      // sets the number of cycles within Spiral 1
      int End_Spiral_2   = End_Spiral_1   + (Spiral_Step * 2);
      int End_Spiral_3   = End_Spiral_2   + (Spiral_Step * 4);
      int End_Spiral_4   = End_Spiral_3   + (Spiral_Step * 6);      // Sets the number of cycles within Spiral 4
      int End_Spiral_5   = End_Spiral_4   + (Spiral_Step * 8);
      int End_Spiral_6   = End_Spiral_5   + (Spiral_Step * 10);
      int End_Spiral_7   = End_Spiral_6   + (Spiral_Step * 12);     // sets the number of cylces within Spiral 7

      // Total number of cycels in all sections should not exceed the Max_Cycles_Spirals 
      // otherwise the mower will stop before the spiral pattern is completed



      // Spiral_Mow = 1 is a Right Hand Sprial
      // Spiral_Mow = 2 is a left hand Spiral
      // Sprial:Mow = 3 no sprial and a normal straight line move

      Get_Compass_Reading();  // keeps the compass active
      
      
      if (Spiral_Mow == 1) analogWrite(ENBPin, PWM_MaxSpeed_LH);  
      if (Spiral_Mow == 2) analogWrite(ENAPin, PWM_MaxSpeed_RH);
      if (Spiral_Mow == 3) {
        analogWrite(ENAPin, PWM_MaxSpeed_RH);
        analogWrite(ENBPin, PWM_MaxSpeed_LH);
      }

      if (Spiral_Mow < 3) {
      

      //Wheel_Status_Value = 10;
      Serial.print(F("Wheel:CIRCLE|"));

      #if defined(LCD_KEYPAD)
      lcd.setCursor(9,1);
      if (Spiral_Mow == 1) lcd.print("R");
      if (Spiral_Mow == 2) lcd.print("L");
      if (Spiral_Mow == 3) lcd.print("|");
      #endif
      

      if (Loop_Cycle_Mowing < End_Linking) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_MaxSpeed_RH);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_MaxSpeed_LH);

        #if defined(LCD_KEYPAD)
        lcd.print("|");
        #endif
        
        }

      //Inside Spiral
      if ((Loop_Cycle_Mowing >= End_Linking) && (Loop_Cycle_Mowing < End_Spiral_1)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_1);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_1);

        #if defined(LCD_KEYPAD)
        lcd.print("1");
        #endif
        }

      // Sprial Ring 2
      if ((Loop_Cycle_Mowing >= End_Spiral_1) && (Loop_Cycle_Mowing < End_Spiral_2)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_2);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_2);

        #if defined(LCD_KEYPAD)
        lcd.print("2");        
        #endif
        }

      // Sprial Ring 3
      if ((Loop_Cycle_Mowing >= End_Spiral_2) && (Loop_Cycle_Mowing < End_Spiral_3)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_3);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_3);

        #if defined(LCD_KEYPAD)
        lcd.print("3");        
        #endif
         
        }

      // Sprial Ring 4
      if ((Loop_Cycle_Mowing >= End_Spiral_3) && (Loop_Cycle_Mowing < End_Spiral_4)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_4);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_4);

        #if defined(LCD_KEYPAD)
        lcd.print("4");        
        #endif 
        }
      
      // Sprial Ring 5
      if ((Loop_Cycle_Mowing >= End_Spiral_4) && (Loop_Cycle_Mowing < End_Spiral_5)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_5);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_5);
        
        #if defined(LCD_KEYPAD)
        lcd.print("5");        
        #endif
         
        }

      // Sprial Ring 6
      if ((Loop_Cycle_Mowing >= End_Spiral_5) && (Loop_Cycle_Mowing < End_Spiral_6)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_6);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_6);

        #if defined(LCD_KEYPAD)
        lcd.print("6");  
        #endif       
        }

      // Sprial Ring 7
      if ((Loop_Cycle_Mowing >= End_Spiral_6) && (Loop_Cycle_Mowing < End_Spiral_7)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_7);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_7);
        
        #if defined(LCD_KEYPAD)
        lcd.print("7"); 
        #endif
                
        }
        
      // Mower moves away to new position
      if ((Loop_Cycle_Mowing >= End_Spiral_7) && (Loop_Cycle_Mowing < Max_Cycles_Spirals)) {
        if (Spiral_Mow == 1) analogWrite(ENAPin, PWM_MaxSpeed_RH);
        if (Spiral_Mow == 2) analogWrite(ENBPin, PWM_MaxSpeed_LH);
        
        #if defined(LCD_KEYPAD)
        lcd.print("X");     
        #endif
        
        }
      }

}



void Pattern_Mow_Parallel() {
  

  int Parallel_Compass_Assist = 1;         // compass tries to keep the mowed line straight
  int Turning_Compass_Assist  = 0;         // compass assists to turn the mower 90° at each corner - DOESNT WORK YET!!!
       


// if the line length hasnt been achieved then keep on going straight.
  if (Loop_Cycle_Mowing < Line_Length_Cycles)  {
    SetPins_ToGoForwards();

    // USe compass assist to keep the mower in a straight line hopefully parallel to the last 
    if (Parallel_Compass_Assist == 1) {  
        
     if (Loop_Cycle_Mowing < 3) {
        if ((Compass_Heading_Hold_Enabled == 1) && (Compass_Activate == 1))  {         // use the heading hold funtion for Parallel Mowing
           Get_Compass_Reading();                                                      // Gets the latest compass reading
           Heading_Lock = Compass_Heading_Degrees;                                     // saves this compass reading to the heading lock
           Compass_Heading_Locked = 1;                                                 // Turns on the heading lock feature
           Compass_Last = Heading_Lock;
           Motor_Action_Go_Mowing_Speed();
           }
        }
    
     if (Loop_Cycle_Mowing >= 3) {
        if ((Compass_Heading_Hold_Enabled == 1) && (Compass_Activate == 1)) {            // if the Mower is tracking using the compass steer here
          if ( (Loop_Cycle_Mowing % 2) == 0 ) {
          Get_Compass_Reading(); 
          Calculate_Compass_Wheel_Compensation();
          Motor_Action_Dynamic_PWM_Steering();              // Removes the full speed function if the mower is trying to hold to the compass heading.
          Print_LCD_Parallel_Mowing();
          Serial.print(F("C-Lock:ON_"));
          Serial.print("|");
          }
        }
     }
    }
     
    // No use of compass assist
    if (Parallel_Compass_Assist == 0) {
        Motor_Action_Go_Mowing_Speed();
        Serial.println("Compass not activated in the settings");
       }
    }

// if the line length has been achieved then turn around.
  if (Loop_Cycle_Mowing > Line_Length_Cycles) {
    Motor_Action_Stop_Motors();
    delay(300);
    
    // Turn 90°
    if (Leg == 1) {
      SetPins_ToTurnLeft();
      Motor_Action_Go_Full_Speed();
      delay(Turn_90_Delay_LH);
      Get_Compass_Reading(); 
    }
    if (Leg == 2) {
      SetPins_ToTurnRight();
      Motor_Action_Go_Full_Speed();
      delay(Turn_90_Delay_RH); 
      Get_Compass_Reading(); 
      }

    Motor_Action_Stop_Motors();
    delay(300);
    
    // Go to the next line
    SetPins_ToGoForwards();
    Motor_Action_Go_Full_Speed();
    delay(Move_to_next_line_delay);
    Get_Compass_Reading(); 
    Motor_Action_Stop_Motors();
    delay(300);
    
    // Turn Final 90° to face in the next parallel direction just using the timing given on the LH and RH wheels 
    // this will give an innaczrate turn but it runs stabkle
    if (Turning_Compass_Assist  == 0) {
      if (Leg == 1) {
        SetPins_ToTurnLeft();
        Motor_Action_Go_Full_Speed();
        delay(Turn_90_Delay_LH);
        Get_Compass_Reading(); 
        Motor_Action_Stop_Motors();
      }
      if (Leg == 2) {
        SetPins_ToTurnRight();
        Motor_Action_Go_Full_Speed();
        delay(Turn_90_Delay_RH); 
        Get_Compass_Reading(); 
        Motor_Action_Stop_Motors();
        }
      }

    //Adjusts the final direction of the mower using the compass turning feature
    if (Turning_Compass_Assist  == 1) {
      
    
      // Sets the target compass degrees to 180° from the last line
      Compass_Target = Heading_Lock + 180;
      if (Compass_Target > 360) (Compass_Target = Compass_Target - 360);

      
      if (Leg == 1) {
        SetPins_ToTurnLeft();
        Motor_Action_Go_Full_Speed();
        delay(Turn_90_Delay_LH);
        Motor_Action_Stop_Motors();
        Get_Compass_Reading(); 
        delay(500);
        #if defined(LCD_KEYPAD)
        lcd.clear();
        lcd.print("Turn Assist");
        lcd.setCursor(0,1);
        lcd.print("Target");
        lcd.print(Compass_Target);
        delay(1000);
        lcd.clear();
        #endif
        
        Turn_To_Compass_Heading();

        #if defined(LCD_KEYPAD)
        lcd.clear();
        #endif
        
      }
      if (Leg == 2) {
        SetPins_ToTurnRight();
        Motor_Action_Go_Full_Speed();
        delay(Turn_90_Delay_RH); 
        Motor_Action_Stop_Motors();
        Get_Compass_Reading(); 
        delay(500);
        #if defined(LCD_KEYPAD)
        lcd.clear();
        lcd.print("Turn Assist");
        lcd.setCursor(0,1);
        lcd.print("Target");
        lcd.print(Compass_Target);
        delay(1000);
        lcd.clear();
        #endif
        
        Turn_To_Compass_Heading();
        
        #if defined(LCD_KEYPAD)
        lcd.clear();
        #endif
        
        }
      }
    
    Motor_Action_Stop_Motors();
    delay(300);
    SetPins_ToGoForwards();


    Loop_Cycle_Mowing   = 0;          // Reset the Loop counter
    
    #if defined(LCD_KEYPAD)
    lcd.setCursor(13, 1);
    lcd.print("   ");
    #endif
    
    Leg = Leg + 1;                    // Advances the leg so the mower turns in the othe direction next.
    if (Leg > 2) Leg = 1;             // Keeps the leg variable to 1 or 2
     
    }
}



void Pattern_Mow_Wire() {

  // Find the wire (unless already found)
  if (Wire_Found == 0) {
    Manouver_Find_Wire_Track();
    Wire_Found = 1;
    }
  // Log the signal strength
  
  // Follow this strength for a number of cycles
  // Turn Left ir Right depending on the direction
  // Follow the wire in the other direction at the new signal strength
  // Turn Left ir Right depending on the direction
  // Follow the wire in the other direction at the new signal strength etc... etc...
  
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Pixhawk_COM
void Set_Mode_PIXHAWK() {
  Mower_PIXHAWK = 1;
}



void Check_PIXHAWK() {

  if (PIXHAWK_Armed == 1) Serial.print(F("PIXHAWK | - ARMED - "));
  if (PIXHAWK_Armed == 0) Serial.print(F("PIXHAWK | Dis-Armed "));

// PIXHAWK Flight Control Modes
// 0 = Manual, 1 = Acro, 3 = Steering, 4 = Hold, 5 = Loiter, 6 = Follow, 7 = Simple, 10 =  Auto
// 11 = RTL, 12 = Smart_RTL, 15 = Guided

  if (Custom_Mode == 0) Serial.print(F(" | Manual | "));
  if (Custom_Mode == 1) Serial.print(F(" | Acro | "));
  if (Custom_Mode == 3) Serial.print(F(" | Steering | "));
  if (Custom_Mode == 4) Serial.print(F(" | Hold | "));
  if (Custom_Mode == 5) Serial.print(F(" | Loiter | "));
  if (Custom_Mode == 6) Serial.print(F(" | Follow | "));
  if (Custom_Mode == 7) Serial.print(F(" | Simple | "));
  if (Custom_Mode == 10) Serial.print(F(" | Auto | "));
  if (Custom_Mode == 11) Serial.print(F(" | RTL | "));
  if (Custom_Mode == 12) Serial.print(F(" | Smart_RTL |"));
  if (Custom_Mode == 15) Serial.print(F(" | Guided |"));
        
  // MAVLink
  /* The default UART header for your MCU */ 
 

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(System_ID, Component_ID, &msg, Type, Autopilot_Type, System_Mode, Custom_Mode, System_State);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;

    Pixhawk_Serial.write(buf,len);


    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
    
    // Request streams from Pixhawk
    Serial.println("Streams requested!");
    Mav_Request_Data();
    num_hbs_pasados=0;
    }

  }

  // Check reception buffer
  comm_receive();
}



void Mav_Request_Data()  {
//Serial.println("COM_Request");

    
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial.write(buf,len);
    //Pixhawk_Serial.write(buf, len);  //???

  }
  
  // Request: PARAM_REQUEST_LIST. Only for full log recording
  /*
   * Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                   uint8_t target_system, uint8_t target_component)
   */
/*
  // Configure
  uint8_t system_id=2;
  uint8_t component_id=200;
  // mavlink_message_t* msg;
  uint8_t target_system=1;
  uint8_t target_component=0;

  // Pack
  mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
    target_system, target_component);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send
#ifdef SOFT_SERIAL_DEBUGGING
    Pixhawk_Serial.write(buf,len);
#else
    Serial.write(buf, len);
#endif
*/
}

void comm_receive() {

  //Serial.println("COM_Receive");

  mavlink_message_t msg;
  mavlink_status_t status;
 
  // Echo for manual debugging
  // Serial.println("---Start---");


  while(Pixhawk_Serial.available()>0) {
    uint8_t c = Pixhawk_Serial.read();



    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

          }
          break;

 case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
  
            float Battery_Volts = sys_status.voltage_battery;
                    
            Serial.print("Volts: ");
            Serial.print(Battery_Volts / 1000 );
            Serial.print(" Amps: ");
            Serial.print(sys_status.current_battery);
            //Serial.print(" Sensors: ");
            //Serial.print(sys_status.onboard_control_sensors_health);
            //Serial.print("], [Comms loss (%): ");
            //Serial.print(sys_status.drop_rate_comm);
            //Serial.print("] ");

          }
          break;

  case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);

            Serial.println("PX PARAM_VALUE");
            Serial.print(" Value: ");
            Serial.print(param_value.param_value);
            Serial.print(" Count: ");
            Serial.print(param_value.param_count);
            Serial.print(" Index: ");
            Serial.print(param_value.param_index);
            Serial.print(" ID: ");
            Serial.print(param_value.param_id);
            Serial.print(" Type: ");
            Serial.print(param_value.param_type);
          }
          break;

  case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            //Serial.println("PX RAW IMU");
            //Serial.println(raw_imu.xacc);

          }
          break;

 case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);

            Serial.print(" ROLL : ");
            Serial.print(attitude.roll);
            Serial.print(" YAW : ");
            Serial.print(attitude.yaw);
            Serial.print(" PITCH : ");
            Serial.println(attitude.pitch);
          }
          break;



      }
    }
  }
}








void Command_long_ARM(){

// Define the system type (see mavlinkTypes.h for list of possible types) 
  
  // Initialize the required buffers 
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
                            // 1 = to arm the PIXHAWK
  CMD_LONG_param1  = 1;                             // 1 = to arm the PIXHAWK
  CMD_LONG_command = MAV_CMD_COMPONENT_ARM_DISARM;  // Arm/Disarm Command
  PIXHAWK_Armed = 1;
      
  // Pack the message
  //mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilotType, system_mode, custom_mode, system_state);
  mavlink_msg_command_long_pack(System_ID, Component_ID, &msg, Target_System, Target_Component, CMD_LONG_command, CMD_LONG_confirmation, CMD_LONG_param1, CMD_LONG_param2, CMD_LONG_param3, CMD_LONG_param4, CMD_LONG_param5, CMD_LONG_param6, CMD_LONG_param7);
  
  // Copy the message to send buffer 

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes) 
  Pixhawk_Serial.write(buf, len);

  // Arm the Mower
  Turn_On_Relay();
  SetPins_ToGoForwards();
  delay(1000);
  }


void Command_long_Disarm(){
  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  CMD_LONG_param1  = 0;                                 // 0 = to dis arm the PIXHAWK
  CMD_LONG_command = MAV_CMD_COMPONENT_ARM_DISARM;      // Arm/Disarm Command
  PIXHAWK_Armed = 0;
  
  // Pack the message
  mavlink_msg_command_long_pack(System_ID, Component_ID, &msg, Target_System, Target_Component, CMD_LONG_command, CMD_LONG_confirmation, CMD_LONG_param1, CMD_LONG_param2, CMD_LONG_param3, CMD_LONG_param4, CMD_LONG_param5, CMD_LONG_param6, CMD_LONG_param7);
   
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);   // Copy the message to send buffer 
  Pixhawk_Serial.write(buf, len);                         // Send the message (.write sends as bytes) 
 
  // Arm the Mower
  Turn_Off_Relay();
  Motor_Action_Stop_Motors();
  delay(1000);      
}


void Acro_Mode() {


// PIXHAWK Flight Control Modes
// 0 = Manual, 1 = Acro, 3 = Steering, 4 = Hold, 5 = Loiter, 6 = Follow, 7 = Simple, 10 =  Auto
// 11 = RTL, 12 = Smart_RTL, 15 = Guided



  Custom_Mode       = 1;                           ///< Custom mode, can be defined by user/adopter

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(System_ID, Component_ID, &msg, Target_System, Base_Mode, Custom_Mode);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Pixhawk_Serial.write(buf,len);

  
  }


void Pix_Manual_Mode() {


// PIXHAWK Flight Control Modes
// 0 = Manual, 1 = Acro, 3 = Steering, 4 = Hold, 5 = Loiter, 6 = Follow, 7 = Simple, 10 =  Auto
// 11 = RTL, 12 = Smart_RTL, 15 = Guided



  Custom_Mode       = 0;                           ///< Custom mode, can be defined by user/adopter

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(System_ID, Component_ID, &msg, Target_System, Base_Mode, Custom_Mode);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Pixhawk_Serial.write(buf,len);

  
  }

//https://www.locarbftw.com/arduino-mavlink-library-changing-flight-modes/

void Auto_Mode() {


// PIXHAWK Flight Control Modes
// 0 = Manual, 1 = Acro, 3 = Steering, 4 = Hold, 5 = Loiter, 6 = Follow, 7 = Simple, 10 =  Auto
// 11 = RTL, 12 = Smart_RTL, 15 = Guided



  Custom_Mode       = 10;                           ///< Custom mode, can be defined by user/adopter

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(System_ID, Component_ID, &msg, Target_System, Base_Mode, Custom_Mode);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Pixhawk_Serial.write(buf,len);

  
  }


void Follow_Mode() {


// PIXHAWK Flight Control Modes
// 0 = Manual, 1 = Acro, 3 = Steering, 4 = Hold, 5 = Loiter, 6 = Follow, 7 = Simple, 10 =  Auto
// 11 = RTL, 12 = Smart_RTL, 15 = Guided



  Custom_Mode       = 6;                           ///< Custom mode, can be defined by user/adopter

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(System_ID, Component_ID, &msg, Target_System, Base_Mode, Custom_Mode);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Pixhawk_Serial.write(buf,len);

  
  }

void Guided_Mode() {


// PIXHAWK Flight Control Modes
// 0 = Manual, 1 = Acro, 3 = Steering, 4 = Hold, 5 = Loiter, 6 = Follow, 7 = Simple, 10 =  Auto
// 11 = RTL, 12 = Smart_RTL, 15 = Guided



  Custom_Mode       = 15;                           ///< Custom mode, can be defined by user/adopter

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(System_ID, Component_ID, &msg, Target_System, Base_Mode, Custom_Mode);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Pixhawk_Serial.write(buf,len);
  }
  

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Pixhawk_Motor
//https://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
void Check_PIXHAWK_PWM() {

  Pixhawk_Motor_Test_Initiate = 1;
    
  if (Calibration_Done == 0) {
    Check_Motor_PWM_LH();
    Calibrate_Initial_PWM_LH();
    Check_Motor_PWM_RH();
    Calibrate_Initial_PWM_RH();

    Serial.print("LH Calib | Min =");
    Serial.print(Motor_C_PWM_Min);
    Serial.print("  Max =");
    Serial.print(Motor_C_PWM_Max);
    
    Serial.print(" RH Calib | Min =");
    Serial.print(Motor_D_PWM_Min);
    Serial.print("  Max =");
    Serial.println(Motor_D_PWM_Max);
    Calibration_Done = 1;
    }

  Calibration_Done = 1;

  int Wheel_ON_PWM = 20;

  Serial.print(  "Motor_C(LH)= ");

  Check_Motor_PWM_LH();
  Calculate_PWM_Arduino_Output_LH();
  
  if (PWM_Arduino_LH >= Wheel_ON_PWM) { 
    Serial.print("ON | PWM_Pix = ");
    Serial.print(PIXHAWK_PWM_Value_LH);
    Calculate_PWM_Arduino_Output_LH();
    Serial.print(" | PWM_Ard = ");
    Serial.print(PWM_Arduino_LH);    
  }
  if (PWM_Arduino_LH  < Wheel_ON_PWM) {
    Serial.print("OFF | PWM_Pix=");
    Serial.print(PIXHAWK_PWM_Value_LH);
    Calculate_PWM_Arduino_Output_LH();
    Serial.print(" | PWM_Ard=");
    Serial.print(PWM_Arduino_LH);    
  }

  Serial.print("    Motor_D(RH = ");

  Check_Motor_PWM_RH();
  Calculate_PWM_Arduino_Output_RH();

  if (PWM_Arduino_RH  >= Wheel_ON_PWM) {
    Serial.print(" ON | PWM_Pix = ");
    Serial.print(PIXHAWK_PWM_Value_RH);
    Calculate_PWM_Arduino_Output_RH();
    Serial.print(" | PWM_Ard = ");
    Serial.print(PWM_Arduino_RH);    
  }
  if (PWM_Arduino_RH < Wheel_ON_PWM) {
    Serial.print(" OFF | PWM_Pix = ");
    Serial.print(PIXHAWK_PWM_Value_RH);
    Calculate_PWM_Arduino_Output_RH();
    Serial.print(" | PWM_Ard = ");
    Serial.print(PWM_Arduino_RH);    
  }

  Serial.print("  |  ");
  PWM_Left = PWM_Arduino_LH;
  PWM_Right = PWM_Arduino_RH;
  Motor_Action_Dynamic_PWM_Steering();
  //Serial.println("");
}

 


void Check_Motor_PWM_LH() {
  PIXHAWK_PWM_Value_LH = pulseIn(PIXHAWK_LH_PWM, HIGH);
  delay(10);
  }

void Check_Motor_PWM_RH() {  
  PIXHAWK_PWM_Value_RH = pulseIn(PIXHAWK_RH_PWM, HIGH);
  delay(10);
}


 

void Calculate_PWM_Arduino_Output_LH() {
  int PWM_Range =   (Motor_C_PWM_Max - Motor_C_PWM_Min) / 255;
  PWM_Arduino_LH = (PIXHAWK_PWM_Value_LH - Motor_C_PWM_Min) / PWM_Range;
  if (PWM_Arduino_LH > 255) PWM_Arduino_LH = 255;
  if (PWM_Arduino_LH < 20) PWM_Arduino_LH = 0;
}

void Calculate_PWM_Arduino_Output_RH() {
  int PWM_Range =   (Motor_D_PWM_Max - Motor_D_PWM_Min) / 255;
  PWM_Arduino_RH = (PIXHAWK_PWM_Value_RH - Motor_D_PWM_Min) / PWM_Range;
  if (PWM_Arduino_RH > 255) PWM_Arduino_RH = 255;
  if (PWM_Arduino_RH < 20) PWM_Arduino_RH = 0;
}


void Calibrate_Initial_PWM_LH() {
    Motor_C_PWM_Min = PIXHAWK_PWM_Value_LH;
    Motor_C_PWM_Max = PIXHAWK_PWM_Value_LH + 305;
    }
  

void Calibrate_Initial_PWM_RH() {
    Motor_D_PWM_Min = PIXHAWK_PWM_Value_RH;
    Motor_D_PWM_Max = PIXHAWK_PWM_Value_RH + 305;
    }


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Pixhawk_Serial_Command
void Check_Serial_Input_PIXHAWK() {

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    int Command_Last = Command;
    Command = Serial.read();
    if (Command == 10) Command = Command_Last;

    // Print Comamnd to Serial Monitor
    if (Command != 0) {
        Serial.print(F(""));
        Serial.print(F(""));
        Serial.println(Command);
        }
  }

Execute_Serial_Command_PIXHAWK();
}


void Execute_Serial_Command_PIXHAWK() {

if (Command == 104) {                   // letter h
    Command = 0;
    Serial.println(F("  "));
    Serial.println(F("  "));
    Serial.println(F("      PIXHAWK HELP MENU"));
    Serial.println(F("      Enter the following letter"));
    Serial.println(F("  -----------------------------"));
    Serial.println(F("  a = ARM"));           //97
    Serial.println(F("  b = AUTO MODE"));       //98
    Serial.println(F("  c = ")); //99
    Serial.println(F("  d = DIS-ARM"));         //100
    Serial.println(F("  e = "));             //101
    Serial.println(F("  f = Follow Mode"));           //102
    Serial.println(F("  g = Guided Mode"));
    Serial.println(F("  h = "));             //104
    Serial.println(F("  i = "));          //105
    Serial.println(F("  j = "));
    Serial.println(F("  k = "));      //107
    Serial.println(F("  l = "));           //108
    Serial.println(F("  m = MANUAL Mode"));
    Serial.println(F("  n = ACRO Mode"));
    Serial.println(F("  o = "));             //111
    Serial.println(F("  p = "));      //112
    Serial.println(F("  q = Quit PIXHAWK Mode"));     //113
    Serial.println(F("  r = "));    //114
    Serial.println(F("  s = "));      //115
    Serial.println(F("  t = "));
    Serial.println(F("  u = "));
    Serial.println(F("  v = "));       //118
    Serial.println(F("  w = "));     //119
    Serial.println(F("  x = "));       //120
    Serial.println(F("  y = "));
    Serial.println(F("  z = "));    //122"));
    Serial.println(F("  "));
    Serial.println(F("  "));


    delay(7000);
    }

if (Command == 97) {                   // letter a
    Command = 0;
    Serial.println("");
    Serial.println("ARMED");
    Serial.println("");
    Command_long_ARM();
    }

if (Command == 100) {                   // letter d
    Command = 0;
    Serial.println("");
    Serial.println("DIS-ARM");
    Serial.println("");
    Command_long_Disarm();
    }

if (Command == 98) {                   // letter b
    Command = 0;
    Serial.println("");
    Serial.println("AUTO MODE");
    Serial.println("");
    Auto_Mode();
    }

if (Command == 102) {                   // letter f
    Command = 0;
    Serial.println("");
    Serial.println("Follow MODE");
    Serial.println("");
    Follow_Mode();
    }

if (Command == 103) {                   // letter f
    Command = 0;
    Serial.println("");
    Serial.println("Guided MODE");
    Serial.println("");
    Guided_Mode();
    }

if (Command == 109) {                   // letter m
    Command = 0;
    Serial.println("");
    Serial.println("Manual MODE");
    Serial.println("");
    Pix_Manual_Mode();
    }

if (Command == 110) {                   // letter n
    Command = 0;
    Serial.println("");
    Serial.println("ACRO Mode");
    Serial.println("");
    Acro_Mode();
    }

if (Command == 113) {                   // letter q
    Command = 0;
      Serial.println("");
      Serial.println("Mower set to ReP_AL Mode");
      Serial.println("");
      Mower_Docked = 1;
      Mower_PIXHAWK = 0;
      }
      
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Rain
void Check_if_Raining_From_Nano() {

  if (Rain_Sensor_Installed == 0) {
    Rain_Hit_Detected = 0; 
  }
  
  if (Rain_Sensor_Installed == 1) {
      
      if ((Rain_Detected == 1) || (Rain_Detected == 0)){                            // If the value recieved is equal to 1 or 0 as expected then print the value to the serial monitor
        Serial.print(F("Rain:"));
        Serial.print(Rain_Detected);
        Serial.print("|");
        Print_Raining_LCD();
        if (Rain_Detected == 1) Rain_Hit_Detected = Rain_Hit_Detected + 1;
        if (Rain_Detected == 0) Rain_Hit_Detected = Rain_Hit_Detected - 1;
        if (Rain_Hit_Detected < 0) Rain_Hit_Detected = 0;
        }
      if ((Rain_Detected != 1) && (Rain_Detected !=0)) {
        Serial.print(F("Rain:"));
        Serial.print(Rain_Detected);
        Serial.print("|");
        Print_Raining_LCD();
        Rain_Detected = 0;
        Rain_Hit_Detected = Rain_Hit_Detected + 1; 
        }
      
    
  else {
    Serial.print(F("Rain:"));
    Serial.print("_|");  
    Print_Raining_LCD();
    Rain_Detected = 0;
    }
  
  Serial.print("RHit:");
  Serial.print(Rain_Hit_Detected);
  Serial.print("|");
  

if ( (Mower_Running == 1) && (Rain_Hit_Detected == Rain_Total_Hits_Go_Home) ) {
  Motor_Action_Stop_Motors();
  Serial.println(F(""));
  Serial.println(F("Rain detected"));
  Serial.println(F(""));
  Print_Raining_LCD();
  delay(2000);
  
  if (Use_Charging_Station == 1) Manouver_Go_To_Charging_Station();            // If the Mower is running then go to the charge station. 
  if (Use_Charging_Station == 0) Manouver_Park_The_Mower();
  }
  }
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Serial_Command
void Check_Serial_Input() {
  
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    int Command_Last = Command;
    Command = Serial.read();
    if (Command == 10) Command = Command_Last;

    // Print Comamnd to Serial Monitor
    if (Command != 0) {
        Serial.print(F(""));
        Serial.print(F(""));
        Serial.println(Command);
        }
  }

Execute_Serial_Command();
}


void Execute_Serial_Command() {
// Rotate 20 degrees CCW

if (Command == 104) {                   // letter h
    Command = 0;
    Serial.println(F("  "));
    Serial.println(F("  "));
    Serial.println(F("      ReP_AL Mower  HELP MENU"));
    Serial.println(F("  "));
    Serial.println(F("      Enter the following letter to execute command"));
    Serial.println(F("      ----------------------------------------------"));
    Serial.println(F("  a = "));           //97
    Serial.println(F("  b = "));       //98
    Serial.println(F("  c = Goto Charging Station")); //99
    Serial.println(F("  d = Set to Docked"));         //100                
    Serial.println(F("  e = Exit dock"));             //101
    Serial.println(F("  f = Set Mower Mode to PIXHAK/ReP_AL"));           //102
    Serial.println(F("  g = "));  //103
    Serial.println(F("  h = Help Menu"));             //104
    Serial.println(F("  i = Drill ON/OFF"));          //105
    Serial.println(F("  j"));
    Serial.println(F("  k = Test Drill Cycle"));      //107
    Serial.println(F("  l = Spikes Down"));           //108
    Serial.println(F("  m = Manual Mode"));
    Serial.println(F("  n"));
    Serial.println(F("  o = Spikes UP"));             //111
    Serial.println(F("  p = Pause/Park Mower"));      //112
    Serial.println(F("  q = Quick Start Mower"));     //113
    Serial.println(F("  r = Rain Sensor ON/OFF"));    //114
    Serial.println(F("  s = "));      //115
    Serial.println(F("  t = "));           //116
    Serial.println(F("  u = "));     //117
    Serial.println(F("  v = Fake Voltage of 12.6V"));       //118
    Serial.println(F("  w = Fake Wire Signal ON/OFF"));    //119
    Serial.println(F("  x = Mission start Pixhawk"));       //120
    Serial.println(F("  y = Fake Wheel_Blocked_Count=4"));  //121
    Serial.println(F("  z = Fake All Volts, Wheel Amps ")); //122"));
    Serial.println(F("  "));
    Serial.println(F("  "));

        
    delay(7000);   
    }

if (Command == 96) {                   // letter c
    Command = 0;
    //analogWrite(PWM_1, 255);
    }


if (Command == 99) {                   // letter c
    Command = 0;
    Manouver_Go_To_Charging_Station();
    }

if (Command == 100) {                   // letter d
    Command = 0;
    Mower_Docked = 1 ;
    Mower_Parked = 0 ;
    }

if (Command == 101) {                   // letter e
    Command = 0;
    Alarm_Start_Exit_Zone_2();
    }

if (Command == 102) {                   // letter f
    Command = 0;
    Serial.println("");
    Serial.println(F("Mower set to PIXHAWK Mode"));
    Serial.println("");
    Mower_Docked = 0;
    Mower_PIXHAWK = 1;
    }



if (Command == 105) {                   // letter i
    Command = 0;
    int Skip = 0;

    
//    if (Drill_ON == 0) {
//      Serial.println(F(""));
//      Serial.println(F("Drill = ON"));
//      Turn_On_Relay();
//      delay(1000);
//      Motor_Action_Spin_Drill();
//      Drill_ON = 1;
//      Skip = 1;
//      }
//    if ((Drill_ON == 1) && (Skip == 0)) {
//      Serial.println(F(""));
//      Serial.println(F("Drill = OFF"));
//      Motor_Action_Stop_Drill();
//      Turn_Off_Relay();
//      Drill_ON = 0;
//      }
     }


if (Command == 109) {                   // letter m  
    Command = 0;
    Manouver_Park_The_Mower(); 
    delay(100);
    Manouver_Manual_Mode(); 
    Turn_On_Relay();
    }

if (Command == 112) {                   // letter p  
    Command = 0;
    Manouver_Park_The_Mower();
    }

if (Command == 113) {                   // letter q  
    Command = 0;
    if (Mower_Docked == 1) {
      Serial.println(F("Overriding Docked Status - Starting Mower"));
      delay(2000);
      Mower_Docked = 0;
      }
    Alarm_Start_Quick_Go();
    }


if (Command == 114) {                   // letter r
    Command = 0;
    if (Rain_Sensor_Installed == 0) {
      Rain_Sensor_Installed = 1;
      Serial.println(F("Rain Sensor Enabled = ON"));
      }
    if (Rain_Sensor_Installed == 1) {
      Rain_Sensor_Installed = 0;
      Serial.println(F("Rain Sensor Enabled = OFF"));
      }
    }


if (Command == 118) {                   // letter v  
    Command = 0;
    }

if (Command == 119) {                   // letter w
    Command = 0;
    bool Skip = 0;
    if (Fake_Wire == 0) {
      Fake_Wire = 1;
      Serial.println(F("Fake_Wire Signal Enabled = ON"));
      Skip = 1;
      }
    if ((Fake_Wire == 1) && (Skip == 0)) {
      Fake_Wire = 0;
      Serial.println(F("Fake_Wire Signal Enabled = OFF"));
      }
    }


if (Command == 120) {                   // letter x  
    Command = 0;
    if (Mower_Docked == 1) {
      Serial.println(F("PIXHAWK Mission Start"));
      delay(2000);
      Mower_Docked = 0;
      }
    //Command_long_Mission_Start();
    }


if (Command == 121) {                   // letter x
  Command = 0;

    if (!Fake_WheelAmp) {
      Fake_WheelAmp = 1;
      Serial.println(F("Fake_WheelAmp Signal Enabled = ON"));
      }
    else if (Fake_WheelAmp) {
      Fake_WheelAmp = 0;
      Serial.println(F("Fake_WheelAmp Signal Enabled = OFF"));
    }
}

if (Command == 122) {                   // letter v
    Command = 0;
    int Skip = 0;

    
    if (Fake_All_Settings == 0) {
      Serial.println(F(""));
      
      Wheel_Amp_Sensor_ON = 0;      
      Fake_All_Settings = 1;     

      Serial.println(F(""));
      Serial.println(F("Fake Volts Enabled = ON"));
      Serial.println(F("Wheel Amp Sensor Disabled"));
      Serial.println(F("Fake Settings ON !!!"));
      Serial.println(F(""));
      Skip = 1;
      }
    
    
    if ((Fake_All_Settings == 1) && (Skip == 0)) {
      Serial.println(F(""));
      
      Perimeter_Wire_Enabled = 1;
      Wheel_Amp_Sensor_ON = 1;
      Fake_All_Settings = 0;
      
      Serial.println(F(""));
      Serial.println(F("All Sensors normal status"));
      Serial.println(F(""));
      }
    

    }

// Blocking bit for quick enable/ disable function when program run to detect which function consume big time
// For using add bit to the logic
// if (F_EN[14] && Mower_Parked == 1) Running_Test_for_Boundary_Wire();

// add this line to setup to enable all options during start up
// for(int i=0; i<33; i++) { F_EN[i] = true; }

if (Command == 63) { for(int i=0; i<33; i++) { F_EN[i] = false; } } // letter ?
if (Command == 64) { for(int i=0; i<33; i++) { F_EN[i] = true; } } // letter @
if (Command == 65) { F_EN[0] = false; } // letter A
if (Command == 66) { F_EN[1] = false; } // letter B
if (Command == 67) { F_EN[2] = false; } // letter C
if (Command == 68) { F_EN[3] = false; } // letter D
if (Command == 69) { F_EN[4] = false; } // letter E
if (Command == 70) { F_EN[5] = false; } // letter F
if (Command == 71) { F_EN[6] = false; } // letter G
if (Command == 72) { F_EN[7] = false; } // letter H
if (Command == 73) { F_EN[8] = false; } // letter I
if (Command == 74) { F_EN[9] = false; } // letter J
if (Command == 75) { F_EN[10] = false; } // letter K
if (Command == 76) { F_EN[11] = false; } // letter L
if (Command == 77) { F_EN[12] = false; } // letter M x
if (Command == 78) { F_EN[13] = false; } // letter N
if (Command == 79) { F_EN[14] = false; } // letter O
if (Command == 80) { F_EN[15] = false; } // letter P

if (Command == 81) { F_EN[16] = false; } // letter Q
if (Command == 82) { F_EN[17] = false; } // letter R
if (Command == 83) { F_EN[18] = false; } // letter S
if (Command == 84) { F_EN[19] = false; } // letter T
if (Command == 85) { F_EN[20] = false; } // letter U
if (Command == 86) { F_EN[21] = false; } // letter V
if (Command == 87) { F_EN[22] = false; } // letter W
if (Command == 88) { F_EN[23] = false; } // letter X
if (Command == 89) { F_EN[24] = false; } // letter Y
if (Command == 90) { F_EN[25] = false; } // letter Z
if (Command == 91) { F_EN[26] = false; } // letter [
if (Command == 93) { F_EN[27] = false; } // letter ] X
if (Command == 48) { F_EN[28] = false; } // letter 0
if (Command == 49) { F_EN[29] = false; } // letter 1
if (Command == 50) { F_EN[30] = false; } // letter 2
if (Command == 51) { F_EN[31] = false; } // letter 3

if (Command == 57) { F_EN[32] = false; } // letter 9 x


}


void Check_Mower_Status() {
      Serial.println("");
      Serial.print(F("Mower Status:  "));
      Serial.print(F("Running ="));
    Serial.print(Mower_Running);
      Serial.print(F("Parked ="));
      Serial.print(Mower_Parked);
      Serial.print(F(" | Docked = "));
      Serial.print(Mower_Docked);
      Serial.print(F(" | Tracking = "));
      Serial.print(Tracking_Wire);
      Serial.print(F(" |  Exit Dock = "));
      Serial.print(Exiting_Dock);
      Serial.print(F(" |  Error Value = "));
      Serial.print(Mower_Error_Value);
      Serial.println("");
      } 
      

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Simulator
void Execute_Fake_It() {
      
      Serial.print("Fake:ON|");
      Volts = 12.6;
      Amps = 0.2;
      Outside_Wire = 0;                                               // Outside wire variable is tuend off
      Outside_Wire_Count = 0;                                         // The number of outside wire counts is reset to 0
      Wire_Refind_Tries = 0;  
      MAG_Now = -1500;
      Wire_Off = 0;
      Wire_Detected = 1;
      Rain_Sensor_Installed = 0;

      Fake_Loops = Fake_Loops++;
      if (Fake_Loops  > 50) {
        Volts = Battery_Min - 0.2 ;
      }
      }


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Sonar
void Check_Sonar_Sensors() {
  
  // Ping Sonar sensors

  //Clears the Trig Pin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin3, LOW);

 // Pings each sonar at a 15ms interval

 if (Sonar_2_Activate == 1) distance2 = PingSonarX(trigPin2, echoPin2, 2, 2, 2, 0, 0);         //SONAR2
 if (Sonar_1_Activate == 1) distance1 = PingSonarX(trigPin1, echoPin1, 1, 1, 1, 1, 0);         //SONAR1
 if (Sonar_3_Activate == 1) distance3 = PingSonarX(trigPin3, echoPin3, 3, 3, 3, 2, 0);         //SONAR3

 }
  


/* SONAR Function
************************************************************************************/
// Function to Ping the Sonar calculate the distance from Object to the Sonars.
// Distance calculated is printed to serial printer and displays X or _ on the LCD Screen
// Distance calculated is then used for the object avoidance logic
// Sonars used can be activated in the settings.

int PingSonarX(int trigPinX, int echoPinX, int distanceX, long durationX, int sonarX, int LCDRow, int LCDColumn) {
  pinMode(trigPinX, OUTPUT);
  pinMode(echoPinX, INPUT);
  //Sets the trigPin at High state for 10 micro secs sending a sound wave
  digitalWrite(trigPinX, HIGH);
  digitalWrite(trigPinX, LOW);
  delayMicroseconds(10);

  /*Reads the echoPin for the bounced wave and records the time in microseconds*/

  durationX = pulseIn(echoPinX, HIGH, 10000);  //LOOP_OPTIMIZE added timeout 10ms - OK for detection object in distance 40cm
  // TODO test Arduino NewPing library https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

  /*Calculates the distance in cm based on the measured time*/
  distanceX = durationX * 0.034 / 2;
  delay(5); //LOOP_OPTIMIZE removed

  /* If a 0 distance is measured normally the Sonar ping has not been received.
    distance is then set to 999cm so the missed ping is not seen as an object detected.*/
  if (distanceX == 0) {
    distanceX = 999;
    Serial.print(F("|S"));
    Serial.print(sonarX);
    Serial.print(F(":"));
    Serial.print(F("NP!"));
    Serial.print(F("|"));

    // Check why removed
    //if (sonarX == 1) Sonar_1_Error++;
    //if (sonarX == 2) Sonar_2_Error++;
    //if (sonarX == 3) Sonar_3_Error++;
    
    if (Sonar_1_Error > Sonar_Max_Error_Shutdown) {
      Sonar_1_Activate = 0;
      Serial.println(F(""));
      Serial.println(F("Sonar 1 - Shutdown Error - Check Wiring"));
      Serial.println(F(""));    
      
      }
    if (Sonar_2_Error > Sonar_Max_Error_Shutdown) {
      Sonar_2_Activate = 0;
      Serial.println(F(""));
      Serial.println(F("Sonar 2 - Shutdown Error - Check Wiring"));
      Serial.println(F(""));    
      }
    if (Sonar_3_Error > Sonar_Max_Error_Shutdown) {
      Sonar_3_Activate = 0;
      Serial.println(F(""));
      Serial.println(F("Sonar 3 - Shutdown Error - Check Wiring"));
      Serial.println(F(""));    
      }
  }

  /*Prints the Sonar letter and distance measured on the serial Monitor*/
  Serial.print(F("|S"));
  Serial.print(sonarX);
  Serial.print(F(":"));
  Serial.print(distanceX);
  Serial.print(F("cm"));
  Serial.print(F("/"));

  /*If sonar distance is less than maximum distance then an object is registered to avoid*/
  if (distanceX <= maxdistancesonar ) {
    //Prints that Sonar X has detected an object to the Mower LCD.
    
    #if defined(LCD_KEYPAD)
    lcd.setCursor(LCDRow, LCDColumn);                //sets location for text to be written
    lcd.print("X");
    #endif
    
    if (sonarX == 1) {
        Sonar_Hit_1_Total = (Sonar_Hit_1_Total + 1);
        Serial.print(Sonar_Hit_1_Total);
        Sonar_1_Error = 0;
        }
      if (sonarX == 2) {
        Sonar_Hit_2_Total = (Sonar_Hit_2_Total + 1);
        Serial.print(Sonar_Hit_2_Total);
        Sonar_2_Error = 0;
        }
      if (sonarX == 3) {
        Sonar_Hit_3_Total = (Sonar_Hit_3_Total + 1);
        Serial.print(Sonar_Hit_3_Total);
        Sonar_3_Error = 0;
        }      
    if ( (Sonar_Hit_1_Total == Max_Sonar_Hit) || (Sonar_Hit_2_Total == Max_Sonar_Hit) || (Sonar_Hit_3_Total == Max_Sonar_Hit) ) {
      Sonar_Hit = 1;  
      Print_Sonar_Hit();
      Serial.println(F(""));
      Serial.println("Sonar Hit Detected");
      }
    
    }

  /*If sonar distance is greater than maximum distance then no object is registered to avoid*/
  if (distanceX > maxdistancesonar) {
    //Prints that the path of Sonar X is open.
    
    #if defined(LCD_KEYPAD)
    lcd.setCursor(LCDRow, LCDColumn);                 //sets location for text to be written
    lcd.print("_");
    #endif
    
      if (sonarX == 1) {
        Sonar_Hit_1_Total = 0;
        Serial.print(Sonar_Hit_1_Total);
        }
      if (sonarX == 2) {
        Sonar_Hit_2_Total = 0;
        Serial.print(Sonar_Hit_2_Total);
        }
      if (sonarX == 3) {
        Sonar_Hit_3_Total = 0;
        Serial.print(Sonar_Hit_3_Total);
        }   
    }
   

  return distanceX;
  return sonarX;
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Special
void Special_Cut_Under_Trampoline_Function() {
  //Enter Code Here
}



// after wire tracking the code to bering the mower to a sensible position away from the wire
// This poart of the code should eb modified to fit your garden needs
void Special_Move_Into_Garden_Zone_X() {
    Serial.println(F(""));
    Serial.println(F("Arrived at Exit Zone"));
    Serial.println(F("Moving into Garden"));
    Motor_Action_Stop_Motors();                                     // Stop the wheel motors
    if (CCW_Tracking_To_Start == 1) SetPins_ToTurnLeft();           // Turn left
    if (CW_Tracking_To_Start == 1) SetPins_ToTurnRight();           // Turn Right    
    Motor_Action_Go_Full_Speed();                                       
    delay(400);                                                     // Turn left for 0.7seconds
    SetPins_ToGoForwards();                                         // Move forward a l
    Motor_Action_Go_Full_Speed();                                       
    delay(3000);
    Motor_Action_Stop_Motors();                                     // Stop the wheel motors.
    SetPins_ToGoForwards();                                         // Get ready to move off
    }

void Manouver_Exit_From_Docking_Station(){
    #if defined(LCD_KEYPAD)
    lcd.clear();
    lcd.print(F("Exiting Garage"));
    #endif

    Serial.println(F("Exiting Docking Garage"));
    delay(2000);
    SetPins_ToGoBackwards();                                        // Prepare motors pins to go Backwards
    Motor_Action_GoFullSpeed_Out_Garage();                          // Turn the wheels
    Serial.print(F("Left Wheel PWM:"));
    Serial.print(PWM_MaxSpeed_LH);
    Serial.print("|");
    Serial.print(F("Right Wheel PWM:"));
    Serial.println(PWM_MaxSpeed_RH);   
    delay (4000);                                             // Backwards time
    Motor_Action_Stop_Motors();                                       // Stop

    if (CCW_Tracking_To_Start == 1) {
        SetPins_ToTurnLeft();                                           // Prepare motors to turn left
        Motor_Action_Turn_Speed();                                      // Turn the wheels
        delay(500);                                                    // Turn time
        Motor_Action_Stop_Motors();                                       // Stop
        SetPins_ToGoBackwards();                                        // Set again to go backwards
        Motor_Action_Go_Full_Speed();                                   // Turn the wheels
        delay (500);                                                    // Backwards Time
        SetPins_ToTurnLeft();                                           // Set to go left             
        Motor_Action_Turn_Speed();                                      // Turn the wheels
        delay(200);                                                     // Turning time
        Motor_Action_Stop_Motors();                                     // Stop
        }
    if (CW_Tracking_To_Start == 1) {
        SetPins_ToTurnRight();                                           // Prepare motors to turn left
        Motor_Action_Turn_Speed();                                      // Turn the wheels
        delay(500);                                                    // Turn time
        Motor_Action_Stop_Motors();                                       // Stop
        SetPins_ToGoBackwards();                                        // Set again to go backwards
        Motor_Action_Go_Full_Speed();                                   // Turn the wheels
        delay (500);                                                    // Backwards Time
        SetPins_ToTurnRight();                                           // Set to go left             
        Motor_Action_Turn_Speed();                                      // Turn the wheels
        delay(200);                                                     // Turning time
        Motor_Action_Stop_Motors();                                     // Stop
        }
    SetPins_ToGoForwards();                                         // Set to go wheel motor pins to go forwards
    Motor_Action_Stop_Motors();                                     // Stop / Park the mower here

    if ((GPS_Enabled == true) && (GPS_Type == 2) && (Run_PIXHAWK_Mission_At_Exit == true)) {
      Serial.println("");
      Serial.println(F("Mower set to PIXHAWK Mode"));
      Serial.println("");
      Mower_Docked = 0;
      Mower_PIXHAWK = 1;
      Serial.println("");
      Serial.println("ARMED");
      Serial.println("");
      Command_long_ARM();
      Serial.println("");
      Serial.println("AUTO MODE");
      Serial.println("");
      Auto_Mode();
      }

    #if defined(LCD_KEYPAD)
    lcd.clear();                                                    // Clears the LCD display
    lcd.print("Garage Clear");                                      // Prints to the LCD screen       
    delay(500);
    lcd.clear();
    #endif
    
    }



void Specials_Find_Wire_Track()  {

  Serial.println(F(""));
  Serial.println(F("Lost Mower - find wire Track"));
  
  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.print("Finding Wire...  ");
  #endif
  
#if defined(WDT)
  wdt_disable(); // disable WDT because of a lot of delay() used or reset WDT after each delay() ???
#endif // -(WDT)-

  Motor_Action_Stop_Spin_Blades();
  delay(5);
  Abort_Wire_Find = 0;
  //No_Wire_Found = 0;
  Running_Test_for_Boundary_Wire();                                                                   // Check to see that the wire is on.

  for (int i = 0; i <= 1; i++) {
    Serial.print(F("Position Try = "));
    Serial.println(i);
    ADCMan.run();
    UpdateWireSensor();
    delay(20);
    ADCMan.run();
    UpdateWireSensor();
    delay(20);
    PrintBoundaryWireStatus();
    //No_Wire_Found = 0;
    Wire_Find_Attempt = 0;  

    // First go backwards if the mower is outside the wire
    if ( inside == false) {                                    // If the mower is outside the wire then run the following code.
      Serial.println("Reversing to find the wire");
      ADCMan.run();
      UpdateWireSensor();
      PrintBoundaryWireStatus();
      Motor_Action_Stop_Motors();                                                           // Stop all wheel motion
      Loop_Cycle_Mowing = 0;
      if ((WIFI_Enabled == 1) && (Manual_Mode == 0)) Get_WIFI_Commands();    
      delay(1000);
      SetPins_ToGoBackwards();                                                              // Set the mower to back up
      delay(100);
      
      #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print("Backwards Try...  ");
      lcd.setCursor(0,1);
      lcd.print("Finding Wire  ");
      delay(100);
      #endif
      
      // While the mower is still outside the boundary wire run this code unless andabort signal from the APP comes or it runs out of tries.
      while (( inside != true) && (Wire_Find_Attempt < 100) ){
        Motor_Action_Go_Full_Speed();                                                       // Go full speed (in this case backwards)
        UpdateWireSensor();                                                                 // Read the wire sensor and see of the mower is now  or outside the wire
        ADCMan.run();
        PrintBoundaryWireStatus();                                                          // Prints of the status of the wire sensor readings.
        Serial.println(F(""));
        Wire_Find_Attempt = Wire_Find_Attempt + 1;                                                      // Counts how many loops have passed to find the wire.
        Serial.print(F("No Wire Count Backwards:"));
        Serial.print(Wire_Find_Attempt);
        Serial.print("|");
        }
      
      }
      
      Motor_Action_Stop_Motors();
      Loop_Cycle_Mowing = 999;
      if (Manual_Mode == 0) Get_WIFI_Commands();  
      delay(5);
      }
    
    Wire_Find_Attempt = 0; 
    // Code to go forwards until the mower is outside/ON the wire
    if ( inside == true) {             // If the Mower is situated  the wire then run the following code.
        Serial.println(F("Moving Forwards to find the wire"));
        ADCMan.run();
        UpdateWireSensor();
        Serial.println(F("CODE POSITION - MOTOR FORWARDS LOOP:  If statements"));
        PrintBoundaryWireStatus();
        Motor_Action_Stop_Motors();
        Loop_Cycle_Mowing = 999;
        if (Manual_Mode == 0) Get_WIFI_Commands();  
        delay(1000);  
        SetPins_ToGoForwards();                                                             // Set the motors to move the mower forwards
        delay(100);
        
        #if defined(LCD_KEYPAD)
        lcd.clear();
        lcd.print(F("Forward Try...  "));
        lcd.setCursor(0,1);
        lcd.print(F("Finding Wire  "));
        delay(100);    
        #endif
        
        // resets the cycles
        while (( inside != false) && (Wire_Find_Attempt < 100)) {                               // Move the mower forward until mower is outisde/ON the wire fence or 500 cycles have passed
          Motor_Action_Go_Full_Speed();                                                     // Go full speed (in this case forwards)
          UpdateWireSensor();                                                               // Read the wire sensor and see of the mower is now  or outside the wire
          ADCMan.run();
          PrintBoundaryWireStatus();                                                        // Prints of the status of the wire sensor readings.
          Serial.println(F(""));
          Wire_Find_Attempt = Wire_Find_Attempt + 1;                                                    // Counts how many loops have passed to find the wire.
          Serial.print("No Wire Count Forwards:");
          Serial.print(Wire_Find_Attempt);
          Serial.print("|");
          }
      }
      Motor_Action_Stop_Motors();
      Loop_Cycle_Mowing = 0;
      if (Manual_Mode == 0) Get_WIFI_Commands();  
      delay(5);
    }


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// TFT_RX
void Receive_7_Data_Sets_Serial3() {
  
  Serial.println("Receiving Data Sets");

  byte recvBuff [15] = {0};
  bool confRecv = false;
  int testNum=0; //timeout
  while(!confRecv && testNum<500) {
    if (SerialCom3.update ())
    {
      byte length = SerialCom3.getLength ();
      if (length > sizeof (recvBuff)) length = sizeof (recvBuff);
      memcpy (&recvBuff, SerialCom3.getData (), length);

      if (recvBuff[0] == 17) {
        Data1 = recvBuff[1] | recvBuff[2] << 8;
        Data2 = recvBuff[3] | recvBuff[4] << 8;
        Data3 = recvBuff[5] | recvBuff[6] << 8;
        Data4 = recvBuff[7] | recvBuff[8] << 8;
        Data5 = recvBuff[9] | recvBuff[10] << 8;
        Data6 = recvBuff[11] | recvBuff[12] << 8;
        Data7 = recvBuff[13] | recvBuff[14] << 8;

        Serial.println("Data Sets Received");
        confRecv = true;
      }
    }  // end if something received
    testNum++; delay(1);
  }

  Serial.print(F("Data1:"));
  Serial.println(Data1);
  Serial.print(F("Data2:"));
  Serial.println(Data2);
  Serial.print(F("Data3:"));
  Serial.println(Data3);  
  Serial.print(F("Data4:"));
  Serial.println(Data4);
  Serial.print(F("Data5:"));
  Serial.println(Data5);
  Serial.print(F("Data6:"));
  Serial.println(Data6);
  Serial.print(F("Data7:"));
  Serial.println(Data7);
  Serial.println(F(""));
}




void Receive_Data_From_TFT()  {

  // Receive the Sonar Values Back again
  if (TFT_Menu_Command == 910) {     
      Serial.println(F("Receiving Sonar Data from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Sonar_1_Activate  = Data1;
      Sonar_2_Activate  = Data2;
      Sonar_3_Activate  = Data3;
      maxdistancesonar  = Data4;
      Max_Sonar_Hit     = Data5;
      Data6 = 0;
      Data7 = 0;
      Serial.print(F("S1 ON = "));
      Serial.println(Sonar_1_Activate);
      Serial.print(F("S2 ON = "));
      Serial.println(Sonar_2_Activate);
      Serial.print(F("S3 ON = "));
      Serial.println(Sonar_3_Activate);
      Serial.print(F("Max Distance = "));
      Serial.println(maxdistancesonar);
      Serial.print(F("Sensitivity = "));
      Serial.println(Max_Sonar_Hit);

      #if defined(BOARD_DUE)
      dueFlashStorage.write(37, 1);
      dueFlashStorage.write(38, Sonar_1_Activate);
      dueFlashStorage.write(39, 1);
      dueFlashStorage.write(40, Sonar_2_Activate);
      dueFlashStorage.write(41, 1);
      dueFlashStorage.write(42, Sonar_3_Activate);
      dueFlashStorage.write(65, 1);
      dueFlashStorage.write(66, maxdistancesonar); 
      dueFlashStorage.write(63, 1);
      dueFlashStorage.write(64, Max_Sonar_Hit);         
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      EEPROM.write(37, 1);
      EEPROM.write(38, Sonar_1_Activate);
      EEPROM.write(39, 1);
      EEPROM.write(40, Sonar_2_Activate);
      EEPROM.write(41, 1);
      EEPROM.write(42, Sonar_3_Activate);
      EEPROM.write(65, 1);
      EEPROM.write(66, maxdistancesonar); 
      EEPROM.write(63, 1);
      EEPROM.write(64, Max_Sonar_Hit);         
      Serial.println(F("Saved to EEPROM"));
      #endif

          
      Serial.println(F(" "));
      }

// Receive the Battery Values Back again
  if (TFT_Menu_Command == 911) {
      Serial.println(F("Receiving Battery Data from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Battery_Min               = Data1;
      Low_Battery_Instances_Chg = Data2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;                                          
      Serial.print(F("Battery Min = "));
      Battery_Min = Battery_Min / 10;    
      Serial.println(Battery_Min);      
      Serial.print(F("Battery Sensitivity = "));
      Serial.println(Low_Battery_Instances_Chg);

      #if defined(BOARD_DUE)
      dueFlashStorage.write(25, 1);
      dueFlashStorage.write(26, (Battery_Min * 10));   
      dueFlashStorage.write(85, 1);
      dueFlashStorage.write(86, Low_Battery_Instances_Chg); 
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      EEPROM.write(25, 1);
      EEPROM.write(26, (Battery_Min * 10));   
      EEPROM.write(85, 1);
      EEPROM.write(86, Low_Battery_Instances_Chg); 
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));
      }
  
// Receive the Sensor Menu 1 Values Back again
  if (TFT_Menu_Command == 96) {
      Serial.println(F("Receiving Sensor Menu 1 Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Perimeter_Wire_Enabled  = Data1;
      WIFI_Enabled            = Data2;
      Bumper_Activate_Frnt    = Data3;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;  
      Serial.print(F("Wire Module Activated = "));
      Serial.println(Perimeter_Wire_Enabled);
      Serial.print(F("WIFI Enabled = "));
      Serial.println(WIFI_Enabled);
//      Serial.print(F("Bumper Activated = "));
//      Serial.println(Bumper_Activate_Frnt);    

      #if defined(BOARD_DUE)
      dueFlashStorage.write(67 , 1);
      dueFlashStorage.write(68 , Perimeter_Wire_Enabled);
      dueFlashStorage.write(81 , 1);
      dueFlashStorage.write(82 , WIFI_Enabled);
//      dueFlashStorage.write(90 , 1);
//      dueFlashStorage.write(91 , Bumper_Activate_Frnt);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      EEPROM.write(67 , 1);
      EEPROM.write(68 , Perimeter_Wire_Enabled);
      EEPROM.write(81 , 1);
      EEPROM.write(82 , WIFI_Enabled);
//      EEPROM.write(90 , 1);
//      EEPROM.write(91 , Bumper_Activate_Frnt);
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
  }



// Receive the Rain Sensor Values Back again
  if (TFT_Menu_Command == 912) {
      Serial.println(F("Receiving Rain Sensor Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Rain_Sensor_Installed   = Data1;
      Rain_Total_Hits_Go_Home = Data2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;  
      Serial.print(F("Rain ON = "));
      Serial.println(Rain_Sensor_Installed);
      Serial.print(F("Sensitivity = "));
      Serial.println(Rain_Total_Hits_Go_Home);   

      #if defined(BOARD_DUE)
      dueFlashStorage.write(77 , 1);
      dueFlashStorage.write(78 , Rain_Sensor_Installed);
      dueFlashStorage.write(79, 1);
      dueFlashStorage.write(80, Rain_Total_Hits_Go_Home);   
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      EEPROM.write(77 , 1);
      EEPROM.write(78 , Rain_Sensor_Installed);
      EEPROM.write(79, 1);
      EEPROM.write(80, Rain_Total_Hits_Go_Home);   
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));
    }


// Receive the Mower Motor Values Back again
  if (TFT_Menu_Command == 914) {
      Serial.println(F("Receiving Mower Motor Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      PWM_MaxSpeed_LH   = Data1;
      PWM_MaxSpeed_RH   = Data2;
      PWM_Slow_Speed_LH = Data3;
      PWM_Slow_Speed_RH = Data4;
      Slow_Speed_MAG    = Data5;
      Wheels_Activate   = Data6;
      Data7 = 0;  
      Serial.print(F("Wheel Motor PWM LH = "));
      Serial.println(PWM_MaxSpeed_LH);
      Serial.print(F("Wheel Motor PWM RH = "));
      Serial.println(PWM_MaxSpeed_RH);
      Serial.print(F("Wheel Motor Wire Speed PWM LH = "));
      Serial.println(PWM_Slow_Speed_LH);
      Serial.print(F("Wheel Motor Wire Speed PWM RH = "));
      Serial.println(PWM_Slow_Speed_RH);
      Serial.print(F("Slow Speed MAG = "));     
      Slow_Speed_MAG = Slow_Speed_MAG * 10;
      Serial.println(Slow_Speed_MAG);
      Serial.print(F("Wheels_Activate = ")); // RVES added
    Serial.println(Wheels_Activate); // RVES added
      Serial.println(F(" "));     

       #if defined(BOARD_DUE)
      dueFlashStorage.write(13, 1);
      dueFlashStorage.write(14, PWM_MaxSpeed_LH);
      dueFlashStorage.write(15, 1);
      dueFlashStorage.write(16, PWM_MaxSpeed_RH);    
      dueFlashStorage.write(94, 1);
      dueFlashStorage.write(95, PWM_Slow_Speed_LH);
      dueFlashStorage.write(96, 1);
      dueFlashStorage.write(97, PWM_Slow_Speed_RH);   
      dueFlashStorage.write(98, 1);
      dueFlashStorage.write(99, ((Slow_Speed_MAG * -1 ) / 10));   
      Serial.println(F("Saved to dueFlashStorage"));
      #endif


       #if defined(BOARD_MEGA)
      EEPROM.write(13, 1);
      EEPROM.write(14, PWM_MaxSpeed_LH);
      EEPROM.write(15, 1);
      EEPROM.write(16, PWM_MaxSpeed_RH);    
      EEPROM.write(94, 1);
      EEPROM.write(95, PWM_Slow_Speed_LH);
      EEPROM.write(96, 1);
      EEPROM.write(97, PWM_Slow_Speed_RH);   
      EEPROM.write(98, 1);
      EEPROM.write(99, ((Slow_Speed_MAG * -1 ) / 10));
      EEPROM.write(123, 1);
    EEPROM.write(124, Wheels_Activate);
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
  }

  
// Receive the Mower Motor Values Back again
  if (TFT_Menu_Command == 915) {
      Serial.println(F("Receiving Blade Motor Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      PWM_Blade_Speed           = Data1;
      Cutting_Blades_Activate   = Data2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;   
      Serial.print(F("Blade Motor PWM = "));
      Serial.println(PWM_Blade_Speed);
      Serial.print(F("Cutting Blade Activated = "));
      Serial.println(Cutting_Blades_Activate);

      #if defined(BOARD_DUE)
      dueFlashStorage.write(17, 1);
      dueFlashStorage.write(18, PWM_Blade_Speed);   
      dueFlashStorage.write(83 , 1);
      dueFlashStorage.write(84 , Cutting_Blades_Activate);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      EEPROM.write(17, 1);
      EEPROM.write(18, PWM_Blade_Speed);   
      EEPROM.write(83 , 1);
      EEPROM.write(84 , Cutting_Blades_Activate);
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));
  }



// Receive Movement Values from TFT
  if (TFT_Menu_Command == 916) {
      Serial.println(F("Receiving Movement Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Max_Cycles_Straight    = Data1;
      Mower_Turn_Delay_Min   = Data2;
      Mower_Turn_Delay_Max   = Data3;
      Mower_Reverse_Delay    = Data4;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;   
      Serial.print(F("Max Mow Length = "));
      Max_Cycles_Straight = Max_Cycles_Straight * 10;
      Serial.println(Max_Cycles_Straight);    
      Serial.print(F("Turn Angle Min = "));
      Mower_Turn_Delay_Min = Mower_Turn_Delay_Min * 100;
      Serial.println(Mower_Turn_Delay_Min);
      Serial.print(F("Turn Angle Max = "));
      Mower_Turn_Delay_Max = Mower_Turn_Delay_Max * 100;
      Serial.println(Mower_Turn_Delay_Max);
      Serial.print(F("Reverse Distance = "));
      Mower_Reverse_Delay = Mower_Reverse_Delay * 100;
      Serial.println(Mower_Reverse_Delay);

      #if defined(BOARD_DUE)
      //dueFlashStorage
      dueFlashStorage.write(57, 1);
      dueFlashStorage.write(58, (Max_Cycles_Straight / 10));
      dueFlashStorage.write(31, 1);
      dueFlashStorage.write(32, (Mower_Turn_Delay_Min/100));
      dueFlashStorage.write(33, 1);
      dueFlashStorage.write(34, (Mower_Turn_Delay_Max/100));
      dueFlashStorage.write(35, 1);
      dueFlashStorage.write(36, (Mower_Reverse_Delay/100));
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM
      EEPROM.write(57, 1);
      EEPROM.write(58, (Max_Cycles_Straight / 10));
      EEPROM.write(31, 1);
      EEPROM.write(32, (Mower_Turn_Delay_Min/100));
      EEPROM.write(33, 1);
      EEPROM.write(34, (Mower_Turn_Delay_Max/100));
      EEPROM.write(35, 1);
      EEPROM.write(36, (Mower_Reverse_Delay/100));
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
  }



// Receive the Mower Motor Values Back again
  if (TFT_Menu_Command == 917) {
      Serial.println(F("Receiving Track Exit Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Track_Wire_Zone_1_Cycles  = Data1 * 100;
      Track_Wire_Zone_2_Cycles  = Data2 * 100;
      CCW_Tracking_To_Start     = Data3;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;  
      Serial.print(F("Track Wire Zone 1 = "));
      Serial.println(Track_Wire_Zone_1_Cycles);      
      Serial.print(F("Track Wire Zone 2 = "));
      Serial.println(Track_Wire_Zone_2_Cycles);      
      if (CCW_Tracking_To_Start == 0) {
        CW_Tracking_To_Start = 1;
        CCW_Tracking_To_Charge = 1;
        CW_Tracking_To_Charge = 0;
        Serial.println(F("Tracking to Charge = CCW"));
        Serial.println(F("Tracking to Start = CW"));
        }       
      if (CCW_Tracking_To_Start == 1) {
        CW_Tracking_To_Start = 0;
        CCW_Tracking_To_Charge = 0;
        CW_Tracking_To_Charge = 1;        
        Serial.println(F("Tracking to Charge = CW"));
        Serial.println(F("Tracking to Start = CCW"));
        }        

      #if defined(BOARD_DUE)
      //dueFlashStorage
      dueFlashStorage.write(43, 1);
      dueFlashStorage.write(44, (Track_Wire_Zone_1_Cycles/100));
      dueFlashStorage.write(45, 1);
      dueFlashStorage.write(46, (Track_Wire_Zone_2_Cycles/100));
      dueFlashStorage.write(49 , 1);
      dueFlashStorage.write(50 , CW_Tracking_To_Charge);                
      dueFlashStorage.write(51 , 1);
      dueFlashStorage.write(52 , CCW_Tracking_To_Charge);       
      dueFlashStorage.write(53 , 1);
      dueFlashStorage.write(54 , CW_Tracking_To_Start);                
      dueFlashStorage.write(55 , 1);
      dueFlashStorage.write(56 , CCW_Tracking_To_Start);  
      Serial.println(F("Saved to dueFlashStorage"));
      #endif


      #if defined(BOARD_MEGA)
      //EEPROM
      EEPROM.write(43, 1);
      EEPROM.write(44, (Track_Wire_Zone_1_Cycles/100));
      EEPROM.write(45, 1);
      EEPROM.write(46, (Track_Wire_Zone_2_Cycles/100));
      EEPROM.write(49 , 1);
      EEPROM.write(50 , CW_Tracking_To_Charge);                
      EEPROM.write(51 , 1);
      EEPROM.write(52 , CCW_Tracking_To_Charge);       
      EEPROM.write(53 , 1);
      EEPROM.write(54 , CW_Tracking_To_Start);                
      EEPROM.write(55 , 1);
      EEPROM.write(56 , CCW_Tracking_To_Start);  
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
  }


// Receive the Mower Motor Values Back again
  if (TFT_Menu_Command == 918) {
      Serial.println(F("Receiving Find Wire Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Max_Cycle_Wire_Find       = Data1 * 100;
      Max_Cycle_Wire_Find_Back  = Data2 * 10;
      Home_Wire_Compass_Heading = Data3;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;   
      Serial.print(F("Wire Find Forwards / Cycles = "));
      Serial.println(Max_Cycle_Wire_Find);
      Serial.print(F("Wire Find Forwards / Cycles = "));
      Serial.println(Max_Cycle_Wire_Find_Back);    
      Serial.print(F("Home Compass Heading / degrees "));
      Serial.println(Home_Wire_Compass_Heading);  

       #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(69, 1);
      dueFlashStorage.write(70, (Max_Cycle_Wire_Find/10));
      dueFlashStorage.write(71, 1);
      dueFlashStorage.write(72, (Max_Cycle_Wire_Find_Back/10));
      dueFlashStorage.write(27, 1);
      dueFlashStorage.write(28, (Home_Wire_Compass_Heading/10));   
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

       #if defined(BOARD_MEGA)
      // EEPROM
      EEPROM.write(69, 1);
      EEPROM.write(70, (Max_Cycle_Wire_Find/10));
      EEPROM.write(71, 1);
      EEPROM.write(72, (Max_Cycle_Wire_Find_Back/10));
      EEPROM.write(27, 1);
      EEPROM.write(28, (Home_Wire_Compass_Heading/10));   
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
  }


// Charging Station Options
  if (TFT_Menu_Command == 98) {
      Serial.println(F("Charging Station"));
      Receive_7_Data_Sets_Serial3();
      Use_Charging_Station = Data1;
      Data2 = 0;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Use Charging Station = "));
      Serial.println(Use_Charging_Station);   

      #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(47 , 1);
      dueFlashStorage.write(48 , Use_Charging_Station);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      // EEPROM
      EEPROM.write(47 , 1);
      EEPROM.write(48 , Use_Charging_Station);
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
      }



// Receive the Mower Start Conditions from the TFT Screen
  if (TFT_Menu_Command == 93) {
      Serial.println(F("RX Mower Exit Dock Start"));
      Receive_7_Data_Sets_Serial3();
      Exit_Zone     = Data1;
      Mow_Time_Set  = Data2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Exit Zone Selected = "));
      Serial.println(Exit_Zone);
      Serial.print(F("Mow Time Selected = "));
      if ( (Mow_Time_Set == 1) || (Mow_Time_Set == 2 ) ){
          Serial.print(Mow_Time_Set);
          Serial.println(F(" hrs"));
          }
      if (Mow_Time_Set == 3) Serial.println(F("Max Mow Time"));
      Serial.println(F(" "));

    if (Exit_Zone == 1) Track_Wire_Itterations = Track_Wire_Zone_1_Cycles;
    if (Exit_Zone == 2) Track_Wire_Itterations = Track_Wire_Zone_2_Cycles;
    if (Exit_Zone == 3) {
      Track_Wire_Itterations = Track_Wire_Zone_2_Cycles;
      Blade_Override = 1;
    }


    if (Mow_Time_Set ==3) Alarm_Timed_Mow_ON = 0;
    if (Mow_Time_Set < 3) {
        if (PCB == 0) {
          Time t = rtc.time();
          }
//        if (PCB == 1) Display_DS3231_Time();
//        Alarm_Timed_Mow_ON = 1;                          // Activate the Mow Timer Alarm
//        Alarm_Timed_Mow_Hour = Time_Hour +  Mow_Time_Set;     // Sets time to (+Mow_Time_Set 1 or 2 )hour later.
//        Alarm_Timed_Mow_Minute = Time_Minute;                  // Minutes are the same
        }
    
    TFT_Menu_Command = 1; 

    Serial.println("In TFT RX Function");
    Check_Mower_Status();
    Manouver_Exit_To_Zone_X();                           // Exit the Dock                
    }


// Receive PID Values
  if (TFT_Menu_Command == 919) {
      Serial.println(F("RX PID Values from TFT"));
      Receive_7_Data_Sets_Serial3();
      Max_Tracking_Turn_Left  = Data1 * 10;
      Max_Tracking_Turn_Right = Data2 * 10;
      P                       = Data3;
      P                       = P / 100;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Max Cycles Left Wheel = "));
      Serial.println(Max_Tracking_Turn_Left);      
      Serial.print(F("Max Cycles Right Wheel = "));
      Serial.println(Max_Tracking_Turn_Right);               
      Serial.print(F("PID P = "));
      Serial.println(P);
      
      #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(73, 1);
      dueFlashStorage.write(74, (Max_Tracking_Turn_Right/10));
      dueFlashStorage.write(75, 1);
      dueFlashStorage.write(76, (Max_Tracking_Turn_Left/10));
      dueFlashStorage.write(21, 1);
      dueFlashStorage.write(22, (P*100));  
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      // EEPROM
      EEPROM.write(73, 1);
      EEPROM.write(74, (Max_Tracking_Turn_Right/10));
      EEPROM.write(75, 1);
      EEPROM.write(76, (Max_Tracking_Turn_Left/10));
      EEPROM.write(21, 1);
      EEPROM.write(22, (P*100));  
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));
      }


// Receive the Compass data from TFT
  if (TFT_Menu_Command == 926) {
      Serial.println(F("RX Compass from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Compass_Activate              = Data1;
      Compass_Heading_Hold_Enabled  = Data2;
      CPower                        = Data3;
      CPower                        = CPower / 10;
      Compass_Setup_Mode            = Data4;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Compass Activated = "));
      Serial.println(Compass_Activate);
      Serial.print(F("Heading Hold Enabled = "));
      Serial.println(Compass_Heading_Hold_Enabled);        
      Serial.print(F("Compass Power = "));
      Serial.println(CPower);
      Serial.print(F("Compas Setup Mode = "));
      Serial.println(Compass_Setup_Mode); 

      #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(19 , 1);
      dueFlashStorage.write(20 , Compass_Activate);
      dueFlashStorage.write(59 , 1);
      dueFlashStorage.write(60 , Compass_Heading_Hold_Enabled);
      dueFlashStorage.write(61, 1);
      dueFlashStorage.write(62, (CPower*10)); 
      dueFlashStorage.write(113, 1);
      dueFlashStorage.write(114, Compass_Setup_Mode); 
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      // EEPROM
      EEPROM.write(19 , 1);
      EEPROM.write(20 , Compass_Activate);
      EEPROM.write(59 , 1);
      EEPROM.write(60 , Compass_Heading_Hold_Enabled);
      EEPROM.write(61, 1);
      EEPROM.write(62, (CPower*10)); 
      EEPROM.write(113, 1);
      EEPROM.write(114, Compass_Setup_Mode); 
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));
      }


// Receive the Compass data from TFT
  if (TFT_Menu_Command == 927) {
      Serial.println(F("RX Compass from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      GPS_Enabled                   = Data1;
      GPS_Type                      = Data2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("GPS: "));
        if (GPS_Enabled == 1) Serial.println("ON");
        if (GPS_Enabled == 0) Serial.println("OFF");    
      Serial.print(F("GPS Type: "));
        if (GPS_Type == 1) Serial.println("ReP_AL");
        if (GPS_Type == 2) {
          Serial.println("PIXHAWK");    
          Setup_PIXHAWK();
          }
        if (GPS_Type == 3) Serial.println("Spare");   

        

      #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(107 , 1);
      dueFlashStorage.write(108 , GPS_Enabled);
      dueFlashStorage.write(125 , 1);
      dueFlashStorage.write(126 , GPS_Type);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      // EEPROM
      EEPROM.write(107 , 1);
      EEPROM.write(108 , GPS_Enabled);
      EEPROM.write(125 , 1);
      EEPROM.write(126 , GPS_Type);
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));
      }



// Receive the GYRO data from TFT
  if (TFT_Menu_Command == 928) {
      Serial.println(F("RX GYRO from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      GYRO_Enabled  = Data1;
      GPower        = Data2;
      GPower        = GPower / 10;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("GYRO Enabled = "));
      Serial.println(GYRO_Enabled);
      Serial.print(F("GYRO Power = "));
      Serial.println(GPower);      
      if (GYRO_Enabled == 1) Setup_Gyro();

      #if defined(BOARD_DUE)
      dueFlashStorage.write(109 , 1);
      dueFlashStorage.write(110, GYRO_Enabled);
      dueFlashStorage.write(111, 1);
      dueFlashStorage.write(112, (GPower*10)); 
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      EEPROM.write(109 , 1);
      EEPROM.write(110, GYRO_Enabled);
      EEPROM.write(111, 1);
      EEPROM.write(112, (GPower*10)); 
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" ")); 
      }



// Receive the Time Alarm data from TFT
  if (TFT_Menu_Command == 920) {
      Serial.println(F("RX Alarm 1 TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Alarm_1_ON     = Data1;
      Alarm_1_Hour   = Data2;
      Alarm_1_Minute = Data3;
      Alarm_1_Repeat = Data4;
      Alarm_1_Action = Data5;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Alarm 1 ON = "));
      Serial.println(Alarm_1_ON);
      Serial.print(F("Alarm 1 Time = "));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      Serial.println(Alarm_1_Minute);
      Serial.print(F("Alarm 1 Repeat = "));
      Serial.println(Alarm_1_Repeat);
      Serial.print(F("Alarm 1 Action = "));
      Serial.println(Alarm_1_Action); 

      #if defined(BOARD_DUE)
      //dueFlashStorage
      dueFlashStorage.write(1, 1);
      dueFlashStorage.write(2, Alarm_1_Hour);
      dueFlashStorage.write(3, Alarm_1_Minute);
      dueFlashStorage.write(4, Alarm_1_ON);
      dueFlashStorage.write(87, Alarm_1_Action);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM
      EEPROM.write(1, 1);
      EEPROM.write(2, Alarm_1_Hour);
      EEPROM.write(3, Alarm_1_Minute);
      EEPROM.write(4, Alarm_1_ON);
      EEPROM.write(87, Alarm_1_Action);
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));    
  }




// Receive the Alarm 2 data from TFT
  if (TFT_Menu_Command == 921) {
      Serial.println(F("RX Alarm 2 TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Alarm_2_ON     = Data1;
      Alarm_2_Hour   = Data2;
      Alarm_2_Minute = Data3;
      Alarm_2_Repeat = Data4;
      Alarm_2_Action = Data5;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Alarm 2 ON = "));
      Serial.println(Alarm_2_ON);
      Serial.print(F("Alarm 2 Time = "));
      Serial.print(Alarm_2_Hour);
      Serial.print(F(":"));
      Serial.println(Alarm_2_Minute);
      Serial.print(F("Alarm 2 Repeat = "));
      Serial.println(Alarm_2_Repeat);      
      Serial.print(F("Alarm 2 Action = "));
      Serial.println(Alarm_2_Action); 

      #if defined(BOARD_DUE)
      //dueFlashStorage
      dueFlashStorage.write(5, 1);
      dueFlashStorage.write(6, Alarm_2_Hour);
      dueFlashStorage.write(7, Alarm_2_Minute);
      dueFlashStorage.write(8, Alarm_2_ON);
      dueFlashStorage.write(88, Alarm_2_Action);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM
      EEPROM.write(5, 1);
      EEPROM.write(6, Alarm_2_Hour);
      EEPROM.write(7, Alarm_2_Minute);
      EEPROM.write(8, Alarm_2_ON);
      EEPROM.write(88, Alarm_2_Action);
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));    
      }


// Receive the Alarm 3 data from TFT
  if (TFT_Menu_Command == 922) {
      Serial.println(F("RX Alarm 3 TFT ..."));
      Receive_7_Data_Sets_Serial3();
      Alarm_3_ON     = Data1;
      Alarm_3_Hour   = Data2;
      Alarm_3_Minute = Data3;
      Alarm_3_Repeat = Data4;
      Alarm_3_Action = Data5;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Alarm 3 ON = "));
      Serial.println(Alarm_3_ON);
      Serial.print(F("Alarm 3 Time = "));
      Serial.print(Alarm_3_Hour);
      Serial.print(F(":"));
      Serial.println(Alarm_3_Minute);
      Serial.print(F("Alarm 3 Repeat = "));
      Serial.println(Alarm_3_Repeat);
      Serial.print(F("Alarm 3 Action = "));
      Serial.println(Alarm_3_Action); 

      #if defined(BOARD_DUE)
      //dueFlashStorage
      dueFlashStorage.write(9, 1);
      dueFlashStorage.write(10, Alarm_3_Hour);
      dueFlashStorage.write(11, Alarm_3_Minute);
      dueFlashStorage.write(12, Alarm_3_ON);
      dueFlashStorage.write(89, Alarm_3_Action);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM
      EEPROM.write(9, 1);
      EEPROM.write(10, Alarm_3_Hour);
      EEPROM.write(11, Alarm_3_Minute);
      EEPROM.write(12, Alarm_3_ON);
      EEPROM.write(89, Alarm_3_Action);
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));    
      }



// Receive the Time data from TFT
  if (TFT_Menu_Command == 923) {
      Serial.println(F("RX DateTime from TFT ..."));
      String Serial3_RX_Value  = "";          
      int set_hour;
      int set_min;
      int set_year;
      int set_month;
      int set_date;
      int set_day;
      Receive_7_Data_Sets_Serial3();
      set_hour   = Data1;
      set_min    = Data2;
      set_year   = Data3;
      set_month  = Data4;
      set_date   = Data5;
      set_day    = Data6;
      Data7 = 0; 

       if (PCB == 0) {           
          rtc.writeProtect(false);
          rtc.halt(false);
          /*
          Serial.print(F("Clock: "));
          Serial.print(set_hour);
          Serial.print(":");
          if (set_min < 10) Serial.print("0");
          Serial.println(set_min);
          Serial.print(F("Date: "));
      Serial.print(set_year);
      Serial.print("-");
      if (set_month < 10) Serial.print("0");
      Serial.print(set_month);
      Serial.print("-");
      if (set_date < 10) Serial.print("0");
      Serial.print(set_date);
      Serial.print(F(" "));
        char buf[15];
        const String day = dayAsString((Time::Day)set_day);
        snprintf(buf, sizeof(buf), " %s", day.c_str());
      Serial.println(buf);
*/
          PrintTimeToSerial(2, set_year, set_month, set_date, set_hour, set_min, 00, set_day, 1);

          Time t(set_year, set_month, set_date, set_hour, set_min, 00, (Time::Day)set_day);            // Year XXXX, Month XX, Day XX, Hour XX, Minute XX, Second, kXYZday
          rtc.time(t);    
          delay(200);
          rtc.writeProtect(true);
          rtc.halt(true);
          rtc.time(t); 
          }

      if (PCB == 1) {
          Set_DS3231_Time(00,set_min, set_hour, set_day, set_date, set_month, set_year);    //second, minute, hour, dayof week, day of month, month, year
          Serial.println(F("TIME SAVED"));
          delay(2000);
          }
  
  }



  if (TFT_Menu_Command == 929) {
        Serial.println(F("RX Wheel Block"));                               
        Receive_7_Data_Sets_Serial3();
        Wheel_Amp_Sensor_ON   = Data1;
        Max_Wheel_Amps        = Data2;                         // Directly place Data 2 into value to ensure float is not lost
        Max_Wheel_Amps        = Max_Wheel_Amps / 100;
        Data3 = 0;
        Data4 = 0;
        Data5 = 0;
        Data6 = 0;
        Data7 = 0;   
        Serial.print(F("Wheel Amp ON: "));
        if (Wheel_Amp_Sensor_ON == 1) Serial.println(F("ON"));
        if (Wheel_Amp_Sensor_ON == 0) Serial.println(F("OFF"));   
        Serial.print(F("Wheel Amps Max: "));
        Serial.print(Max_Wheel_Amps);

        #if defined(BOARD_DUE)
        //dueFlashStorage Saved Values
        dueFlashStorage.write(115, 1);
        dueFlashStorage.write(116, Wheel_Amp_Sensor_ON);
        dueFlashStorage.write(117, 1);
        dueFlashStorage.write(118, Max_Wheel_Amps * 10);
        Serial.println(F("Saved to dueFlashStorage"));
        #endif

        #if defined(BOARD_MEGA)
        //EEPROM Saved Values
        EEPROM.write(115, 1);
        EEPROM.write(116, Wheel_Amp_Sensor_ON);
        EEPROM.write(117, 1);
        EEPROM.write(118, Max_Wheel_Amps * 10);
        Serial.println(F("Saved to EEPROM"));
        #endif
        
        Serial.println(F(" "));  
       }


  if (TFT_Menu_Command == 924) {
      Serial.println(F("RX Tip from TFT ..."));                               
      Receive_7_Data_Sets_Serial3();
      Angle_Sensor_Enabled    = Data1;
      Tip_Over_Sensor_Enabled = Data2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;   
      Serial.print(F("Tilt Angle Sensor = "));
      Serial.println(Angle_Sensor_Enabled);
      Serial.print(F("Tip Over Sensor = "));
      Serial.println(Tip_Over_Sensor_Enabled);
      Serial.println(F(" "));
      
      #if defined(BOARD_DUE)
      //dueFlashStorage Saved Values
      dueFlashStorage.write(29, 1);
      dueFlashStorage.write(30, Angle_Sensor_Enabled);
      dueFlashStorage.write(92, 1);
      dueFlashStorage.write(93, Tip_Over_Sensor_Enabled);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM Saved Values
      EEPROM.write(29, 1);
      EEPROM.write(30, Angle_Sensor_Enabled);
      EEPROM.write(92, 1);
      EEPROM.write(93, Tip_Over_Sensor_Enabled);
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));  
      }



// Receive the Pattern Values Back again
  if (TFT_Menu_Command == 925) {
      Serial.println(F("RX Pattern from TFT"));
      Receive_7_Data_Sets_Serial3();
      Pattern_Mow             = Data1;
      Turn_90_Delay_LH        = Data2 * 10;
      Turn_90_Delay_RH        = Data3 * 10;
      Move_to_next_line_delay = Data4 * 10;
      Line_Length_Cycles      = Data5;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("Pattern Mow: "));
      if (Pattern_Mow == 0) Serial.println(F("OFF"));
      if (Pattern_Mow == 1) Serial.println(F("Parallel"));  
      if (Pattern_Mow == 2) Serial.println(F("Spiral"));      
      Serial.print(F("Turn 90° LH= "));
      Serial.println(Turn_90_Delay_LH);
      Serial.print(F("Turn 90° RH= "));
      Serial.println(Turn_90_Delay_RH);
      Serial.print(F("Distance to next row= "));
      Serial.println(Move_to_next_line_delay);          
      Serial.print(F("Line_Length_Cycles= "));
      Serial.println(Line_Length_Cycles); 
      
      #if defined(BOARD_DUE)
      //dueFlashStorage Saved Values
      dueFlashStorage.write(23, 1);
      dueFlashStorage.write(24, Pattern_Mow);
      dueFlashStorage.write(101, 1);
      dueFlashStorage.write(102, Turn_90_Delay_LH / 10);
      dueFlashStorage.write(103, 1);
      dueFlashStorage.write(104, Turn_90_Delay_RH / 10);
      dueFlashStorage.write(105, 1);
      dueFlashStorage.write(106, Line_Length_Cycles / 10);
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM Saved Values
      EEPROM.write(23, 1);
      EEPROM.write(24, Pattern_Mow);
      EEPROM.write(101, 1);
      EEPROM.write(102, Turn_90_Delay_LH / 10);
      EEPROM.write(103, 1);
      EEPROM.write(104, Turn_90_Delay_RH / 10);
      EEPROM.write(105, 1);
      EEPROM.write(106, Line_Length_Cycles / 10);
      Serial.println(F("Saved to EEPROM"));
      #endif

      
      Serial.println(F(" "));  
  }



// Receive the Other SETUP Values from the TFT
  if (TFT_Menu_Command == 930) {
      Serial.println(F("RX PCB from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      PCB               = Data1;
      Robot_Type        = Data2;
      int Clear_EEPROM  = Data3;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("PCB = "));
      if (PCB == 1) Serial.println(F("Enabled"));
      if (PCB == 0) Serial.println(F("Disabled"));
      if (Robot_Type == 1) Serial.println(F("Robot = Mower"));
//      if (Robot_Type == 2) Serial.println(F("Robot = Aerator"));
     
      
      #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(119 , 1);
      dueFlashStorage.write(120, PCB);      
      dueFlashStorage.write(121 , 1);
      dueFlashStorage.write(122, Robot_Type);     
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      // EEPROM      
      EEPROM.write(119 , 1);
      EEPROM.write(120, PCB);      
      EEPROM.write(121 , 1);
      EEPROM.write(122, Robot_Type);    
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));

      if (Clear_EEPROM == 5) {
        Serial.println(F(""));
        Serial.println(F(""));
        Serial.println(F("CLEARING ALL EEPROM MEGA SETTINGS"));
        Serial.println(F(""));
        Serial.println(F(""));
        Clear_EERPOM_Data();
        Clear_EEPROM = 0;
      }
      
      Serial.println(F(" "));

}



// Receive the Navigation data from TFT
  if (TFT_Menu_Command == 299) {
      Serial.println(F("Receiving Navigation Values from TFT ..."));
      Receive_7_Data_Sets_Serial3();
      GPS_Enabled             = Data1;
      Data2 = 0;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.print(F("GPS Enabled = "));
      Serial.println(GPS_Enabled);
      if ((GPS_Enabled == 1) && (GPS_Type == 1)) Setup_ADCMan();
      if ((GPS_Enabled == 1) && (GPS_Type == 2)) Setup_PIXHAWK();
      Serial.println(" ");
      
      #if defined(BOARD_DUE)
      // dueFlashStorage
      dueFlashStorage.write(107 , 1);
      dueFlashStorage.write(108, GPS_Enabled);     
      Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      // EEPROM
      EEPROM.write(107 , 1);
      EEPROM.write(108, GPS_Enabled);     
      Serial.println(F("Saved to EEPROM"));
      #endif
      
      Serial.println(F(" "));
      }
  

// Receive the Mower Start Conditions from the TFT Screen
  if (TFT_Menu_Command == 92) {
      Serial.println(F("RX Mower Start Values"));
      Receive_7_Data_Sets_Serial3();
      Mow_Time_Set = Data1;
      Data2 = 0;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0; 
      Serial.println(F("Quick Start Selected"));
      Serial.print(F("Mow Time Selected = "));
      if ( (Mow_Time_Set == 1) || (Mow_Time_Set == 2 ) ){
          Serial.print(Mow_Time_Set);
          Serial.println(" hrs");
          }
      if (Mow_Time_Set == 3) Serial.println(F("Max Mow Time"));

      Serial.println(F(" "));

    if (Mow_Time_Set == 3) Alarm_Timed_Mow_ON = 0;
    if (Mow_Time_Set < 3) {
        if (PCB == 0) {Time t = rtc.time();}
//        if (PCB == 1) Display_DS3231_Time();
//        Alarm_Timed_Mow_ON = 1;                          // Activate the Mow Timer Alarm
//        Alarm_Timed_Mow_Hour = Time_Hour +  Mow_Time_Set;     // Sets time to (+Mow_Time_Set 1 or 2 )hour later.
//        Alarm_Timed_Mow_Minute = Time_Minute;                  // Minutes are the same
        }
           
    Serial.print(F("Mow Time Ends: "));
    if (Alarm_Timed_Mow_Hour > 23) Alarm_Timed_Mow_Hour = Alarm_Timed_Mow_Hour - 24;
    if (Alarm_Timed_Mow_Hour < 10) Serial.print(F("0"));
    Serial.print(Alarm_Timed_Mow_Hour);
    Serial.print(F(":"));
    if (Alarm_Timed_Mow_Minute < 10) Serial.print(F("0"));
    Serial.print(Alarm_Timed_Mow_Minute);       
    
    TFT_Menu_Command = 1; 
    Manouver_Start_Mower();                              // Quick Start
                        
    }

// Receive the Mower Conditions from the TFT Screen after Goto Dock 
// is pressed

  if (TFT_Menu_Command == 94) {
      Serial.println(F("TFT Says Go To Dock"));
      delay(1100);
      String Serial3_RX_Value  = "";                                              
      TFT_Menu_Command = 1; 
      Mower_Parked  = 0;  
      Mower_Running = 1;                              // As the TFT Menu is only hiding the running status in the background.
      Check_Mower_Status();
      if (Use_Charging_Station == 1) Manouver_Go_To_Charging_Station();  // Go to Dock                     
      }




// Start Relay Test
  if (TFT_Menu_Command == 42) {
    Serial.println(F("Running Relay Test"));
    Test_Relay();
    }

// Start Wheel Motor Test
  if (TFT_Menu_Command == 43) {
    Serial.println(F("Running Wheel Test"));
    Test_Wheel_Motors();

    }

// Start Wheel Amp Test
  if (TFT_Menu_Command == 38) {
    Serial.println(F("Running Wheel Amp Test"));
    Test_Wheel_Amps();
    }

// Start Motor Test
  if (TFT_Menu_Command == 44) {
    
    if (Robot_Type == 1) {
      Serial.println(F("Running Blade Motor Test"));
      Test_Mower_Blade_Motor();
      }
    
//    if (Robot_Type == 2) {
//      Serial.println(F("Running Drill Motor Test"));
//      Turn_On_Relay();
//      delay(7000);
//      Motor_Action_Spin_Drill();
//      Drill_ON = 1;
//      delay(3000);
//      Motor_Action_Stop_Drill();      
//      delay(1000);
//      Turn_Off_Relay();
//      Drill_ON = 0;
//      }
   
    }

// PIXHAWK Wheel Motor Test
  if (TFT_Menu_Command == 64) {
    Serial.println(F("PIXHAWK Motor Test"));

    Pixhawk_Motor_Test_Initiate = 0;
    Turn_On_Relay();
    SetPins_ToGoForwards();
    delay(1000);
    cycles = 0;

      
    while (cycles < 500) {
      cycles ++;
      Serial.print("Cycles:");
      Serial.print(cycles);  
      Serial.print(" | ");
      Check_PIXHAWK_PWM();
      Send_PIXHAWK_Running_Data();    
      Serial.println("");    
      }

    Serial.println(F("Test Completed"));
    Turn_Off_Relay();
    Command = 0;
    }

// PIXHAWK Disarm
  if (TFT_Menu_Command == 31) {
    Serial.println("");
    Serial.println(F("Mower set to PIXHAWK Mode"));
    Serial.println("");
    Mower_Docked = 0;
    Mower_PIXHAWK = 1; 
  }


// PIXHAWK Disarm
  if (TFT_Menu_Command == 74) {
    Serial.println("");
    Serial.println("DIS-ARMING PIXHAWK");
    Serial.println("");
    Command_long_Disarm();    
  }


// PIXHAWK ARM
  if (TFT_Menu_Command == 75) {
    Serial.println("");
    Serial.println("ARMING PIXHAWK");
    Serial.println("");
    Command_long_ARM();    
  }

// PIXHAWK Guided
  if (TFT_Menu_Command == 76) {
    Serial.println("");
    Serial.println("PIXHAWK - Guided");
    Serial.println("");
    Guided_Mode();   
  }

// PIXHAWK Auto
  if (TFT_Menu_Command == 77) {
    Serial.println("");
    Serial.println("PIXHAWK - Auto");
    Serial.println("");
    Auto_Mode();    
  }

// PIXHAWK Acro
  if (TFT_Menu_Command == 78) {
    Serial.println("");
    Serial.println("PIXHAWK - Acro");
    Serial.println("");
    Acro_Mode();    
  }

// PIXHAWK EXIT
  if (TFT_Menu_Command == 79) {
    Serial.println("");
    Serial.println("EXIT PIXHAWK Mode");
    Command_long_Disarm();
    Turn_Off_Relay();
    Serial.println(F("Mower set to ReP_AL Mode"));
    Serial.println("");
    Mower_Docked = 1;
    Mower_PIXHAWK = 0; 
    Serial.println("");

  }
  
// Receive the Stop Mower Command Red Button
  if (TFT_Menu_Command == 13) {
      Manouver_Park_The_Mower();
      TFT_Menu_Command = 1;
//      if (Robot_Type == 2) Stop = 1;   // Aborts the drill sequence
      }



// Receive the Mower Start Conditions from the TFT Screen
  if (TFT_Menu_Command == 49) {
      Serial.println(F("TFT Says Go To Dock"));
      delay(1100);
      String Serial3_RX_Value  = "";                                              
    TFT_Menu_Command = 1; 
    if (Use_Charging_Station == 1) Manouver_Go_To_Charging_Station();  // Go to Dock
                        
    }

// Test Left Front Wheel Forwards Motion
  if (TFT_Menu_Command == 66) {
    Serial.println(F("LH Front Wheel FWD"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    Set_Mecanum_Forwards_Left_Front();
    Full_Speed_Mecanum_Left_Front();
    delay(2000);
    Motor_Action_Stop_Motors();  
    }


// Test Left Front Wheel backwards Motion
  if (TFT_Menu_Command == 67) {
    Serial.println(F("LH Front Wheel REV"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    Set_Mecanum_Backwards_Left_Front();
    Full_Speed_Mecanum_Left_Front();
    delay(2000);
    Motor_Action_Stop_Motors();  
    }

// Test Right Front Wheel Forwards Motion
  if (TFT_Menu_Command == 68) {
    Serial.println(F("RH Front Wheel FWD"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    Set_Mecanum_Forwards_Right_Front();
    Full_Speed_Mecanum_Right_Front();
    delay(2000);
    Motor_Action_Stop_Motors();  
    }


// Test Right Front Wheel backwards Motion
  if (TFT_Menu_Command == 69) {
    Serial.println(F("RH Front Wheel REV"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    Set_Mecanum_Backwards_Right_Front();
    Full_Speed_Mecanum_Right_Front();
    delay(2000);
    Motor_Action_Stop_Motors();  
    }


// Test Left Rear Wheel Forwards Motion
  if (TFT_Menu_Command == 70) {
    Serial.println(F("LH Rear Wheel FWD"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    
    if (Robot_Type == 1) {
      Set_Mower_Forwards_Left();
      Motor_Action_Go_Full_Speed();
      }   
    
//    if (Robot_Type == 2) {
//      Set_Mecanum_Forwards_Left_Rear();
//      Full_Speed_Mecanum_Left_Rear();
//      }
    delay(2000);
    Motor_Action_Stop_Motors();  
    }


// Test Left Rear Wheel Backwards Motion
  if (TFT_Menu_Command == 71) {
    Serial.println(F("LH Rear Wheel REV"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    
    if (Robot_Type == 1) {
      Set_Mower_Backwards_Left();
      Motor_Action_Go_Full_Speed();
      }   
    
//    if (Robot_Type == 2) {
//      Set_Mecanum_Backwards_Left_Rear();
//      Full_Speed_Mecanum_Left_Rear();
//      }
    delay(2000);
    Motor_Action_Stop_Motors();  
    }


// Test Right Rear Wheel Forwards Motion
  if (TFT_Menu_Command == 72) {
    Serial.println(F("RH Rear Wheel FWD"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    if (Robot_Type == 1) {
      Set_Mower_Forwards_Right();
      Motor_Action_Go_Full_Speed();
      }   
    
//    if (Robot_Type == 2) {
//      Set_Mecanum_Forwards_Right_Rear();
//      Full_Speed_Mecanum_Right_Rear();
//      }
    delay(2000);
    Motor_Action_Stop_Motors();  
    }


// Test Right Rear Wheel Backwards Motion
  if (TFT_Menu_Command == 73) {
    Serial.println(F("RH Rear Wheel REV"));
    Turn_On_Relay();
    Motor_Action_Stop_Motors();
    
    if (Robot_Type == 1) {
      Set_Mower_Backwards_Right();
      Motor_Action_Go_Full_Speed();
      }   
//    if (Robot_Type == 2) {
//      Set_Mecanum_Backwards_Right_Rear();
//      Full_Speed_Mecanum_Right_Rear();
//      }

    delay(2000);
    Motor_Action_Stop_Motors();  
    }

// Exit and turn off relay
  if (TFT_Menu_Command == 80) {
    Serial.println(F("Motor Test Finished"));
    Turn_Off_Relay();
    Motor_Action_Stop_Motors();  
    }
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// TFT_TX
// Transmits 7 Data Values to the TFT MEGA on Serial3 Com Port.
// The data values are declared below for the different Menu
// Commands.  This standardises the data transfer method for all
// functions.
//
// Values are transfered from the Mower MEGA to the TFT MEGA for editing in
// the TFT GUI.  The values are then sent back and stored to dueFlashStorage on the MOWER
// MEGA so the mower can carry out the instructions according to the user settings


void TX_7_Data_Values() {
      Serial.println("Sending Data Sets");
  byte data[15];
  data[0] = 18; // Packet ID

  data[1]=Data1;
  data[2]=Data1 >> 8;
  data[3]=Data2;
  data[4]=Data2 >> 8;
  data[5]=Data3;
  data[6]=Data3 >> 8;
  data[7]=Data4;
  data[8]=Data4 >> 8;
  data[9]=Data5;
  data[10]=Data5 >> 8;
  data[11]=Data6;
  data[12]=Data6 >> 8;
  data[13]=Data7;
  data[14]=Data7 >> 8;

    SerialCom3.sendMsg (data, sizeof (data));

    //Check_TX_Received_PacketNum(17);

      Serial.print(F("Data1:"));
      Serial.println(Data1);
      Serial.print(F("Data2:"));
      Serial.println(Data2);
      Serial.print(F("Data3:"));
      Serial.println(Data3);  
      Serial.print(F("Data4:"));
      Serial.println(Data4);
      Serial.print(F("Data5:"));
      Serial.println(Data5);
      Serial.print(F("Data6:"));
      Serial.println(Data6);
      Serial.print(F("Data7:"));
      Serial.println(Data7);    
      }


void Send_Data_To_TFT() {
    // Depending on the code received the following data set is sent through the Serial 3 port.
    
    // Menu Sonar
    if (TFT_Menu_Command == 10) {
      Serial.println("TX Sonar to TFT");
      Data1 = Sonar_1_Activate;
      Data2 = Sonar_2_Activate;
      Data3 = Sonar_3_Activate;
      Data4 = maxdistancesonar;
      Data5 = Max_Sonar_Hit;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();

      Serial.print(F("S1 ON = "));
      Serial.println(Sonar_1_Activate);
      Serial.print(F("S2 ON = "));
      Serial.println(Sonar_2_Activate);
      Serial.print(F("S3 ON = "));
      Serial.println(Sonar_3_Activate);
      Serial.print(F("Sonar Max Distance = "));
      Serial.println(maxdistancesonar);
      Serial.print(F("Sonar Sensitivity = "));
      Serial.println(Max_Sonar_Hit);
      Serial.println(F(" "));
      }

    
// 11 is sometimes read if a serial port miscommunication occurs therefore this
// number is not used to start communication from the MEGA:
    if (TFT_Menu_Command == 11) {
      Serial.println("False Command");      
      }


// Navigation Menu is selected on the TFT
    if (TFT_Menu_Command == 9) {
      Serial.println("TX Nav to TFT");
      
      Serial3.print(GPS_Enabled);
      Serial3.println("\a");
      delay(200);  

      Serial.print(F("GPS Enabled = "));
      Serial.println(GPS_Enabled);   
      }


// Battery Menu is selected on the TFT
    if (TFT_Menu_Command == 26) {
      Serial.println("TX Battery to TFT");
      float Temp = Battery_Min * 10;          // Create Temp float value to transfer the decimal to the int
      int Temp2 = Temp;
      Data1 = Temp2;
      Data2 = Low_Battery_Instances_Chg;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      Serial.print(F("Battery Min = "));
      Serial.println(Battery_Min);
      Serial.print(F("Battery Sensitivity = "));
      Serial.println(Low_Battery_Instances_Chg);
      Serial.println(" ");
      }

// Setup Other Menu is selected on the TFT
    if (TFT_Menu_Command == 30) {
      Serial.println("Setup Other");
      Data1 = PCB;
      Data2 = Robot_Type;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      Serial.print(F("PCB = "));
      if (PCB == 1) Serial.println(F("Enabled"));
      if (PCB == 0) Serial.println(F("Disabled"));
      }


      
// Perimeter Wire Menu is selected on the TFT
    if (TFT_Menu_Command == 6) {
      Serial.println("TX Perimeter to TFT");
      Data1 = Perimeter_Wire_Enabled;
      Data2 = WIFI_Enabled;
//      Data3 = Bumper_Activate_Frnt;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      Serial.print(F("Wire Sensor ON = "));
      Serial.println(Perimeter_Wire_Enabled);
      Serial.print(F("WIFI Enabled = "));
      Serial.println(WIFI_Enabled);
//      Serial.print(F("Bumper ON = "));
//      Serial.println(Bumper_Activate_Frnt);
      Serial.println(F(" "));
      }


// Rain Sensor Menu is selected on the TFT
    if (TFT_Menu_Command == 12) {
      Serial.println("TX Rain to TFT");
      Data1 = Rain_Sensor_Installed;
      Data2 = Rain_Total_Hits_Go_Home;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();

      Serial.print(F("Rain Sensor ON = "));
      Serial.println(Rain_Sensor_Installed);
      Serial.print(F("Rain Sensitivity = "));
      Serial.println(Rain_Total_Hits_Go_Home);
      Serial.println(F(" "));
      }


// Wheel Motor Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 14) {
      Serial.println("TX Wheel PWM to TFT");
      Data1 = PWM_MaxSpeed_LH;
      Data2 = PWM_MaxSpeed_RH;
      Data3 = PWM_Slow_Speed_LH;
      Data4 = PWM_Slow_Speed_RH;
      Data5 = Slow_Speed_MAG / 10;
      Data6 = Wheels_Activate;  // RVES added
      Data7 = 0;
      TX_7_Data_Values();

      Serial.print(F("PWM LH = "));
      Serial.println(PWM_MaxSpeed_LH);
      Serial.print(F("PWM RH = "));
      Serial.println(PWM_MaxSpeed_RH);
      Serial.print(F("PWM S LH = "));
      Serial.println(PWM_Slow_Speed_LH);
      Serial.print(F("PWM S RH = "));
      Serial.println(PWM_Slow_Speed_RH);
      Serial.print(F("Slow MAG = "));
      Serial.println(Slow_Speed_MAG);
      Serial.print(F("Wheels_Activate = ")); // RVES added
      Serial.println(Wheels_Activate); // RVES added
      Serial.println(F(" ")); 
      }
      
      

// Blade Motor Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 15) {

      Serial.println("TX Blade Data to TFT");
      Data1 = PWM_Blade_Speed;
      Data2 = Cutting_Blades_Activate;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data6 = 0;
      TX_7_Data_Values();
     
      Serial.print(F("Blade Motor PWM = "));
      Serial.println(PWM_Blade_Speed);
      Serial.print(F("Cutting Blade Activated = "));
      Serial.println(Cutting_Blades_Activate);
      Serial.println(F(" "));
      }


// Motion Turns Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 16) {

      Serial.println("TX Turns Data to TFT");
      Data1 = Max_Cycles_Straight / 10 ;
      Data2 = Mower_Turn_Delay_Min / 100;
      Data3 = Mower_Turn_Delay_Max / 100;
      Data4 = Mower_Reverse_Delay / 100;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      
      Serial.print(F("Max Mow L = "));
      Serial.println(Max_Cycles_Straight);
      Serial.print(F("Turn Min = "));
      Serial.println(Mower_Turn_Delay_Min);
      Serial.print(F("Turn Max = "));
      Serial.println(Mower_Turn_Delay_Max);
      Serial.print(F("Reverse = "));
      Serial.println(Mower_Reverse_Delay);
      Serial.println(F(" "));

}


// Tracking to Exit Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 17) {
      Serial.println("TX Exit to TFT");

      Data1 = Track_Wire_Zone_1_Cycles / 100 ;
      Data2 = Track_Wire_Zone_2_Cycles / 100;
      Data3 = CCW_Tracking_To_Start;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      Serial.print(F("Track Wire Zone 1 = "));
      Serial.println(Track_Wire_Zone_1_Cycles);    
      Serial.print(F("Track Wire Zone 2 = "));
      Serial.println(Track_Wire_Zone_2_Cycles);      
      if (CCW_Tracking_To_Start == 0) {
        CW_Tracking_To_Start = 1;
        CCW_Tracking_To_Charge = 1;
        CW_Tracking_To_Charge = 0;
        Serial.println(F("Tracking to Charge = CCW"));
        Serial.println(F("Tracking to Start = CW"));
        }        
      if (CCW_Tracking_To_Start == 1) {
        CW_Tracking_To_Start = 0;
        CCW_Tracking_To_Charge = 0;
        CW_Tracking_To_Charge = 1;        
        Serial.println(F("Tracking to Charge = CW"));
        Serial.println(F("Tracking to Start = CCW"));       
        }
    }


// Charging Options Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 8) {
      Serial.println("TX Tracking to TFT");
    
      Serial3.print(Use_Charging_Station);
      Serial3.println("\a");
      delay(300); 

      Serial.print(F("Use Charging Station = "));
      Serial.println(Use_Charging_Station); 
      } 


// Find Wire Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 18) {
      Serial.println("TX Find Wire to TFT");
      Data1 = Max_Cycle_Wire_Find / 100 ;
      Data2 = Max_Cycle_Wire_Find_Back / 10;
      Data3 = Home_Wire_Compass_Heading;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      Serial.print(F("Wire Find Forwards / cylces = "));
      Serial.println(Max_Cycle_Wire_Find);      
      Serial.print(F("Wire Find Backwards / cylces = "));
      Serial.println(Max_Cycle_Wire_Find_Back);
      Serial.print(F("Home Compass Heading / degrees "));
      Serial.println(Home_Wire_Compass_Heading);      
      }

// Tracking PID Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 19) {
      Serial.println("TX Find Wire to TFT");
      
      Serial3.print(Max_Tracking_Turn_Left / 10);
      Serial3.println("\a");
      delay(300);  

      Serial3.print(Max_Tracking_Turn_Right / 10);
      Serial3.println("\b");
      delay(200);  

      Serial3.print(P * 100);
      Serial3.println("\c");
      delay(200);  

      Serial.print(F("Max Cycles Left Wheel = "));
      Serial.println(Max_Tracking_Turn_Left);
      Serial.print(F("Max Cycles Right Wheel = "));
      Serial.println(Max_Tracking_Turn_Right);         
      Serial.print(F("PID P = "));
      Serial.println(P);
      Serial.println(F(" "));     
      }


// Compass Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 27) {
      Serial.println("TX Compass to TFT");
      
      Serial3.print(Compass_Activate);
      Serial3.println("\a");
      delay(300);  

      Serial3.print(Compass_Heading_Hold_Enabled);
      Serial3.println("\b");
      delay(300);  

      Serial3.print(CPower * 10);
      Serial3.println("\c");
      delay(300);  
      
      Serial3.print(Compass_Setup_Mode);
      Serial3.println("\d");
      delay(200);  

      Serial.print(F("Compass Activated = "));
      Serial.println(Compass_Activate);
      Serial.print(F("Heading Hold Enabled = "));
      Serial.println(Compass_Heading_Hold_Enabled);          
      Serial.print(F("Compass Power = "));
      Serial.println(CPower);    
      Serial.print(F("Compass Setup Mode = "));
      Serial.println(Compass_Setup_Mode); 
      }


// GYRO Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 28) {

      Serial3.print(GYRO_Enabled);
      Serial3.println("\a");
      delay(300);  

      Serial3.print(GPower * 10);
      Serial3.println("\b");
      delay(300);

      Serial.print(F("GYRO Enabled= "));
      Serial.println(GYRO_Enabled); 
      Serial.print(F("GYRO Power = "));
      Serial.println(GPower);     
    }


// GPS Menu
    if (TFT_Menu_Command == 32) {
      Serial.println("GPS Enabled");

      Serial3.print(GPS_Enabled);
      Serial3.println("\a");
      delay(300);  

      Serial3.print(GPS_Type);
      Serial3.println("\b");
      delay(300);

    Serial.print(F("GPS: "));
        if (GPS_Enabled == 1) Serial.println("ON");
        if (GPS_Enabled == 0) Serial.println("OFF");    
    Serial.print(F("GPS Type: "));
        if (GPS_Type == 1) Serial.println("ReP_AL");
        if (GPS_Type == 2) Serial.println("PIXHAWK");    
        if (GPS_Type == 3) Serial.println("Spare");      
    }


// Wheel Blockage Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 29) {

      Serial.println("TX Wheel Block to TFT");
      Data1 = Wheel_Amp_Sensor_ON;
      float Temp = Max_Wheel_Amps * 100;
      int Temp2 = Temp;
      Data2 = Temp2;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();

    Serial.print(F("Wheel Amp ON: "));
        if (Wheel_Amp_Sensor_ON == 1) Serial.println("ON");
        if (Wheel_Amp_Sensor_ON == 0) Serial.println("OFF");   
    Serial.print(F("Wheel Amps Max: "));
    Serial.println(Max_Wheel_Amps);     
    }

// Alarm 1 Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 20) {
      Serial.println("TX Alarm1 TFT");

      Data1 = Alarm_1_ON;
      Data2 = Alarm_1_Hour;
      Data3 = Alarm_1_Minute;
      Data4 = Alarm_1_Repeat;
      Data5 = Alarm_1_Action;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();

      Serial.print(F("Alarm 1 ON = "));
      Serial.println(Alarm_1_ON);
      Serial.print(F("Alarm 1 Time = "));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      Serial.println(Alarm_1_Minute);
      Serial.print(F("Alarm 1 Repeat = "));
      Serial.println(Alarm_1_Repeat);
      Serial.print(F("Alarm 1 Action = "));
      Serial.println(Alarm_1_Action);    
      }


// Alarm 2 Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 21) {
      Serial.println("TX Alarm2 TFT");
      
      Data1 = Alarm_2_ON;
      Data2 = Alarm_2_Hour;
      Data3 = Alarm_2_Minute;
      Data4 = Alarm_2_Repeat;
      Data5 = Alarm_2_Action;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();

      Serial.print(F("Alarm 2 ON = "));
      Serial.println(Alarm_2_ON);
      Serial.print(F("Alarm 2 Time = "));
      Serial.print(Alarm_2_Hour);
      Serial.print(F(":"));
      Serial.println(Alarm_2_Minute);
      Serial.print(F("Alarm 2 Repeat = "));
      Serial.println(Alarm_2_Repeat);
      Serial.print(F("Alarm 2 Action = "));
      Serial.println(Alarm_2_Action);    
      }


// Alarm 3 Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 22) {
      Serial.println("TX Alarm3 TFT");
      
      Data1 = Alarm_3_ON;
      Data2 = Alarm_3_Hour;
      Data3 = Alarm_3_Minute;
      Data4 = Alarm_3_Repeat;
      Data5 = Alarm_3_Action;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();

      Serial.print(F("Alarm 3 ON = "));
      Serial.println(Alarm_3_ON);
      Serial.print(F("Alarm 3 Time = "));
      Serial.print(Alarm_3_Hour);
      Serial.print(F(":"));
      Serial.println(Alarm_3_Minute);
      Serial.print(F("Alarm 3 Repeat = "));
      Serial.println(Alarm_3_Repeat);
      Serial.print(F("Alarm 3 Action = "));
      Serial.println(Alarm_3_Action);    
      }


// Set Time Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 23) {
      Serial.println("Set Time TFT");
      if (PCB == 0) {
        Time t = rtc.time();
        Time_Hour = t.hr;
        Time_Minute = t.min;
        Time_Date = t.date;
        Time_Month = t.mon;
        Time_Year = t.yr;
        Time_Day = t.day;
        }
      if (PCB == 1) {
        byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
        Read_DS3231_PCB_Time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
        Time_Hour = hour;
        Time_Minute = minute;
        Time_Second = second;
        Time_Date = dayOfMonth;
        Time_Month = month;
        Time_Year = year;
        Time_Day = dayOfWeek;
        }
 
      Serial3.print(Time_Hour);
      Serial3.println("\a");
      delay(300);  

      Serial3.print(Time_Minute);
      Serial3.println("\b");
      delay(200);

      Serial3.print(Time_Date);
    Serial3.println("\e");
    delay(200);

      Serial3.print(Time_Month);
    Serial3.println("\f");
    delay(200);

      Serial3.print(Time_Year);
    Serial3.println("\w");
    delay(200);

      Serial3.print(Time_Day);
    Serial3.println("\h");
    delay(200);
/*
      Serial.print(F("DateTime Now = "));
      Serial.print(Time_Year);
      Serial.print(F("-"));
      if (Time_Month < 10) Serial.print(F("0"));
      Serial.print(Time_Month);
    Serial.print(F("-"));
    if (Time_Date < 10) Serial.print(F("0"));
    Serial.print(Time_Date);
    Serial.print(F(" "));
    if (Time_Hour < 10) Serial.print(F("0"));
      Serial.print(Time_Hour);
      Serial.print(F(":"));
      if (Time_Minute < 10) Serial.print(F("0"));
      Serial.println(Time_Minute);
      */
    PrintTimeToSerial(2, Time_Year, Time_Month, Time_Date, Time_Hour, Time_Minute, 0, Time_Day, 1);
      }



// Tip Sensor Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 24) {

      Serial.println("TX Tip Sensor to TFT");
      Data1 = Angle_Sensor_Enabled;
      Data2 = Tip_Over_Sensor_Enabled;
      Data3 = 0;
      Data4 = 0;
      Data5 = 0;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      Serial.print(F("Angle ON = "));
      Serial.println(Angle_Sensor_Enabled);
      Serial.print(F("Tip ON = "));
      Serial.println(Tip_Over_Sensor_Enabled);
      Serial.println(" ");
}


// Pattern Mow Menu is selected on the TFT Screen
    if (TFT_Menu_Command == 25) {
      Serial.println("TX Pattern to TFT");
      Data1 = Pattern_Mow;
      Data2 = Turn_90_Delay_LH / 10;
      Data3 = Turn_90_Delay_RH / 10;
      Data4 = Move_to_next_line_delay / 10;
      Data5 = Line_Length_Cycles;
      Data6 = 0;
      Data7 = 0;
      TX_7_Data_Values();
      if (Pattern_Mow == 0) Serial.println("OFF");
      if (Pattern_Mow == 1) Serial.println("Parallel");  
      if (Pattern_Mow == 2) Serial.println("Spiral"); 
      Serial.print(F("Turn 90° LH= "));
      Serial.println(Turn_90_Delay_LH);  
      Serial.print(F("Turn 90° RH= "));
      Serial.println(Turn_90_Delay_RH);      
      Serial.print(F("Distance to next row= "));
      Serial.println(Move_to_next_line_delay);      
      Serial.print(F("Row Length = "));
      Serial.println(Line_Length_Cycles);  
      Serial.println(" "); 
}



// Initial Start Up Values requested from the TFT
// The TFT first sends a request for the MEGA to 
    if (TFT_Menu_Command == 55) {
      Serial.println("TX Start Up Values TFT - Alarms");

      Data1 = Alarm_1_ON;
      Data2 = Alarm_2_ON;
      Data3 = Alarm_3_ON;
      //Data4 = reserve;
      //Data5 = reserve;
      Data6 = GPS_Enabled;
      Data7 = Perimeter_Wire_Enabled;
      TX_7_Data_Values();

      Serial.print(F("Alarm 1 ON = "));
      Serial.println(Alarm_1_ON);
      Serial.print(F("Alarm 2 ON = "));
      Serial.println(Alarm_2_ON);
      Serial.print(F("Alarm 3 ON = "));
      Serial.println(Alarm_3_ON);

      Serial.print(F("GPS:"));
      if (GPS_Enabled == 1) Serial.println("ON");   
      if (GPS_Enabled == 0) Serial.println("OFF"); 
      Serial.print(F("WIRE:"));
      if (Perimeter_Wire_Enabled == 1) Serial.println("ON");   
      if (Perimeter_Wire_Enabled == 0) Serial.println("OFF"); 

}


  // Date
  if (TFT_Menu_Command == 56) {
    Serial.println("TX Start Up Values TFT - DateTime");

    Get_Current_Time_Print_On_Serial_Monitor();

    if (PCB == 0) {
      //Time t = rtc.time();
      Serial.println("Using Clock Module None PCB");
      }
    else if (PCB == 1) {
      Serial.println("Using PCB Clock");
          //byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
          //Read_DS3231_PCB_Time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
      }

    Data1 = Time_Year;
    Data2 = Time_Month;
    Data3 = Time_Date;
    Data4 = Time_Hour;
    Data5 = Time_Minute;
    Data6 = Time_Day;
    Data7 = 0;
    TX_7_Data_Values();
/*
    Serial.print(F("DateTime: "));
    Serial.print(Time_Year);
    Serial.print(F("-"));
    if (Time_Month < 10) Serial.print(F("0"));
    Serial.print(Time_Month);
    Serial.print(F("-"));
    if (Time_Date < 10) Serial.print(F("0"));
    Serial.print(Time_Date);
    Serial.print(F(" "));
    if (Time_Hour < 10) Serial.print("0");
    Serial.print(Time_Hour);
    Serial.print(F(":"));
    if (Time_Minute < 10) Serial.print("0");
    Serial.print(Time_Minute);

    Serial.print(F(" DOW:"));
    Serial.println(Time_Day);
    */
    PrintTimeToSerial(2, Time_Year, Time_Month, Time_Date, Time_Hour, Time_Minute, 0, Time_Day, 1);
  }


// Test Wire Sensor
    if (TFT_Menu_Command == 41) {
      Serial.println("Test Wire Sensor");

      bool Test_Complete = 0;

      for (int i = 0; i <= 30; i++) {

          if (i > 28) Test_Complete = 1;
            ADCMan.run();
            // ADCMan.setCapture(pinPerimeterLeft, 1, 0);
          
            if (millis() >= nextTime)  {
              nextTime = millis() + 50;
              if (perimeter.isInside(0) != inside) {
                inside = perimeter.isInside(0);
                counter++;
              }
            }
     
          Serial3.print((perimeter.isInside(0)));
          Serial3.println("\a");
          delay(300);  
    
          Serial3.print(perimeter.getMagnitude(0));
          Serial3.println("\b");
          delay(200);  

          Serial3.print(Test_Complete);
          Serial3.println("\c");
          delay(200);  
    
          Serial.print(F("IN/Out:"));
          Serial.print(perimeter.isInside(0));
          Serial.print(F("   MAG:"));
          Serial.print(perimeter.getMagnitude(0)); 
          Serial.print(F("   i:"));
          Serial.println(i);
          }
      Serial.println(F("Test Completed"));
      }


// Start Sonar Test
  if (TFT_Menu_Command == 45) {
    Serial.println(F("Sonar Test"));
    bool Test_Complete = 0;

    // turn on the Sonoar Sensors for the test
    int Sonar1_Status = Sonar_1_Activate; 
    int Sonar2_Status = Sonar_2_Activate; 
    int Sonar3_Status = Sonar_3_Activate; 

    Serial.println(F("Activating Sonar Array"));
    Sonar_1_Activate = 1;
    Sonar_2_Activate = 1;
    Sonar_3_Activate = 1;    
    
      for (int i = 0; i <= 30; i++) {

        if (i > 28) Test_Complete = 1;
        if (Sonar_1_Activate == 1) distance1 = PingSonarY(trigPin1, echoPin1, 1, 1, 1, 5, 0);          //SONAR1
        if (Sonar_1_Activate == 0) distance1 = 999;
        delay(15);       
        if (Sonar_2_Activate == 1) distance2 = PingSonarY(trigPin2, echoPin2, 2, 2, 2, 0, 0);         //SONAR2
        if (Sonar_2_Activate == 0) distance1 = 999;
        delay(15);
        if (Sonar_3_Activate == 1) distance3 = PingSonarY(trigPin3, echoPin3, 3, 3, 3, 10, 0);          //SONAR3
        if (Sonar_3_Activate == 0) distance1 = 999;
        delay(15);
        
        Serial3.print(distance1);
        Serial3.println("\a");
        delay(300);  
    
        Serial3.print(distance2);
        Serial3.println("\b");
        delay(200);  

        Serial3.print(distance3);
        Serial3.println("\c");
        delay(200); 

        Serial3.print(Test_Complete);
        Serial3.print("\d");
        delay(200);    

        Serial.print("  Test Complete");
        Serial.println(Test_Complete);
      
      
      }
    Serial.println(F("Sonar Test Complete"));

    Serial.println(F("Restting Sonar Array ON/OFFStatus"));
    Sonar_1_Activate = Sonar1_Status;
    Sonar_2_Activate = Sonar2_Status;
    Sonar_3_Activate = Sonar3_Status;   

    }


// Start Compass Test
  if (TFT_Menu_Command == 48) {
    Serial.println(F("Compass Test"));
    bool Test_Complete = 0;
    // turn on the Compass Sensor for the test
    int Compass_Status = Compass_Activate; 

    if (Compass_Activate == 0) {
        Serial.println(F("Turning on Compass for Test"));
        Compass_Activate = 1;
        Setup_DFRobot_QMC5883_HMC5883L_Compass();                     // USes the DFRobot Library
        }

      int Cycles = 40;

      for (int i = 0; i <= Cycles; i++) {

        if (i > (Cycles - 2) ) Test_Complete = 1;
        Get_Compass_Reading();

        int Compass_Degrees_TX = Compass_Heading_Degrees;
        delay(300);
        
        Serial3.print(Heading * 10);
        Serial3.println("\a");
        delay(300);  
      
        Serial3.print(Compass_Degrees_TX);
        Serial3.println("\b");
        delay(300); 

        Serial.print("Heading: ");
        Serial.print(Heading);
        Serial.print("  Degrees: ");
        Serial.println(Compass_Degrees_TX);
  
     
        Serial3.print(Test_Complete);
        Serial3.println("\c");
        delay(300);    


      }

    Serial.print(F("Returning Compass ON/OFF to original state"));
    Compass_Activate = Compass_Status; 
    Serial.print(F("Compass Test Complete"));
    Serial.println(Test_Complete);
    }


// Start GYRO Test
  if (TFT_Menu_Command == 39) {
    Serial.println(F("GYRO Test"));
    bool Test_Complete = 0;

      int Cycles = 60;

      for (int i = 0; i <= Cycles; i++) {

        if (i > (Cycles - 2) ) Test_Complete = 1;
        Get_GYRO_Reading();
        Print_GYRO_Reading();

       
        Serial3.print(GYRO_Angle_X);
        Serial3.println("\a");
        delay(300);  
      
        Serial3.print(GYRO_Angle_Y);
        Serial3.println("\b");
        delay(300); 
    
        Serial3.print(Test_Complete);
        Serial3.println("\c");
        delay(300);    

        //Serial.print("Angle X: ");
        //Serial.print(GYRO_Angle_X);
        //Serial.print(" Angle Y: ");
        //Serial.println(GYRO_Angle_Y);

      }

    Serial.print(F("Compass Test Complete"));
    Serial.println(Test_Complete);
    }


// Start Volt Amp Test
  if (TFT_Menu_Command == 47) {
    Serial.println(F("Volt Amp Test"));
    bool Test_Complete = 0;

      for (int i = 0; i <= 30; i++) {

        if (i > 28) Test_Complete = 1;
        
        Read_Serial1_Nano();
        delay(200);

        int VoltsRX = (Volts * 100) / 2;
   
        Serial3.print(VoltsRX);
        Serial3.println("\a");
        delay(100);  
    
        Serial3.print(Amps * 100);
        Serial3.println("\b");
        delay(100); 
        
        Serial3.print(Test_Complete);
        Serial3.println("\c");
        delay(300);    

        Serial.print("VoltsRX: ");
        Serial.print(VoltsRX);
        Serial.print("  Volts: ");
        Serial.print(Volts);
        Serial.print("  Amps: ");
        Serial.println(Amps);
      }

    Serial.print(F("Volt Amp Test Complete"));
    Serial.println(Test_Complete);
    }

// Start Tilt Test
  if (TFT_Menu_Command == 40) {
    Serial.println(F("Tilt Test"));
    bool Test_Complete = 0;
    Setup_Tilt_Tip_Safety();

      for (int i = 0; i <= 30; i++) {

        if (i > 28) Test_Complete = 1;
        
        Check_Tilt_Tip_Angle(); 
        delay(200);
   
        Serial3.print(Tilt_Angle_Sensed);
        Serial3.println("\a");
        delay(100);  
    
        Serial3.print(Tilt_Orientation_Sensed);
        Serial3.println("\b");
        delay(100); 
        
        Serial3.print(Test_Complete);
        Serial3.println("\c");
        delay(300);    

        Serial.print(F("   Data sent"));

        Serial.print(F("  Test Completed "));
        Serial.println(Test_Complete);
      }

    Serial.print(F("Tilt Test Complete : "));
    Serial.println(Test_Complete);
    }



// Start Bumper Test
  if (TFT_Menu_Command == 50) {
    Serial.println(F("Bumper Bar Test"));
    
    Setup_Microswitches();
    
    bool Test_Complete = 0;
    bool Bump_LH;
    bool Bump_RH;

      for (int i = 0; i <= 30; i++) {

        if (i > 28) Test_Complete = 1;
        
        if (digitalRead(Microswitch_2))  Bump_LH = 0; 
        if (!digitalRead(Microswitch_2)) Bump_LH = 1;
        if (digitalRead(Microswitch_1))  Bump_RH = 0; 
        if (!digitalRead(Microswitch_1)) Bump_RH = 1;
        delay(200);

 
        Serial3.print(Bump_LH);
        Serial3.println("\a");
        delay(100);  
    
        Serial3.print(Bump_RH);
        Serial3.println("\b");
        delay(100); 
        
        Serial3.print(Test_Complete);
        Serial3.println("\c");
        delay(300);    

        Serial.print("Bump LH: ");
        Serial.print(Bump_LH);
        Serial.print("  Bump RH: ");
        Serial.println(Bump_RH);
      }

    Serial.print(F("Volt Bumper Test Complete"));
    Serial.println(Test_Complete);
    }

}

void Send_Mower_Tracking_Data() {
      byte data[7];
      data[0] = 11; // Packet ID
      data[1]=Turn_To_Home;
      data[2]=Find_Wire_Track;
      data[3]=Go_To_Charging_Station;
      data[4]=Mower_Docked;
      data[5]=Robot_Status_Value;
      data[6]=Mower_Error_Value;
      SerialCom3.sendMsg (data, sizeof (data));
      }

void Send_Mower_Error_Data() {
        
        Serial3.print(Robot_Status_Value);
        Serial3.println("\a");
        delay(300); 

        Serial3.print(Mower_Error_Value);
        Serial3.println("\b");
        delay(300); 
        
        Serial.print("Sending Error Data to TFT");

        Serial.println(F(""));
        Serial.print(F("|RSV:"));
        Serial.print(Robot_Status_Value); 
        Serial.print(F("|ME:"));
        Serial.print(Mower_Error_Value); 
        Serial.println(F(""));
        }

void Send_Mower_Setup_Data() {
        
        Serial3.print(Robot_Status_Value);
        Serial3.println("\a");
        delay(300); 
        
        Serial.print(F("TX Setup to TFT"));
        Serial.print(F("|RSV:"));
        Serial.print(Robot_Status_Value); 

        }



void Send_Mower_Running_Data() {
    float VoltsTX1 = Volts * 100;
    VoltsTX = VoltsTX1;
//    Bumper_Status = Bumper;
    float WheelAmpsTX1 = WheelAmps * 100;
    WheelAmpsTX = WheelAmpsTX1;
    Compass_Heading_DegreesTX = Compass_Heading_Degrees;
    byte data[19];
    data[0] = 12; // Packet ID

    data[1] = Sonar_Status;
    data[2] = Outside_Wire;
//    data[3] = Bumper_Status;
    data[4] = Robot_Status_Value;
    data[5] = Mower_Error_Value;
    data[6] = Tilt_Angle_Sensed;
    data[7] = VoltsTX;
    data[8] = VoltsTX >> 8;
    data[9] = GPS_Inside_Fence;
    data[10]= GPS_Lock_OK;
    data[11]= Loop_Cycle_Mowing;
    data[12]= Loop_Cycle_Mowing >> 8;
    data[13]= Compass_Steering_Status;
    data[14]= WheelAmpsTX;
    data[15]= WheelAmpsTX >> 8;
    data[16]= Compass_Heading_DegreesTX;
    data[17]= Compass_Heading_DegreesTX >> 8;

    bitWrite(data[18],0,Wheel_Blocked_Status);
    //bitWrite(data[18],1,reserve);
    //bitWrite(data[18],2,reserve);
    //bitWrite(data[18],3,reserve);
    //bitWrite(data[18],4,reserve);
    //bitWrite(data[18],5,reserve);
    //bitWrite(data[18],6,reserve);
    //bitWrite(data[18],7,reserve);

    SerialCom3.sendMsg (data, sizeof (data));

  Calculate_TFT_Robot_Status_Value();

  Serial.print(F("|S:"));
  Serial.print(Sonar_Status);
  Serial.print(F("|W:"));
  Serial.print(Outside_Wire);
//  Serial.print(F("|B:"));
//  Serial.print(Bumper_Status);
  Serial.print(F("|RSV:"));
  Serial.print(Robot_Status_Value);
  Serial.print(F("|ME:"));
  Serial.print(Mower_Error_Value);
  Serial.print(F("|Tip:"));
  Serial.print(Tilt_Angle_Sensed);
  Serial.print(F("|VTX:"));
  Serial.print(VoltsTX);
  if (GPS_Enabled) {
    Serial.print(F("|GPS Inside Fence:"));
    if (GPS_Inside_Fence == 0) Serial.print("OUT");
    if (GPS_Inside_Fence == 1) Serial.print("IN");
    Serial.print(F("|GPS Lock:"));
    if (GPS_Lock_OK == 0) Serial.print("No lock");
    if (GPS_Lock_OK == 1) Serial.print("RTK FIX");
  }
}


void Send_Aerator_Running_Data() {
    float VoltsTX1 = Volts * 100;
    VoltsTX = VoltsTX1;
//    Bumper_Status = Bumper;
    float WheelAmpsTX1 = WheelAmps * 100;
    WheelAmpsTX = WheelAmpsTX1;
    Compass_Heading_DegreesTX = Compass_Heading_Degrees;
    byte data[19];
    data[0] = 12; // Packet ID

    data[1] = Sonar_Status;
    data[2] = Outside_Wire;
    data[3] = Drill_Status;
    data[4] = Robot_Status_Value;
    data[5] = Mower_Error_Value;
    data[6] = Tilt_Angle_Sensed;
    data[7] = VoltsTX;
    data[8] = VoltsTX >> 8;
    data[9] = GPS_Inside_Fence;
    data[10]= GPS_Lock_OK;
    data[11]= Loop_Cycle_Mowing;
    data[12]= Loop_Cycle_Mowing >> 8;
    data[13]= Compass_Steering_Status;
    data[14]= WheelAmpsTX;
    data[15]= WheelAmpsTX >> 8;
    data[16]= Compass_Heading_DegreesTX;
    data[17]= Compass_Heading_DegreesTX >> 8;

    bitWrite(data[18],0,Wheel_Blocked_Status);
    //bitWrite(data[18],1,reserve);
    //bitWrite(data[18],2,reserve);
    //bitWrite(data[18],3,reserve);
    //bitWrite(data[18],4,reserve);
    //bitWrite(data[18],5,reserve);
    //bitWrite(data[18],6,reserve);
    //bitWrite(data[18],7,reserve);

    SerialCom3.sendMsg (data, sizeof (data));

  Calculate_TFT_Robot_Status_Value();


        Serial.print(F("|S:"));          
        Serial.print(Sonar_Status);
        Serial.print(F("|W:"));
        Serial.print(Outside_Wire);  
//        Serial.print(F("|B:"));
//        Serial.print(Bumper_Status); 
        Serial.print(F("|RSV:"));
        Serial.print(Robot_Status_Value);  
        Serial.print(F("|ME:"));
        Serial.print(Mower_Error_Value); 
        Serial.print(F("|Tip:"));
        Serial.print(Tilt_Angle_Sensed);     
        Serial.print(F("|VTX:"));
        Serial.print(VoltsTX);
        if (GPS_Enabled) {
      Serial.print(F("|GPS Inside Fence:"));
      if (GPS_Inside_Fence == 0) Serial.print("OUT");
      if (GPS_Inside_Fence == 1) Serial.print("IN");
      Serial.print(F("|GPS Lock:"));
      if (GPS_Lock_OK == 0) Serial.println("No lock");
      if (GPS_Lock_OK == 1) Serial.println("RTK FIX");
        }
  }


//*********

void Send_PIXHAWK_Running_Data() {
        int Delay_running = 10;
        int spare = 0;
        
        Serial.print(F("TX:"));
       
        Serial3.print(PWM_Arduino_LH);
        Serial3.println(F("\a"));        
        delay(Delay_running);
    
        Serial3.print(PWM_Arduino_RH);
        Serial3.println(F("\b"));
        delay(Delay_running);        
        
        Serial3.print(spare);
        Serial3.println("\c");
        delay(Delay_running);    

        Serial3.print(spare);
        Serial3.println("\s");
        delay(Delay_running);    

        Serial3.print(spare);      
        Serial3.println("\e");
        delay(Delay_running); 
  
        Serial3.print(spare);      
        Serial3.println("\f");
        delay(Delay_running); 
        
        Serial3.print(spare);      
        Serial3.println("\g");
        delay(Delay_running);

        Serial3.print(spare);      
        Serial3.println("\h");
        delay(Delay_running);

        Serial3.print(cycles);      
        Serial3.println("\i");
        delay(Delay_running);        

        Serial.print(F("|LW:"));          
        Serial.print(PWM_Arduino_LH);
        Serial.print(F("|RW:"));
        Serial.print(PWM_Arduino_RH);  

        Serial.print(F("|C:"));
        Serial.print(cycles);  
        Serial.print(F("|"));
        }


// Sends Wheel Amp Data during Test
void Send_Wheel_Amp_Data() {
        Serial3.print(Wheel_Blocked);      
        Serial3.println("\a");
        delay(300);
        }



// Transfers the values to the TFT in Docked Mode.
void Send_Mower_Docked_Data()  {

      Calculate_TFT_Robot_Status_Value();
      // Reformat Volts value for transmission
      float VoltsTX1 = Volts * 100;
      VoltsTX = VoltsTX1;

      byte data[15];
      data[0] = 10; // Packet ID

      data[1]   = Time_Year;
      data[2]   = Time_Year >> 8;
      data[3]   = Time_Month;
      data[4]   = Time_Date;
      data[5]   = Time_Hour;
      data[6]   = Time_Minute;
      data[7]   = Time_Second;
      data[8]   = Time_Day;

      data[9]   = VoltsTX;
      data[10]  = VoltsTX >> 8;
      data[11]  = Robot_Status_Value;
      data[12]  = Mower_Error_Value;
      data[13]  = Charging;

      int GPS_Lock_OK_Docked_TX;
      if (GPS_Lock_OK == 0) GPS_Lock_OK_Docked_TX = 2;
      if (GPS_Lock_OK == 1) GPS_Lock_OK_Docked_TX = 5;
      data[14]=GPS_Lock_OK_Docked_TX;

      SerialCom3.sendMsg (data, sizeof (data));


        Serial.print(F("|VTX:"));
        Serial.print(VoltsTX);
        Serial.print(F("|RSV:"));
        Serial.print(Robot_Status_Value);            
        Serial.print(F("|ME:"));
        Serial.print(Mower_Error_Value);                 
        Serial.print(F("|C:"));
        Serial.print(Charging); 
        Serial.print(F("|LOCK:"));
        Serial.print(GPS_Lock_OK);            
        } 

void Calculate_TFT_Robot_Status_Value() {

  //    Mower Status Values
  //    1 = Docked
  //    2 = Parked
  //    3 = Setup App
  //    4 = Error State
  //    5 = Mowing
  //    6 = Manual Mode
  //    7 = Tracking
  //    8 = Rain
  //    9 = Exiting Dock
  //    11 = Manual Mode


   Robot_Status_Value = 0;

   if ((Wire_Detected == 1) && (Tilt_Orientation_Sensed == 0)) {
      Mower_Error = 0;
      if ((Mower_Docked == 1) || (Mower_Parked == 1)) {   
         Mower_Error_Value = 0;
         }
   }

   
   
   if (Wire_Detected == 0) {

        // If the mower is running and no wire is detected then the full error screen is displayed
        if (Mower_Running == 1) {
              Mower_Error = 1;
              Mower_Error_Value = 1;     // 1 = No Wire Detected
              }
        // If the mower is docked then the Wire Off message is displayed on the docked screen. 
        if ((Mower_Docked == 1) || (Mower_Parked == 1)) {
              Mower_Error_Value = 1;
              }
        }


   if (Mower_Docked == 1)           Robot_Status_Value = 1;
   if (Mower_Parked == 1)           Robot_Status_Value = 2; 
   
   if (Mower_Parked_Low_Batt == 1)  {
     Robot_Status_Value = 4;  // Error Mode
     Mower_Error_Value = 2;   // 2 = Low Battery
     }
   
   if (Mower_Error == 1)                                    Robot_Status_Value = 4;  // Error
   if (Mower_Error == 3) {
        Robot_Status_Value = 4;  // Error
        Mower_Error_Value = 3;   
        }
   
   //NEW
   if (Mower_Setup_Mode == 1)                               Robot_Status_Value = 3;
   if (Mower_Running == 1)                                  Robot_Status_Value = 5;
   if (Manual_Mode == 1)                                    Robot_Status_Value = 6;
   if (Tracking_Wire == 1)                                  Robot_Status_Value = 7;
   if (Rain_Hit_Detected == 1)                              Robot_Status_Value = 8;
   if (Exiting_Dock == 1)                                   Robot_Status_Value = 9;
   }
       

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// TXRX_NANO
void Read_Serial1_Nano() {

  if (Mower_PIXHAWK == 0) {

  byte recvBuff [10] = {0};

  if (SerialCom1.update ())
  {
    byte length = SerialCom1.getLength ();
    if (length > sizeof (recvBuff)) length = sizeof (recvBuff);
    memcpy (&recvBuff, SerialCom1.getData (), length);

      RawValueAmp =   recvBuff[0] | recvBuff[1] << 8;
      RawValueVolt =  recvBuff[2] | recvBuff[3] << 8;
      Rain_Detected = recvBuff[4] | recvBuff[5] << 8;
      RawWheelAmp =   recvBuff[6] | recvBuff[7] << 8;
      RawDrillAmp =   recvBuff[8] | recvBuff[9] << 8;
  }  // end if something received

  //Serial.print("WAmpRaw: ");
  //Serial.print(RawWheelAmp);
  //Serial.print(" |");

  //Serial.print("DAMpRaw: ");
  //Serial.print(RawDrillAmp);
  //Serial.print(" |");

  Calculate_Volt_Amp_Charge();

  }
}    

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// TX_RX_TFT
// Listens to the serial 3 port if a menu command has been executed.
void Check_TFT_Serial_Input() {
  byte recvBuff [3] = {0};

  if (SerialCom3.update ())
  {
    byte length = SerialCom3.getLength ();
    if (length > sizeof (recvBuff)) length = sizeof (recvBuff);
    memcpy (&recvBuff, SerialCom3.getData (), length);

    if (recvBuff[0] == 21) {
      TFT_Menu_Command =   recvBuff[1] | recvBuff[2] << 8;
      Serial.print(F("|TFTserIn:OK|"));

      Confirm_TX_or_RX_Action();    // Prints the Menu Selected
      Confirm_RX();
      delay(5);
      Send_Data_To_TFT();                                   // Send data to TFT - depends on menu code received
      Receive_Data_From_TFT();                              // Receive Data from TFT - depends on menu code received
    }
  }  // end if something received
}



void Activate_TFT_Menu() {

        Serial.println();
        Serial.println(F("TFT Menu Activated"));
        Menu_Complete_TFT = false;                                // Menu complete will return to the normal loop
        Serial.println("waiting for command");
     
     while (Menu_Complete_TFT == false) {                      // holds the program in a loop until a selection has been made in the membrane button menu
          Check_TFT_Serial_Input();
          if (TFT_Menu_Command == 1) Menu_Complete_TFT = true;  
          if (Mower_PIXHAWK == 1) Run_in_TFT_PIXHAWK_Mode();        // Runs the Mower even though its in the TFT Menu
          }

      }



void Run_in_TFT_PIXHAWK_Mode() {

   
   while (Mower_PIXHAWK == true) {  
     Check_TFT_Serial_Input();
     Check_PIXHAWK();       
     if (PIXHAWK_Armed == 1) Check_PIXHAWK_PWM();  
     Serial.println("");   
     }
  
}

void Confirm_RX() {      
      // confirms to the TFT that a request for data transfer has been received.
    byte data[3];
    data[0] = 22; // Packet ID

    int TFT_Menu_Command_Buf = TFT_Menu_Command * 3;
    data[1]=TFT_Menu_Command_Buf;
    data[2]=TFT_Menu_Command_Buf >> 8;

    SerialCom3.sendMsg (data, sizeof (data));
      Serial.print("Confirmstion Code Sent : ");
      Serial.println(TFT_Menu_Command * 3); 
      }




// Based on the code received this menu confirms the menu item to be used.
void Confirm_TX_or_RX_Action() {

Serial.print("TFT Menu Command: ");
Serial.print(TFT_Menu_Command );
Serial.print(" = ");

if (TFT_Menu_Command == 1)  Serial.println(F("Main Menu"));
if (TFT_Menu_Command == 2)  Serial.println(F("Quick Start Menu"));
if (TFT_Menu_Command == 3)  Serial.println(F("Exit Dock Menu"));
if (TFT_Menu_Command == 4)  Serial.println(F("Options Menu"));
if (TFT_Menu_Command == 5)  Serial.println(F("Time Menu"));
if (TFT_Menu_Command == 6)  Serial.println(F("Sensors Menu"));
if (TFT_Menu_Command == 7)  Serial.println(F("Motion Menu"));
if (TFT_Menu_Command == 8)  Serial.println(F("Tracking Menu"));
if (TFT_Menu_Command == 9)  Serial.println(F("Navigation Menu"));
if (TFT_Menu_Command == 10) Serial.println(F("Sonar Menu"));
//  Leave 11 Free.
if (TFT_Menu_Command == 12) Serial.println(F("Rain Sensor Menu"));
if (TFT_Menu_Command == 13) Serial.println(F("Stop Menu"));
if (TFT_Menu_Command == 14) Serial.println(F("Wheel Motor Menu"));
if (TFT_Menu_Command == 15) Serial.println(F("Blade Motor Menu"));
if (TFT_Menu_Command == 16) Serial.println(F("Movement Menu"));
if (TFT_Menu_Command == 17) Serial.println(F("Tracking Exit Points"));
if (TFT_Menu_Command == 18) Serial.println(F("Find Wire"));
if (TFT_Menu_Command == 19) Serial.println(F("Track PID"));
if (TFT_Menu_Command == 20) Serial.println(F("Set Alarm 1"));
if (TFT_Menu_Command == 21) Serial.println(F("Set Alarm 2"));
if (TFT_Menu_Command == 22) Serial.println(F("Set Alarm 3"));
if (TFT_Menu_Command == 23) Serial.println(F("Set Time"));
if (TFT_Menu_Command == 24) Serial.println(F("Tip Sensor Menu"));
if (TFT_Menu_Command == 25) Serial.println(F("Pattern Menu"));
if (TFT_Menu_Command == 26) Serial.println(F("Battery Menu"));
if (TFT_Menu_Command == 27) Serial.println(F("Compass Menu"));
if (TFT_Menu_Command == 28) Serial.println(F("GYRO Menu"));
if (TFT_Menu_Command == 29) Serial.println(F("Wheel Block Amps Menu"));
if (TFT_Menu_Command == 30) Serial.println(F("Setup Other Menu"));
if (TFT_Menu_Command == 31) Serial.println(F("PIXHAWK Go Menu"));
if (TFT_Menu_Command == 32) Serial.println(F("GPS Main Menu"));

if (TFT_Menu_Command == 38) Serial.println(F("Wheel Amp Test"));
if (TFT_Menu_Command == 39) Serial.println(F("GYRO Test"));
if (TFT_Menu_Command == 40) Serial.println(F("Tilt Test"));
if (TFT_Menu_Command == 41) Serial.println(F("Wire Test"));
if (TFT_Menu_Command == 42) Serial.println(F("Relay Test"));
if (TFT_Menu_Command == 43) Serial.println(F("Wheel Test"));
if (TFT_Menu_Command == 44) Serial.println(F("Blade Test"));
if (TFT_Menu_Command == 45) Serial.println(F("Sonar Test"));
if (TFT_Menu_Command == 46) Serial.println(F("Turn Test"));
if (TFT_Menu_Command == 47) Serial.println(F("Volt Amp Test"));
if (TFT_Menu_Command == 48) Serial.println(F("Compass Test"));
if (TFT_Menu_Command == 49) Serial.println(F("Go Home Test"));
if (TFT_Menu_Command == 50) Serial.println(F("Bumper Bar"));
if (TFT_Menu_Command == 55) Serial.println(F("Start-Up Value"));
if (TFT_Menu_Command == 56) Serial.println(F("Start-Up Value Date"));
if (TFT_Menu_Command == 57) Serial.println(F("TX Docked Info"));
if (TFT_Menu_Command == 58) Serial.println(F("Mower Running"));
if (TFT_Menu_Command == 59) Serial.println(F("Test Lift Mechanism"));
if (TFT_Menu_Command == 60) Serial.println(F("Test Lift Mechanism UP"));
if (TFT_Menu_Command == 61) Serial.println(F("Test Lift Mechanism DOWN"));
if (TFT_Menu_Command == 62) Serial.println(F("Initiate Drill Test"));
if (TFT_Menu_Command == 63) Serial.println(F("PIXHAWK Test"));
if (TFT_Menu_Command == 64) Serial.println(F("Start PWM Read"));
if (TFT_Menu_Command == 65) Serial.println(F("Wheel Test Method 2"));
if (TFT_Menu_Command == 66) Serial.println(F("Left Front Forwards"));
if (TFT_Menu_Command == 67) Serial.println(F("Left Front Backwards"));
if (TFT_Menu_Command == 68) Serial.println(F("Right Front Forwards"));
if (TFT_Menu_Command == 69) Serial.println(F("Right Front Backwards"));
if (TFT_Menu_Command == 70) Serial.println(F("Left Rear Forwards"));
if (TFT_Menu_Command == 71) Serial.println(F("Left Rear Backwards"));
if (TFT_Menu_Command == 72) Serial.println(F("Right Rear Forwards"));
if (TFT_Menu_Command == 73) Serial.println(F("Right Rear Backwards"));
if (TFT_Menu_Command == 74) Serial.println(F("Right Rear Backwards"));
if (TFT_Menu_Command == 75) Serial.println(F("Right Rear Backwards"));

if (TFT_Menu_Command == 99) Serial.println(F("Break TX loop and wait for data..."));
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Test_Mower_Sketches
/* Perimieter Wire Collision Motion
  ************************************************************************************/
  void Test_Mower_Check_Wire()  {
  
  ADCMan.run();
  // ADCMan.setCapture(pinPerimeterLeft, 1, 0);

  if (millis() >= nextTime)  {
    nextTime = millis() + 50;
    if (perimeter.isInside(0) != inside) {
      inside = perimeter.isInside(0);
      counter++;
    }
  }

  /* Prints Values to the Serial Monitor of mag, smag and signal quality.  */
  Serial.print(F("Inside (1) or Outside (0):  "));
  Serial.print((perimeter.isInside(0)));
  Serial.print(F("     MAG: "));
  Serial.print((int)perimeter.getMagnitude(0));
  Serial.print(F("    smag: "));
  Serial.print((int)perimeter.getSmoothMagnitude(0));
  Serial.print(F("     qaulity: "));
  Serial.println((perimeter.getFilterQuality(0)));


  #if defined(LCD_KEYPAD)
  lcd.setCursor(0,0);
  lcd.print("IN/Out:");
  lcd.setCursor(8,0);
  lcd.print(perimeter.isInside(0));
  lcd.setCursor(0,1);
  lcd.print("MAG:");
  lcd.setCursor(8,1);
  lcd.print(perimeter.getMagnitude(0)); 
  #endif


}


void Test_Relay() {
  
  Turn_Off_Relay();
  Serial.println("Relay OFF");
  
  #if defined(LCD_KEYPAD)
  lcd.print("Relay OFF");
  lcd.clear();
  #endif
  
  Turn_On_Relay();
  Serial.println("Relay ON");
  delay(2000);
  
  #if defined(LCD_KEYPAD)
  lcd.print("Relay ON");
  delay(1000);
  lcd.clear();
  #endif
  
  Turn_Off_Relay();
  Serial.println("Relay OFF");
  
  #if defined(LCD_KEYPAD)
  lcd.print("Relay OFF");
  #endif

}


void Test_Wheel_Amps () {
    Serial.println("Test Wheel Amps");
    Turn_On_Relay();
    delay(300);
    SetPins_ToGoForwards();
    Motor_Action_Go_Full_Speed();
      for (int i = 0; i <= 100; i++) {
        Read_Serial1_Nano(); 
        Calculate_Wheel_Amps();
        Test_Check_Wheel_Amps();             
        Send_Wheel_Amp_Data();
        Serial.println(F(""));
        }
    Motor_Action_Stop_Motors();
    Turn_Off_Relay();
}


void Test_Wheel_Motors() {
  I = 1;
  Serial.println(F("Wheel Test Started"));
  Turn_On_Relay();
  delay(200);
  if (I == 1) {

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Drive Wheel");
  lcd.setCursor(1,0);
  lcd.print("Test");
  delay(1000);
  lcd.clear();
  lcd.print("Remove ALL");
  lcd.setCursor(0,1);
  lcd.print("Blades!!!");
  delay(1000);
  lcd.clear();
  lcd.print("<-- Turn Left");
  #endif

  Full_Speed_Achieved = 0;
  
  delay(500);
  SetPins_ToTurnLeft();
  Motor_Action_Go_Full_Speed();
  delay(2000);
  Motor_Action_Stop_Motors();
  delay(500);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.print("Turn Right -->");
  #endif
  
  delay(500);
  SetPins_ToTurnRight();
  Motor_Action_Go_Full_Speed();
  delay(2000);
  Motor_Action_Stop_Motors();
  delay(500);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.print("Forwards");
  #endif
  
  delay(500);
  SetPins_ToGoForwards();
  Motor_Action_Go_Full_Speed();
  delay(2000);
  Motor_Action_Stop_Motors(); 
  delay(500);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.print("Backwards");
  #endif
  
  delay(500);
  Serial.println(F(""));
  SetPins_ToGoBackwards();   
  Motor_Action_Go_Full_Speed();
  delay(2000);
  Motor_Action_Stop_Motors();  
  delay(5000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Dynamic");
  lcd.setCursor(0,1);
  #endif
  
  PWM_Left = 150;
  PWM_Right = 150;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,1); 
  #endif
   
  PWM_Left = 255;
  PWM_Right = 0;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,1);
  #endif
  
  PWM_Left = 155;
  PWM_Right = 0;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  Serial.println(F(""));
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,1);  
  #endif
  
  PWM_Left = 255;
  PWM_Right = 0;

  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Dynamic");
  lcd.setCursor(0,1);
  #endif
  
  PWM_Left = 150;
  PWM_Right = 150;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,1);
  #endif
  
  PWM_Left = 0;
  PWM_Right = 255;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,1); 
  #endif
   
  PWM_Left = 0;
  PWM_Right = 155;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);
  
  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.setCursor(0,1);  
  #endif
  
  PWM_Left = 0;
  PWM_Right = 255;
  
  #if defined(LCD_KEYPAD)
  lcd.print("L:");
  lcd.print(PWM_Left);
  lcd.print("  R:");
  lcd.print(PWM_Right);
  #endif
  
  SetPins_ToGoForwards();
  Motor_Action_Dynamic_PWM_Steering();
  delay(2000);

  
  Motor_Action_Stop_Motors();  
  delay(1000);

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.print("Test Finished");
  delay(1000);
  lcd.clear();
  #endif

  I = 2;
  }
  Turn_Off_Relay();
  delay(200);

  Serial.println(F("Wheel Test Complete"));
}     



void Test_Mower_Blade_Motor() {
  // Spin the blade motor for 7 seconds
  Turn_On_Relay();;
  delay(200);
  
  #if defined(LCD_KEYPAD)
  lcd.print("Blade Motor");
  lcd.setCursor(0,1);
  lcd.print("Test..!!");
  delay(1000);
  lcd.clear();
  lcd.print("Remove ALL");
  lcd.setCursor(0,1);
  lcd.print("Blades!!!");
  delay(4000);
  lcd.clear();
  delay(2000);
  lcd.print("BLADE MOTOR");
  delay(500);
  #endif
  
  Serial.println("Blades ON");
  delay(2000);
  
  #if defined(LCD_KEYPAD)
  lcd.setCursor(0,1);
  lcd.print("ON ");
  lcd.setCursor(6,1);
  lcd.print("PWM =");
  lcd.print(PWM_Blade_Speed);
  #endif
  
  Motor_Action_Spin_Blades();
  delay(7000);


  // Stop the blade motor spinning for 2 seconds

  Serial.println("Blades OFF");

  #if defined(LCD_KEYPAD)
  lcd.clear();
  lcd.print("BLADE MOTOR");
  lcd.setCursor(0,1);
  lcd.print("OFF..  ");
  #endif
  
  Motor_Action_Stop_Spin_Blades();
  delay(2000);
  
  #if defined(LCD_KEYPAD)
  lcd.clear();
  #endif
  
  delay(500);

  Turn_Off_Relay();
  delay(200);

  }


void Test_Sonar_Array()   {

  //Clears the Trig Pin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin3, LOW);


  /*Fires all Sonars to detect objects ahead...
   * Sonars are not fired in order to avoid reflections of sonar in the next sensor.
     distance# reurned (trigpin#, echopin#, distance#, duration#, Sonar#, LCDColumn#, LCD Row#)
   *********************************************************************************************/
    if (Sonar_1_Activate) distance1 = PingSonarY(trigPin1, echoPin1, 1, 1, 1, 5, 0);          //SONAR1
    delay(15);
    if (Sonar_2_Activate) distance2 = PingSonarY(trigPin2, echoPin2, 2, 2, 2, 0, 0);         //SONAR2
    delay(15);
    if (Sonar_3_Activate) distance3 = PingSonarY(trigPin3, echoPin3, 3, 3, 3, 10, 0);          //SONAR3
    delay(15);
  }





/* SONAR Function
************************************************************************************/
// Function to Ping the Sonar calculate the distance from Object to the Sonars.
// Distance calculated is printed to serial printer and displays X or _ on the LCD Screen
// Distance calculated is then used for the object avoidance logic
// Sonars used can be activated in the settings.

int PingSonarY(int trigPinY, int echoPinY, int distanceY, long durationY, int sonarY, int LCDRow, int LCDColumn) {
  pinMode(trigPinY, OUTPUT);
  pinMode(echoPinY, INPUT);
  //Sets the trigPin at High state for 10 micro secs sending a sound wave
  digitalWrite(trigPinY, HIGH);
  digitalWrite(trigPinY, LOW);
  delayMicroseconds(10);

  /*Reads the echoPin for the bounced wave and records the time in microseconds*/
  durationY = pulseIn(echoPinY, HIGH);

  /*Calculates the distance in cm based on the measured time*/
  distanceY = durationY * 0.034 / 2;
  delay(5);

  /* If a 0 distance is measured normally the Sonar ping has not been received.
    distance is then set to 999cm so the missed ping is not seen as an object detected.*/
  if (distanceY == 0) {
    distanceY = 999;
    Serial.print(F("SONAR "));
    Serial.print(sonarY);
    Serial.print(": ");
    Serial.println(F("NO PING ERROR REMOVED"));
  }

  /*Prints the Sonar letter and distance measured on the serial Monitor*/
  Serial.print(F("SONAR "));
  Serial.print(sonarY);
  Serial.print(": ");
  Serial.print(distanceY);
  Serial.println(F(" cm"));
  //Serial.println(maxdistancesonar);

  /*If sonar distance is less than maximum distance then an object is registered to avoid*/
  if (distanceY <= maxdistancesonar ) {
    //Prints that Sonar X has detected an object to the Mower LCD.
    
    #if defined(LCD_KEYPAD)
    lcd.setCursor(LCDRow, LCDColumn);                //sets location for text to be written
    lcd.print("X");
    LCDColumn = LCDColumn + 1;    
    lcd.setCursor(LCDRow, LCDColumn);                //sets location for text to be written
    lcd.print("   ");
    lcd.setCursor(LCDRow, LCDColumn);
    lcd.print(distanceY);
    #endif
    
    delay(10);
  }

  /*If sonar distance is greater than maximum distance then no object is registered to avoid*/
  if (distanceY > 100) {
    //Prints that the path of Sonar X is open.
    #if defined(LCD_KEYPAD)
    LCDColumn = LCDColumn - 1;   
    lcd.setCursor(LCDRow, LCDColumn);                 //sets location for text to be written
    lcd.print("_");
    delay(10);
    #endif
  }

  return distanceY;
  return sonarY;

}


void Test_Compass_Turn_Function() {
    Turn_On_Relay();
    delay(200);
    SetPins_ToGoForwards();
    Motor_Action_Go_Full_Speed();
    delay(2000);
    Manouver_Turn_Around();
    Turn_To_Compass_Heading();
    SetPins_ToGoForwards();
    Motor_Action_Go_Full_Speed();
    delay(2000); 
    Turn_Off_Relay();
    }
    

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Time_Alarms
// Access to the PCB onboard Clock and arduino clock Module
// Alarm actions.


void Get_Current_Time_Print_On_Serial_Monitor() {

if (Mower_PIXHAWK == 0) {
      
      if (PCB == 0) {
        Time t = rtc.time();
        Time_Year = t.yr;
    Time_Month = t.mon;
    Time_Date = t.date;
        Time_Hour = t.hr;
        Time_Minute = t.min;
        Time_Second = t.sec;
        Time_Day = t.day;
        }
//      if (PCB == 1) {
//        Display_DS3231_Time();          // onboard RTC clock on the PCB board
//
//        }
/*
      Serial.print(F("Time:"));  
      if (Time_Hour < 10) Serial.print(F("0"));
      Serial.print(Time_Hour);
      Serial.print(F(":"));
      if (Time_Minute < 10) Serial.print(F("0"));
      Serial.print(Time_Minute);
      Serial.print(F(":"));
      if (Time_Second < 10) Serial.print(F("0"));
      Serial.print(Time_Second);
      Serial.print(F("|"));
*/
      PrintTimeToSerial(0, 0, 0, 0, Time_Hour, Time_Minute, Time_Second, 0, 0);
      }
}

// Arduino RTC Clock Module
void DisplayTime_DS1302()   {
  Serial.print(F("Time:"));
  Time t = rtc.time();  
 
  // Name the day of the week.
  const String day = dayAsString(t.day);

  // Format the time and date and insert into the temporary buffer.
  char buf[50];
  snprintf(buf, sizeof(buf), "%s %04d-%02d-%02d %02d:%02d:%02d",
           day.c_str(),
           t.yr, t.mon, t.date,
           t.hr, t.min, t.sec);

  // Print the formatted string to serial so we can see the time.
  Serial.print(buf);
 
  }


// Print Date and Time on serial console
void PrintTimeToSerial(byte _type, int _year, byte _month, byte _date, byte _hour, byte _minute, byte _second, byte _dow, bool _endType)   {
  // _type 0: time
  // _type 1: date time
  // _type 2: date time DoW

  // _endType 0: Serial.print(F("|"));
  // _endType 1: Serial.println("");


  if (_type > 0) Serial.print(F("DateTime: "));
  else Serial.print(F("Time:"));

  if (_type > 0) {
    Serial.print(_year);
    Serial.print(F("-"));
    if (_month < 10) Serial.print(F("0"));
    Serial.print(_month);
    Serial.print(F("-"));
    if (_date < 10) Serial.print(F("0"));
    Serial.print(_date);
    Serial.print(F(" "));
  }
    if (_hour < 10) Serial.print(F("0"));
    Serial.print(_hour);
    Serial.print(F(":"));
    if (_minute < 10) Serial.print(F("0"));
    Serial.print(_minute);
    Serial.print(F(":"));
    if (_second < 10) Serial.print(F("0"));
    Serial.print(_second);

  if (_type == 2) {
    char buf[15];
    const String day = dayAsString((Time::Day)_dow);
    snprintf(buf, sizeof(buf), " %s", day.c_str());
    Serial.print(buf);
  }

  if (_type > 2) {
    Serial.print(F("bad _type on PrintTimeToSerial()"));
  }

  if (_endType) {
    Serial.println("");
  } else {
    Serial.print(F("|"));
  }
}




// PCB Clock
void Set_DS3231_Time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
    // sets time and date data to DS3231
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(0);                                // set next input to start at the seconds register
    Wire.write(decToBcd(second));                 // set seconds
    Wire.write(decToBcd(minute));                 // set minutes
    Wire.write(decToBcd(hour));                   // set hours
    Wire.write(decToBcd(dayOfWeek));              // set day of week (1=Sunday, 7=Saturday)
    Wire.write(decToBcd(dayOfMonth));             // set date (1 to 31)
    Wire.write(decToBcd(month));                  // set month
    Wire.write(decToBcd(year));                   // set year (0 to 99)
    Wire.endTransmission();
    }




// PCB Clock
void Display_DS3231_Time() {
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  Read_DS3231_PCB_Time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  static byte trigger = 1;
  Time_Hour = hour;
  Time_Minute = minute;
  Time_Second = second;
  Time_Year = year;
  Time_Month = month;
  Time_Date = dayOfMonth;
  Time_Day = dayOfWeek;

  if (second >= 30) //use second for once a minute, use minute for once an hour
  {
    if (trigger)
    {
    //digitalWrite(LED_BUILTIN, HIGH);
    trigger = 0;
    }
    else
    {
      trigger = 1;
    }
   
  }
  else
  {
    if (trigger)
    {
    //digitalWrite(LED_BUILTIN, LOW);
    trigger = 0;
    }
    else
    {trigger = 1;
    }
  }
}



// PCB Clock
void Read_DS3231_PCB_Time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                                                    // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);                          // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
  }



void Activate_Alarms() {
  
  // Manual ALARM 1
  if (Alarm_1_ON == 1) {  
     if ((Time_Hour== Alarm_1_Hour) && (Time_Minute == Alarm_1_Minute)) {
       Serial.println(F(""));
       Serial.println(F("ALARM 1 - Activated "));
       Serial.println(F(""));
       delay(2000);

        // Exit Dock : Zone 1
       if (Alarm_1_Action == 1) {
         Mow_Time(3);             // Set max mow time: 1=1hour, 2=2hours, 3=max mow time
         Alarm_Start_Exit_Zone_1();           // Exit Dock : Zone 1
       }
       if (Alarm_1_Action == 2) {
         Mow_Time(3);             // Set max mow time: 1=1hour, 2=2hours, 3=max mow time
         Alarm_Start_Exit_Zone_2();           // Exit Dock : Zone 2
       }
       if (Alarm_1_Action == 3) {                                    // Mow The Line
        Alarm_Start_Exit_Zone_2();
        Blade_Override = 1;
        } 
       if (Alarm_1_Action == 4) Alarm_Start_Quick_Go();              // Quick Go

       
       if (Alarm_1_Action == 5) {                                    // Custom Option
        Serial.println(F("Insert Custom Code"));
         }      
       }
    }
 
  
  // Manual ALARM 2
  if (Alarm_2_ON == 1) {  
     if ((Time_Hour == Alarm_2_Hour) && (Time_Minute == Alarm_2_Minute)) {
       Serial.println(F(""));
       Serial.println(F("ALARM 2 - Activated "));
       Serial.println(F(""));
       delay(2000);

        // Exit Dock : Zone 1
       if (Alarm_2_Action == 1) {
         Mow_Time(3);             // Set max mow time: 1=1hour, 2=2hours, 3=max mow time
         Alarm_Start_Exit_Zone_1();           // Exit Dock : Zone 1
       }
       if (Alarm_2_Action == 2) {
         Mow_Time(3);             // Set max mow time: 1=1hour, 2=2hours, 3=max mow time
         Alarm_Start_Exit_Zone_2();           // Exit Dock : Zone 2
       }
       if (Alarm_2_Action == 3) {                                    // Mow The Line
        Alarm_Start_Exit_Zone_2();
        Blade_Override = 1;
        }
       if (Alarm_2_Action == 4) Alarm_Start_Quick_Go();              // Quick Go

       
       if (Alarm_2_Action == 5) {                                    // Custom Option
        Serial.println(F("Insert Custom Code"));
         }      
       }
    }
     
  // Manual ALARM 3
  if (Alarm_3_ON == 1) {  
     if ((Time_Hour == Alarm_3_Hour) && (Time_Minute == Alarm_3_Minute)) {
       Serial.println(F(""));
       Serial.println(F("ALARM 3 - Activated "));
       Serial.println(F(""));
       delay(2000);

        // Exit Dock : Zone 1
       if (Alarm_3_Action == 1) {
         Mow_Time(3);             // Set max mow time: 1=1hour, 2=2hours, 3=max mow time
         Alarm_Start_Exit_Zone_1();           // Exit Dock : Zone 1
       }
       if (Alarm_3_Action == 2) {
         Mow_Time(3);             // Set max mow time: 1=1hour, 2=2hours, 3=max mow time
         Alarm_Start_Exit_Zone_2();           // Exit Dock : Zone 2
       }
       if (Alarm_3_Action == 3) {                                    // Mow The Line
        Alarm_Start_Exit_Zone_2();
        Blade_Override = 1;
        }
       if (Alarm_3_Action == 4) Alarm_Start_Quick_Go();              // Quick Go

       
       if (Alarm_3_Action == 5) {                                    // Custom Option
        Serial.println(F("Insert Custom Code"));
         }      
       }
    }

}


void Alarm_Start_Exit_Zone_2() {
         Serial.print(F("Exit Dock| "));
         Serial.println(F("Zone 2"));
       
         #if defined(LCD_KEYPAD)
           lcd.clear();
           lcd.print(F("Alarm Start"));
           lcd.setCursor(0,1);
           lcd.print(F("Exit Dock Z2"));
           delay(500);
           lcd.clear();
           #endif
       
         Mower_Parked = 0;
         Exiting_Dock = 2;
         Mower_Error_Value = 0;
         Send_Mower_Docked_Data();                                   // Send the Docked TX Data package to the mower.       
         Exit_Zone = 2;
         Track_Wire_Itterations = Track_Wire_Zone_1_Cycles;
         Exiting_Dock = 1;
         Manouver_Exit_To_Zone_X();    
         }



void Alarm_Start_Exit_Zone_1() {
  
         Serial.print(F("Exit Dock| "));
         Serial.println(F("Zone 1"));
       
         #if defined(LCD_KEYPAD)
           lcd.clear();
           lcd.print(F("Alarm Start"));
           lcd.setCursor(0,1);
           lcd.print(F("Exit Dock Z1"));
           delay(500);
           lcd.clear();
           #endif
       
         Exiting_Dock = 1;
         Mower_Error_Value = 0;
         Send_Mower_Docked_Data();                                   // Send the Docked TX Data package to the mower.       
         Exit_Zone = 1;
         Track_Wire_Itterations = Track_Wire_Zone_1_Cycles;
         Manouver_Exit_To_Zone_X();     
         }


 void Alarm_Start_Quick_Go() {
   Serial.println(F("Alarm - Quick Start"));
   #if defined(LCD_KEYPAD)
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print(F("WIFI Start"));
     #endif
   

   if (Mower_Docked == 0) { 
     Manouver_Start_Mower();
     #if defined(LCD_KEYPAD)
     lcd.clear();   
     #endif 
     if (TFT_Screen_Menu == 1) Send_Mower_Docked_Data();    // Send the Docked TX Data package to the mower.
     }    
   else Serial.println(F("Mower Docked - Quick Start not possible"));   
   }



// Set when choosing an option of 1hr or 2hr mow etc.
void Check_Timed_Mow() {
    
  if (Alarm_Timed_Mow_ON == 1) {  
     if ((Time_Hour == Alarm_Timed_Mow_Hour) && (Time_Minute == Alarm_Timed_Mow_Minute)) {
       Serial.println(F("Timed Mow Complete"));
       delay(2000);
       //Insert action for Timed Mow Alarm Here
         if (Use_Charging_Station == 1) Manouver_Go_To_Charging_Station();                       // Stops the mowing and sends the mower back to the charging station via the permieter wire
         if (Use_Charging_Station == 0) Manouver_Park_The_Mower_Low_Batt();                      // Parks the mower with a low battery warning
       }
     }
}
 

// Prints the alarms set to the serial monitor
void Display_Next_Alarm()  {
  //Print_Day();
  
  if (Alarm_1_ON == 1 ) {
      Serial.print(F("|Alarm 1:"));
      Serial.print(Alarm_1_Hour);
      Serial.print(F(":"));
      if (Alarm_1_Minute < 10) Serial.print ("0");
      Serial.print(Alarm_1_Minute);
      Serial.print("|");
      }
  if (Alarm_1_ON == 0) Serial.print("|A1arm_1:OFF");
 
  if (Alarm_2_ON == 1) {
  Serial.print(F("|Alarm 2:"));
  Serial.print(Alarm_2_Hour);
  Serial.print(F(":"));
  if (Alarm_2_Minute < 10) Serial.print ("0");
  Serial.print(Alarm_2_Minute);
  Serial.print("|");
  }
  if (Alarm_2_ON == 0) Serial.print("|Alarm_2:OFF");
  
  if (Alarm_3_ON == 1) {
  Serial.print(F("|Alarm 3:"));
  Serial.print(Alarm_3_Hour);
  Serial.print(F(":"));
  if (Alarm_3_Minute < 10) Serial.print ("0");
  Serial.print(Alarm_3_Minute);
  Serial.print("|");
  }
  if (Alarm_3_ON == 0) Serial.print("|Alarm_3:OFF");

   
}




void Manage_Alarms() {
    Alarm_Timed_Mow_ON = 0;                                           // Turns off the 1 hr Alarm
    if (Alarm_1_Repeat == 0) Alarm_1_ON = 0;
    if (Alarm_2_Repeat == 0) Alarm_2_ON = 0;
    if (Alarm_3_Repeat == 0) Alarm_3_ON = 0;
    }




// Arduino RTC Clock Module
//void Manual_Set_Time_On_DS1302(){
//   // Set_Time to 1 in the setting menu to set time.  Load the sketch then immediatley Set_Time = 0 and reload the sketch.
//   rtc.writeProtect(false);
//   rtc.halt(false);
//   Time t(2022, 4, 21, 21, 05, 00, Time::kThursday);            // Year XXXX, Month XX, Day XX, Hour XX, Minute XX, Second, kXYZday
//   rtc.time(t);    
//   delay(10);
//   rtc.writeProtect(true);
//   }

//void Manual_Set_Time_On_DS1307(){
//  rtc.set(0, 56, 12, 5, 13, 1, 22);
//  delay(10);
//}



// PCB Clock
void Manual_Set_Time_DS3231() {
    // Set_Time to 1 in the setting menu to set time.  Load the sketch then immediatley Set_Time = 0 and reload the sketch.
    Set_DS3231_Time(30,56,22,2,14,7,20);    //second, minute, hour, dayof week, day of month, month, year
    }


// Added function to be able set mow time for alarms - not possible to set over TFT yet
// Set max mow time: 1=1hour, 2=2hours, 3=max mow time
void Mow_Time(byte _mowTime) {
  Mow_Time_Set = _mowTime;

  Serial.print(F("Mow Time Selected = "));
  if ( (Mow_Time_Set == 1) || (Mow_Time_Set == 2 ) ){
    Serial.print(Mow_Time_Set);
    Serial.println(" hrs");
    }
  if (Mow_Time_Set == 3) Serial.println(F("Max Mow Time"));

  Serial.println(F(" "));

  if (Mow_Time_Set == 3) Alarm_Timed_Mow_ON = 0;
  if (Mow_Time_Set < 3) {
    if (PCB == 0) {Time t = rtc.time();}
    if (PCB == 1) Display_DS3231_Time();
    Alarm_Timed_Mow_ON = 1;                          // Activate the Mow Timer Alarm
    Alarm_Timed_Mow_Hour = Time_Hour +  Mow_Time_Set;     // Sets time to (+Mow_Time_Set 1 or 2 )hour later.
    Alarm_Timed_Mow_Minute = Time_Minute;                  // Minutes are the same

    Serial.print(F("Mow Time Ends: "));
    if (Alarm_Timed_Mow_Hour > 23) Alarm_Timed_Mow_Hour = Alarm_Timed_Mow_Hour - 24;
    if (Alarm_Timed_Mow_Hour < 10) Serial.print(F("0"));
    Serial.print(Alarm_Timed_Mow_Hour);
    Serial.print(F(":"));
    if (Alarm_Timed_Mow_Minute < 10) Serial.print(F("0"));
    Serial.print(Alarm_Timed_Mow_Minute);
  }
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Tip_Sensors
void Check_Tilt_Tip_Angle() {

  // CAREFUL Tilt_Angle_Sensed = 0 = ON   1 = OFF
  // Tilt_Orientation_Sensed =   1 = ON   0 = OFF


  // Check Angle Sensor
  if (Angle_Sensor_Enabled == 1)  {  
      Tilt_Angle_Sensed = digitalRead(Tilt_Angle);          // reads the Tilt Angle sensor
      // Check if the Tilt Sensor is active
      Serial.print("|A:");
      Serial.print(Tilt_Angle_Sensed); 
      if ((Mower_Running == 1) && (Tilt_Angle_Sensed == 0) && (Tilt_Orientation_Sensed == 0)) Take_Tilt_Action();
      }
  if (Angle_Sensor_Enabled == 0) {
    Tilt_Angle_Sensed = 1;         // 0 = ON   1 = OFF
    }
  
  // Check Tip Sensor
  if (Tip_Over_Sensor_Enabled == 1) {
    Tilt_Orientation_Sensed = digitalRead(Tilt_Orientation);    // reads the Tilt Orientation sensor
    Serial.print("|O:");
    Serial.print(Tilt_Orientation_Sensed);
    Serial.print("|"); 
    if ((Mower_Running == 1) && (Tilt_Orientation_Sensed == 1)) Take_Orientation_Action();
    }
  if (Tip_Over_Sensor_Enabled == 0) {
    Tilt_Orientation_Sensed = 0;   // 1 = ON   0 = OFF
    }

}


// Action if Angle Sensor is activated
void Take_Tilt_Action() {
  if (Angle_Sensor_Enabled == 1) {        
        Serial.println(" ");
        Serial.println("Tilt Sensed");
        Motor_Action_Stop_Spin_Blades();
        Motor_Action_Stop_Motors();
  #if not defined(NODELAY_BACKWARD)
        SetPins_ToGoBackwards();
  #else
        if (Mower_RunBack == 0) {
          SetPins_ToGoBackwards();
        }
        else {
          SetPins_ToGoForwards();
          Manouver_Turn_Around_Sonar_Phase = 0;
          Manouver_Turn_Around_Phase = 0;
        Mower_RunBack = 0;
        }
  #endif

        Motor_Action_Go_Full_Speed();
        delay(1500);
        Motor_Action_Stop_Motors();
        if (TFT_Screen_Menu == 1) {
            if (Robot_Type == 1) Send_Mower_Running_Data();
//            if (Robot_Type == 2) Send_Aerator_Running_Data();
            }
        delay(2000);
        
        bool Tip_Stop = true;
    
        while (Tip_Stop == true) {
          Check_Tilt_Tip_Angle();
          if (Tilt_Angle_Sensed == 1) Tip_Stop = false;
        }
    
        Serial.println("Turning Around After Tip Stop");
        SetPins_ToTurnRight(); 
        Motor_Action_Go_Full_Speed();
        delay (random(Mower_Turn_Delay_Min, Mower_Turn_Delay_Max));
        Motor_Action_Stop_Motors();
        }
    }


// Action if Tip Sensor is activated
void Take_Orientation_Action() {
    
    if (Tip_Over_Sensor_Enabled == 1)  {

          // Double check to make sure its not an error message
          Tilt_Orientation_Sensed = digitalRead(Tilt_Orientation);    // reads the Tilt Orientation sensor

          if (Tilt_Orientation_Sensed == 1) {   
              Serial.println(F(" ")); 
              Serial.println(F("***********************************"));
              Serial.println(F("***** TIP OVER PROTECTION !!! *****"));     
              Serial.println(F("***********************************"));
              Serial.println(F(" ")); 
              Motor_Action_Stop_Spin_Blades();
              Motor_Action_Stop_Motors();
              Tilt_Orientation_Sensed = 1;
              Manouver_Hibernate_Mower();
              delay(5000);
              }
           }
    }

    
//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// WIFI
// Blynk TX and RX functions to communicate with the MEGA and NODEMCU
// Each transmission has a label n or p or q or 

// USE V5 Blynk  12th Feb 2019

void Get_WIFI_Commands() {
  
  if (WIFI_Enabled == 1) {
      Receive_Data_From_NODEMCU(); 
      if (Mower_Setup_Mode != 1) Transmit_All_To_NODEMCU(); 
      if (Mower_Setup_Mode == 1) Transmit_Setup_Data_to_NODEMCU();
      }
  }

  
void Receive_Data_From_NODEMCU() {
  while(Serial2.available()>0){
      val_WIFI = Serial2.parseInt();
      if(Serial2.read()== '\p'){
        Execute_Blynk_Command_To_Mower();
        }
  }
 }



//**********************************

void Receive_Data_From_NODEMCU_2() {
  
  byte recvBuff [1] = {0};
  int recvVal = 0;

  if (SerialCom2.update ()) {
  
    byte length = SerialCom2.getLength ();
    if (length > sizeof (recvBuff)) length = sizeof (recvBuff);
    memcpy (&recvBuff, SerialCom2.getData (), length);
    
    val_WIFI = recvBuff[0];

  }

}


//**************************






void Receive_WIFI_Manual_Commands() {
  while(Serial2.available()>0){
      val_WIFI = Serial2.parseInt();
      if(Serial2.read()== '\p'){
        delay(5);
        Execute_Manual_Blynk_Command_To_Mower();
        }
  }
 }

void Transmit_All_To_NODEMCU() {

  Serial.print(F("|TX:WIFI"));

  #if not defined(WEBSERVER)
  byte data[7];
  #else
  byte data[53];
  #endif // -(WEBSERVER)-

  data[0] = Volts*100;
  data[1] = (int)(Volts*100) >> 8;
  data[2] = Loop_Cycle_Mowing;
  data[3] = Loop_Cycle_Mowing >> 8;
  bitWrite(data[4],0,Mower_Docked);
  bitWrite(data[4],1,Mower_Running);
  bitWrite(data[4],2,Mower_Parked);
  bitWrite(data[4],3,Tracking_Wire);
  bitWrite(data[4],4,Rain_Detected);
  bitWrite(data[4],5,Wire_Detected);
  bitWrite(data[4],6,0);
  data[5] = Charging;
  data[6] = Pattern_Mow;

  
  
  #if defined(WEBSERVER)
  //int AmpsTX = Amps * 100;
  //int Compass_TargetTX = Compass_Target * 100;

  data[12]   = Mower_RunBack;
  data[13]  = Robot_Status_Value;
  data[14]  = Mower_Error_Value;
  data[15]  = Compass_Steering_Status;
  float WheelAmpsTX1 = WheelAmps * 100;
  WheelAmpsTX = WheelAmpsTX1;
  data[16]  = WheelAmpsTX;
  data[17]  = WheelAmpsTX >> 8;
  Compass_Heading_DegreesTX = Compass_Heading_Degrees;
  data[18]  = Compass_Heading_DegreesTX;
  data[19]  = Compass_Heading_DegreesTX >> 8;
  data[20]  = Manouver_Turn_Around_Phase;
  data[21]  = Manouver_Turn_Around_Sonar_Phase;
  //data[22]= reserve;
  data[23]  = PWM_Right;
  data[24]  = PWM_Left;
  float Compass_Error = Compass_Heading_Degrees - Heading_Lock;
  if (Compass_Error > 180) Compass_Error = Compass_Error * - 1 ;
  if (Compass_Error < -180) Compass_Error = Compass_Error * - 1 ;
  int Compass_ErrorTX = (Compass_Error * 100) + 1000;
  data[25]=Compass_ErrorTX;
  data[26]=Compass_ErrorTX >> 8;

  int AmpsTX = (Amps * 100) + 1000;
  data[27]=AmpsTX;
  data[28]=AmpsTX >> 8;

  data[29]=distance1;
  data[30]=distance1 >> 8;
  data[31]=distance2;
  data[32]=distance2 >> 8;
  data[33]=distance3;
  data[34]=distance3 >> 8;

  bitWrite(data[35],0,Sonar_Status);
  bitWrite(data[35],1,Outside_Wire);
  bitWrite(data[35],2,Bumper);
  bitWrite(data[35],3,Tilt_Angle_Sensed);
  bitWrite(data[35],4,GPS_Inside_Fence);
  bitWrite(data[35],5,Tilt_Orientation_Sensed);
  bitWrite(data[35],6,Wheel_Blocked_Status);
  bitWrite(data[35],7,Wheels_Activate);

  bitWrite(data[36],1,Ramp_Motor_ON);
  bitWrite(data[36],2,MAG_Speed_Adjustment);
  bitWrite(data[36],3,Perimeter_Wire_Enabled);
  bitWrite(data[36],4,Use_Charging_Station);
  bitWrite(data[36],5,Blade_flagRun);
  bitWrite(data[36],6,Fake_All_Settings);
  bitWrite(data[36],7,Fake_Loops);

  bitWrite(data[37],1,Fake_Wire);
  bitWrite(data[37],2,Fake_WheelAmp);
  //bitWrite(data[34],4,);

  data[38] = Time_Year;
  data[39] = Time_Year >> 8;
  data[40] = Time_Month;
  data[41] = Time_Date;
  data[42] = Time_Hour;
  data[43] = Time_Minute;
  data[44] = Time_Second;
  data[45] = Time_Day;

  data[46] = Wheel_Status_Value;
  data[47] = MAG_Now;
  data[48] = MAG_Now >> 8;
  data[49] = Sonar_Hit_1_Total;
  data[50] = Sonar_Hit_2_Total;
  data[51] = Sonar_Hit_3_Total;
  data[52] = Low_Battery_Detected;

  #endif // -(WEBSERVER)-

  SerialCom2.sendMsg (data, sizeof (data));


  }



void Transmit_Setup_Data_to_NODEMCU() {

  Serial.print(F("|TX:SETUP"));
  byte data[3];

  bitWrite(data[0],0,Alarm_1_ON);                     // WIFI = 57 ON / 58 OFF
  bitWrite(data[0],1,Alarm_2_ON);                     // WIFI = 59 ON / 60 OFF
  bitWrite(data[0],2,Alarm_3_ON);                     // WIFI = 61 ON / 62 OFF
  bitWrite(data[0],3,Compass_Activate);               // WIFI = 40 ON / 41 OFF
  bitWrite(data[0],4,Compass_Heading_Hold_Enabled);   // WIFI = 42 ON / 43 OFF
  bitWrite(data[0],5,GYRO_Enabled);
  bitWrite(data[0],6,Sonar_1_Activate);
  bitWrite(data[0],7,Sonar_2_Activate);

  bitWrite(data[1],0,Sonar_3_Activate);
//  bitWrite(data[1],1,Bumper_Activate_Frnt);
  bitWrite(data[1],2,GPS_Enabled);
  bitWrite(data[1],3,Angle_Sensor_Enabled);
  bitWrite(data[1],4,Tip_Over_Sensor_Enabled);
  bitWrite(data[1],5,Wheel_Amp_Sensor_ON);
  bitWrite(data[1],6,Wheels_Activate);
  bitWrite(data[1],7,Ramp_Motor_ON);

  int Spare = 1;
  
  bitWrite(data[2],0,MAG_Speed_Adjustment);
  bitWrite(data[2],1,Perimeter_Wire_Enabled);
  bitWrite(data[2],2,Use_Charging_Station);
  bitWrite(data[2],3,Fake_All_Settings);
  bitWrite(data[2],4,Fake_Wire);
  bitWrite(data[2],5,Spare);
  bitWrite(data[2],6,Spare);
  bitWrite(data[2],7,Spare);
  
  SerialCom2.sendMsg (data, sizeof (data));

      Serial.print(F("|A1:"));
      Serial.print(Alarm_1_ON);
      Serial.print(F("|A2:"));
      Serial.print(Alarm_2_ON);
      Serial.print(F("|A3:"));
      Serial.print(Alarm_3_ON);
      Serial.print(F("|Comp:"));
      Serial.print(Compass_Activate);
      Serial.print(F("|HHold:"));
      Serial.print(Compass_Heading_Hold_Enabled);
      Serial.print(F("|GYRO:"));
      Serial.print(GYRO_Enabled);
      Serial.print(F("|S1:"));
      Serial.print(Sonar_1_Activate);
      Serial.print(F("|S2:"));
      Serial.print(Sonar_2_Activate);
      Serial.print(F("|S3:"));
      Serial.print(Sonar_3_Activate);
//      Serial.print(F("|Bump:"));
//      Serial.print(Bumper_Activate_Frnt); 
      Serial.print(F("|GPS:"));
      Serial.print(GPS_Enabled);
      Serial.print(F("|AngSens:"));
      Serial.print(Angle_Sensor_Enabled );
      Serial.print(F("|Tip:"));
      Serial.print(Tip_Over_Sensor_Enabled);
      Serial.print(F("|WheelAmp:"));
      Serial.print(Wheel_Amp_Sensor_ON);
      Serial.print(F("|Wheels:"));
      Serial.print(Wheels_Activate);
      Serial.print(F("|Ramp:"));
      Serial.print(Ramp_Motor_ON);    
      Serial.print(F("|MAG_Speed:"));
      Serial.print(MAG_Speed_Adjustment);       
      Serial.print(F("|WIRE:"));
      Serial.print(Perimeter_Wire_Enabled);     
      Serial.print(F("|Dock:"));
      Serial.print(Use_Charging_Station);     
  }









void Transmit_APP_Buttons_Status() {

}



void Execute_Blynk_Command_To_Mower() {

// Updates the Serial Monitor with the latest Blynk Commands and can be used to start
// functions on the mower when the command is recieved.
delay(30);


// Exit Dock to Zone 1
 if (val_WIFI == 14) {
       if (Mower_Docked == 1)   {  
    
       // Update the TFT Display
       Serial.println(F(""));
       Serial.println(F("Updating TFT: WIFI Exit dock"));
       Exiting_Dock = 1;
       Mower_Error_Value = 0;
       Send_Mower_Docked_Data();                                   // Send the Docked TX Data package to the mower.
         
       Serial.println(F(""));
       Serial.print(F("WIFI Command: ")); 
       Serial.print(val_WIFI);
       Serial.print(F("Exit Dock| "));
       Serial.println(F("Zone 1"));
       
       #if defined(LCD_KEYPAD)
       lcd.clear();
       lcd.print(F("WIFI Start"));
       lcd.setCursor(0,1);
       lcd.print(F("Exit Dock Z1"));
       delay(500);
       lcd.clear();
       #endif
       
       Exit_Zone = 1;
       Track_Wire_Itterations = Track_Wire_Zone_1_Cycles;
       Manouver_Exit_To_Zone_X();
       }
   if (Mower_Parked == 1) {
    Serial.println(F(""));
    Serial.println(F("Mower is Parked, not docked"));
   }
 }

 // Quick Start Button in Blynk App
 if (val_WIFI == 13) {
   Serial.println(F(""));
   Serial.print(F("WIFI Command:")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Quick Start"));
   
   #if defined(LCD_KEYPAD)
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print(F("WIFI Start"));
   #endif
   
   Serial.println(F("Quick Start"));
   if (Mower_Docked == 0) { 
     Manouver_Start_Mower();
     #if defined(LCD_KEYPAD)
     lcd.clear();   
     #endif 
     if (TFT_Screen_Menu == 1) Send_Mower_Docked_Data();    // Send the Docked TX Data package to the mower.
     }    
   else Serial.println(F("Mower Docked - Quick Start not possible"));   
   }

// Go To Dock Button in Blynk App
 if (val_WIFI == 12) {
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Go To Dock"));
   
   #if defined(LCD_KEYPAD)
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print(F("WIFI Go To Dock"));
   #endif
   
   Menu_Mode_Selection = 0;                                      // Releases the loop in the membrane button section.
   Motor_Action_Stop_Spin_Blades();
   Motor_Action_Stop_Motors();
   delay(500);
   
   #if defined(LCD_KEYPAD)
   lcd.clear();
   #endif
   
   Manouver_Go_To_Charging_Station();      
   }

// STOP / Cancel Button in Blynk App
 if (val_WIFI == 11)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Pause/Stop"));
   Manouver_Park_The_Mower(); 
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   }

// Manual Button in Blynk App
 if (val_WIFI == 15)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Manual Mode"));

   // Reset the TFT out of Setup Mode to Manual Mode
   if (Mower_Setup_Mode == 1) {
    Serial.print(F(" Setting RSV to 2 ")); 
    Robot_Status_Value = 6;
    Send_Mower_Setup_Data(); 
    }  
   
   Manouver_Park_The_Mower(); 
   Manouver_Manual_Mode(); 
   Turn_On_Relay(); 
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   }

 if (val_WIFI == 23)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Setup Mode"));
   Motor_Action_Stop_Spin_Blades();
   Manouver_Park_The_Mower(); 
   Manouver_Setup_Mode();  

   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   }

   
// Automatic RANDOM Button in Blynk App
 if (val_WIFI == 16)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Automatic Mode RANDOM"));
   
   #if defined(LCD_KEYPAD)
   lcd.clear();
   lcd.print("Auto Random");
   lcd.setCursor(0,1);
   lcd.print("Pattern");
   delay(200);
   lcd.clear();
   #endif

   Serial.print(F("RSV:"));
   Serial.print(Robot_Status_Value); 
   

   // Reset the TFT out of Setup Mode
   if (Mower_Setup_Mode == 1) {
    Serial.print(F(" Setting RSV to 2 ")); 
    Robot_Status_Value = 2;
    Send_Mower_Setup_Data(); 
    }   
   
   // Reset the TFT out of Manual Mode
   if (Manual_Mode == 1) {
    Serial.print(F(" Setting RSV to 2 ")); 
    Robot_Status_Value = 2;
    Send_Mower_Docked_Data(); 
    }  
   
   if (Mower_Running == 0) {
    Manouver_Park_The_Mower(); 
    Turn_On_Relay(); 
    }
   Pattern_Mow = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }

// Automatic SPIRAL Button in Blynk App
 if (val_WIFI == 21)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Automatic Mode SPIRAL"));
   
   #if defined(LCD_KEYPAD)
   lcd.clear();
   lcd.print("Auto Spiral");
   lcd.setCursor(0,1);
   lcd.print("Pattern");
   delay(200);
   lcd.clear();
   #endif
   
   if (Mower_Running == 0) {
    Manouver_Park_The_Mower(); 
    Turn_On_Relay(); 
    }
   Pattern_Mow = 2;

   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }

// Automatic Parallel Button in Blynk App
 if (val_WIFI == 22)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Automatic Mode PARALLEL"));
   
   #if defined(LCD_KEYPAD)
   lcd.clear();
   lcd.print("Auto Parallel");
   lcd.setCursor(0,1);
   lcd.print("Pattern");
   delay(200);
   lcd.clear();
   #endif
   
   if (Mower_Running == 0) {
    Manouver_Park_The_Mower(); 
    Turn_On_Relay(); 
    }
   Pattern_Mow = 1;

   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }


if (val_WIFI == 202)  {                    // letter d
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|Set to Docked Status"));   
   //Mower_Docked = 1 ;
   //Mower_Parked = 0 ;
   Manouver_Dock_The_Mower();  
   }



// SETTINGS MOWER
//**************************************

// Compass Setting Via WIFI.
 if (val_WIFI == 40)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Compass ON"));
   Compass_Activate = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(19 , 1);
   dueFlashStorage.write(20 , Compass_Activate);
   Serial.println(F("Saved to due FlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(19 , 1);
   EEPROM.write(20 , Compass_Activate);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }
 if (val_WIFI == 41)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Compass OFF"));
   Compass_Activate = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(19 , 1);
   dueFlashStorage.write(20 , Compass_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(19 , 1);
   EEPROM.write(20 , Compass_Activate);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }

// Heading Hold
 if (val_WIFI == 42)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Heading Hold ON"));
   Compass_Heading_Hold_Enabled = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(59 , 1);
   dueFlashStorage.write(60 , Compass_Heading_Hold_Enabled);
   Serial.println(F("Saved to due FlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(59 , 1);
   EEPROM.write(60 , Compass_Heading_Hold_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }
  
 if (val_WIFI == 43)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Heading Hold OFF"));
   Compass_Heading_Hold_Enabled = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(59 , 1);
   dueFlashStorage.write(60 , Compass_Heading_Hold_Enabled);
   Serial.println(F("Saved to due FlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(59 , 1);
   EEPROM.write(60 , Compass_Heading_Hold_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }

// GYRO
 if (val_WIFI == 44)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|GYRO ON"));
   GYRO_Enabled = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(109 , 1);
   dueFlashStorage.write(110, GYRO_Enabled);
   Serial.println(F("Saved to due FlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(109 , 1);
   EEPROM.write(110, GYRO_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }
  
 if (val_WIFI == 45)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|GYRO OFF"));
   GYRO_Enabled = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(109 , 1);
   dueFlashStorage.write(110, GYRO_Enabled);
   Serial.println(F("Saved to due FlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(109 , 1);
   EEPROM.write(110, GYRO_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }


// SONAR 1 Setting Via WIFI.
 if (val_WIFI == 47)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|SONAR 1 ON"));
   Sonar_1_Activate = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(37, 1);
   dueFlashStorage.write(38, Sonar_1_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(37, 1);
   EEPROM.write(38, Sonar_1_Activate);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }
   
 if (val_WIFI == 48)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|SONAR 1 OFF"));
   Sonar_1_Activate = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(37, 1);
   dueFlashStorage.write(38, Sonar_1_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(37, 1);
   EEPROM.write(38, Sonar_1_Activate);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }


// SONAR 2 Setting Via WIFI.
 if (val_WIFI == 49)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|SONAR 2 ON"));
   Sonar_2_Activate = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(39, 1);
   dueFlashStorage.write(40, Sonar_2_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(39, 1);
   EEPROM.write(40, Sonar_2_Activate);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }
  
 if (val_WIFI == 50)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|SONAR 2 OFF"));
   Sonar_2_Activate = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(39, 1);
   dueFlashStorage.write(40, Sonar_2_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(39, 1);
   EEPROM.write(40, Sonar_2_Activate);
   Serial.println(F("Saved to EEPROM"));
   #endif
   
   Serial.println(F(" "));
   }


// SONAR 3 Setting Via WIFI.
 if (val_WIFI == 51)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|SONAR 3 ON"));
   Sonar_3_Activate = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
   dueFlashStorage.write(41, 1);
   dueFlashStorage.write(42, Sonar_3_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(41, 1);
   EEPROM.write(42, Sonar_3_Activate);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif
   
   Serial.println(F(" "));
   }

  
 if (val_WIFI == 52)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|SONAR 3 OFF"));
   Sonar_3_Activate = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   
   #if defined(BOARD_DUE)
     dueFlashStorage.write(41, 1);
     dueFlashStorage.write(42, Sonar_3_Activate);
     Serial.println(F("Saved to dueFlashStorage"));
     #endif 

   #if defined(BOARD_MEGA)
     EEPROM.write(41, 1);
     EEPROM.write(42, Sonar_3_Activate);
     Serial.println(F("Saved to dueFlashStorage"));
     #endif 
   
   Serial.println(F(" "));
   }


// Bumper Setting Via WIFI.
// if (val_WIFI == 53)  {    
//   Serial.println(F(""));
//   Serial.print(F("WIFI Command: ")); 
//   Serial.print(val_WIFI);
//   Serial.println(F("|Bumper ON"));
//   Bumper_Activate_Frnt = 1;
//   Setup_Microswitches();
//   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
//
//   #if defined(BOARD_DUE)
//     dueFlashStorage.write(90 , 1);
//     dueFlashStorage.write(91 , Bumper_Activate_Frnt);
//     Serial.println(F("Saved to dueFlashStorage"));
//     #endif
//
//   #if defined(BOARD_MEGA)
//     EEPROM.write(90 , 1);
//     EEPROM.write(91 , Bumper_Activate_Frnt);
//     Serial.println(F("Saved to EEPROM"));
//     #endif
//
//   
//   Serial.println(F(" "));
//   }
//
//
//
// if (val_WIFI == 54)  {    
//   Serial.println(F(""));
//   Serial.print(F("WIFI Command: ")); 
//   Serial.print(val_WIFI);
//   Serial.println(F("|Bumper OFF"));
//   Bumper_Activate_Frnt = 0;
//   Setup_Microswitches();
//   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
//   
//   #if defined(BOARD_DUE)
//     dueFlashStorage.write(90 , 1);
//     dueFlashStorage.write(91 , Bumper_Activate_Frnt);
//     Serial.println(F("Saved to dueFlashStorage"));
//     #endif
//   
//   #if defined(BOARD_MEGA)
//     EEPROM.write(90 , 1);
//     EEPROM.write(91 , Bumper_Activate_Frnt);
//     Serial.println(F("Saved to EEPROM"));
//     #endif   
//   
//   
//   Serial.println(F(" "));
//   }


// Alarm 1 ON Setting Via WIFI.
 if (val_WIFI == 57)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Alarm 1 ON"));
   Alarm_1_ON = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
     dueFlashStorage.write(1 , 1);
     dueFlashStorage.write(4 , Alarm_1_ON);
     Serial.println(F("Saved to dueFlashStorage"));
     #endif

   #if defined(BOARD_MEGA)
     EEPROM.write(1 , 1);
     EEPROM.write(4 , Alarm_1_ON);
     Serial.println(F("Saved to EEPROM"));
     #endif

   
   Serial.println(F(" "));
   }


// Alarm 1 OFF Setting Via WIFI.
 if (val_WIFI == 58)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Alarm 1 OFF"));
   Alarm_1_ON = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
     dueFlashStorage.write(1 , 1);
     dueFlashStorage.write(4 , Alarm_1_ON);
     Serial.println(F("Saved to dueFlashStorage"));
     #endif

   #if defined(BOARD_MEGA)
     EEPROM.write(1 , 1);
     EEPROM.write(4 , Alarm_1_ON);
     Serial.println(F("Saved to EEPROM"));
     #endif

   
   Serial.println(F(" "));
   }




// Alarm 2 ON Setting Via WIFI.
 if (val_WIFI == 59)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Alarm 2 ON"));
   Alarm_2_ON = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
     dueFlashStorage.write(5 , 1);
     dueFlashStorage.write(8 , Alarm_2_ON);
     Serial.println(F("Saved to dueFlashStorage"));
     #endif

   #if defined(BOARD_MEGA)
     EEPROM.write(5 , 1);
     EEPROM.write(8 , Alarm_2_ON);
     Serial.println(F("Saved to EEPROM"));
     #endif

   
   Serial.println(F(" "));
   }


// Alarm 2 OFF Setting Via WIFI.
 if (val_WIFI == 60)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Alarm 2 OFF"));
   Alarm_2_ON = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
     dueFlashStorage.write(5 , 1);
     dueFlashStorage.write(8 , Alarm_2_ON);
     Serial.println(F("Saved to dueFlashStorage"));
     #endif

   #if defined(BOARD_MEGA)
     EEPROM.write(5 , 1);
     EEPROM.write(8 , Alarm_2_ON);
     Serial.println(F("Saved to EEPROM"));
     #endif

   
   Serial.println(F(" "));
   }



// Alarm 3 ON Setting Via WIFI.
 if (val_WIFI == 61)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Alarm 3 ON"));
   Alarm_3_ON = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(9 , 1);
   dueFlashStorage.write(12 , Alarm_3_ON);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(9 , 1);
   EEPROM.write(12 , Alarm_3_ON);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }


// Alarm 3 OFF Setting Via WIFI.
 if (val_WIFI == 62)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Alarm 3 OFF"));
   Alarm_3_ON = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(9 , 1);
   dueFlashStorage.write(12 , Alarm_3_ON);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(9 , 1);
   EEPROM.write(12 , Alarm_3_ON);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }



// GPS Enabled OFF Setting Via WIFI.
 if (val_WIFI == 65)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|GPS ON"));
   GPS_Enabled = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(107 , 1);
   dueFlashStorage.write(108 , GPS_Enabled);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(107 , 1);
   EEPROM.write(108 , GPS_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }


// GPS Enabled OFF Setting Via WIFI.
 if (val_WIFI == 66)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|GPS OFF"));
   GPS_Enabled = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(107 , 1);
   dueFlashStorage.write(108 , GPS_Enabled);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(107 , 1);
   EEPROM.write(108 , GPS_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }



// Angle Sensor
 if (val_WIFI == 67)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Angle Sensor ON"));
   Angle_Sensor_Enabled = 1;
   Setup_Tilt_Tip_Safety();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(29 , 1);
   dueFlashStorage.write(30 , Angle_Sensor_Enabled);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(29 , 1);
   EEPROM.write(30 , Angle_Sensor_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }


// Angle Sensor
 if (val_WIFI == 68)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Angle Sensor OFF"));
   Angle_Sensor_Enabled = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(29 , 1);
   dueFlashStorage.write(30 , Angle_Sensor_Enabled);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(29 , 1);
   EEPROM.write(30 , Angle_Sensor_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }



// Tip Sensor
 if (val_WIFI == 69)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Tip Sensor ON"));
   Tip_Over_Sensor_Enabled = 1;
   Setup_Tilt_Tip_Safety();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(92 , 1);
   dueFlashStorage.write(93 , Tip_Over_Sensor_Enabled);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(92 , 1);
   EEPROM.write(93 , Tip_Over_Sensor_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }


// Tip Sensor
 if (val_WIFI == 70)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Tip Sensor OFF"));
   Tip_Over_Sensor_Enabled = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(92 , 1);
   dueFlashStorage.write(93 , Tip_Over_Sensor_Enabled);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(92 , 1);
   EEPROM.write(93 , Tip_Over_Sensor_Enabled);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }



// Wheel Amp Sensor
 if (val_WIFI == 71)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Wheel Amp Sensor ON"));
   Wheel_Amp_Sensor_ON = 1;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)
   dueFlashStorage.write(115 , 1);
   dueFlashStorage.write(116 , Wheel_Amp_Sensor_ON);
   Serial.println(F("Saved to dueFlashStorage"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(115 , 1);
   EEPROM.write(116 , Wheel_Amp_Sensor_ON);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }


// Wheel Amp Sensor
 if (val_WIFI == 72)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Wheel Amp Sensor OFF"));
   Wheel_Amp_Sensor_ON = 0;
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once

   #if defined(BOARD_DUE)   
   dueFlashStorage.write(115 , 1);
   dueFlashStorage.write(116 , Wheel_Amp_Sensor_ON);
   Serial.println(F("Saved to DueFlash"));
   #endif

   #if defined(BOARD_MEGA)
   EEPROM.write(115 , 1);
   EEPROM.write(116 , Wheel_Amp_Sensor_ON);
   Serial.println(F("Saved to EEPROM"));
   #endif

   
   Serial.println(F(" "));
   }


 // Wheels_Activate
  if (val_WIFI == 101)  {
    Serial.println(F(""));
    Serial.print(F("WIFI Command: "));
    Serial.print(val_WIFI);
    Serial.println(F("|Wheels_Activate ON"));
    Wheels_Activate = 1;
    val_WIFI = 0;   // reset val2 to zero so the command is only executed once

    #if defined(BOARD_DUE)
    dueFlashStorage.write(123 , 1);
    dueFlashStorage.write(124 , Wheels_Activate);
    Serial.println(F("Saved to dueFlashStorage"));
    #endif

    #if defined(BOARD_MEGA)
    EEPROM.write(123, 1);
    EEPROM.write(124 , Wheels_Activate);
    Serial.println(F("Saved to EEPROM"));
    #endif

    Serial.println(F(" "));
    }

 // Wheels_Activate
  if (val_WIFI == 102)  {
    Serial.println(F(""));
    Serial.print(F("WIFI Command: "));
    Serial.print(val_WIFI);
    Serial.println(F("|Wheels_Activate OFF"));
    Wheels_Activate = 0;
    val_WIFI = 0;   // reset val2 to zero so the command is only executed once

    #if defined(BOARD_DUE)
    dueFlashStorage.write(123 , 1);
    dueFlashStorage.write(124 , Wheels_Activate);
    Serial.println(F("Saved to DueFlash"));
    #endif

    #if defined(BOARD_MEGA)
    EEPROM.write(123 , 1);
    EEPROM.write(124 , Wheels_Activate);
    Serial.println(F("Saved to EEPROM"));
    #endif

    Serial.println(F(" "));
    }


  // Ramp_Motor_ON
   if (val_WIFI == 103)  {
     Serial.println(F(""));
     Serial.print(F("WIFI Command: "));
     Serial.print(val_WIFI);
     Serial.println(F("|Ramp_Motor_ON"));
     Ramp_Motor_ON = 1;
     val_WIFI = 0;   // reset val2 to zero so the command is only executed once

     #if defined(BOARD_DUE)
     //dueFlashStorage.write(x , 1);
     //dueFlashStorage.write(x , Ramp_Motor_ON);
     //Serial.println(F("Saved to dueFlashStorage"));
     #endif

     #if defined(BOARD_MEGA)
     //EEPROM.write(x , 1);
     //EEPROM.write(x , Ramp_Motor_ON);
     //Serial.println(F("Saved to EEPROM"));
     #endif


     Serial.println(F(" "));
     }

  // Ramp_Motor_ON
   if (val_WIFI == 104)  {
     Serial.println(F(""));
     Serial.print(F("WIFI Command: "));
     Serial.print(val_WIFI);
     Serial.println(F("|Ramp_Motor_ON OFF"));
     Ramp_Motor_ON = 0;
     val_WIFI = 0;   // reset val2 to zero so the command is only executed once

     #if defined(BOARD_DUE)
     //dueFlashStorage.write(x , 1);
     //dueFlashStorage.write(x , Ramp_Motor_ON);
     //Serial.println(F("Saved to DueFlash"));
     #endif

     #if defined(BOARD_MEGA)
     //EEPROM.write(x , 1);
     //EEPROM.write(x , Ramp_Motor_ON);
     //Serial.println(F("Saved to EEPROM"));
     #endif


     Serial.println(F(" "));
     }



   // MAG_Speed_Adjustment
    if (val_WIFI == 105)  {
      Serial.println(F(""));
      Serial.print(F("WIFI Command: "));
      Serial.print(val_WIFI);
      Serial.println(F("|MAG_Speed_Adjustment ON"));
      MAG_Speed_Adjustment = 1;
      val_WIFI = 0;   // reset val2 to zero so the command is only executed once

      #if defined(BOARD_DUE)
      //dueFlashStorage.write(x , 1);
      //dueFlashStorage.write(x , MAG_Speed_Adjustment);
      //Serial.println(F("Saved to dueFlashStorage"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM.write(x , 1);
      //EEPROM.write(x , MAG_Speed_Adjustment);
      //Serial.println(F("Saved to EEPROM"));
      #endif


      Serial.println(F(" "));
      }

   // MAG_Speed_Adjustment
    if (val_WIFI == 106)  {
      Serial.println(F(""));
      Serial.print(F("WIFI Command: "));
      Serial.print(val_WIFI);
      Serial.println(F("|MAG_Speed_Adjustment OFF"));
      MAG_Speed_Adjustment = 0;
      val_WIFI = 0;   // reset val2 to zero so the command is only executed once

      #if defined(BOARD_DUE)
      //dueFlashStorage.write(x , 1);
      //dueFlashStorage.write(x , MAG_Speed_Adjustment);
      //Serial.println(F("Saved to DueFlash"));
      #endif

      #if defined(BOARD_MEGA)
      //EEPROM.write(x , 1);
      //EEPROM.write(x , MAG_Speed_Adjustment);
      //Serial.println(F("Saved to EEPROM"));
      #endif


      Serial.println(F(" "));
      }

    // Perimeter_Wire_Enabled
     if (val_WIFI == 107)  {
       Serial.println(F(""));
       Serial.print(F("WIFI Command: "));
       Serial.print(val_WIFI);
       Serial.println(F("|Perimeter_Wire_Enabled ON"));
       Perimeter_Wire_Enabled = 1;
       val_WIFI = 0;   // reset val2 to zero so the command is only executed once

       #if defined(BOARD_DUE)
       dueFlashStorage.write(67, 1);
       dueFlashStorage.write(68, Perimeter_Wire_Enabled);
       Serial.println(F("Saved to dueFlashStorage"));
       #endif

       #if defined(BOARD_MEGA)
       EEPROM.write(67, 1);
       EEPROM.write(68, Perimeter_Wire_Enabled);
       Serial.println(F("Saved to EEPROM"));
       #endif

       Serial.println(F(" "));
       }

    // Perimeter_Wire_Enabled
     if (val_WIFI == 108)  {
       Serial.println(F(""));
       Serial.print(F("WIFI Command: "));
       Serial.print(val_WIFI);
       Serial.println(F("|Perimeter_Wire_Enabled OFF"));
       Perimeter_Wire_Enabled = 0;
       val_WIFI = 0;   // reset val2 to zero so the command is only executed once

       #if defined(BOARD_DUE)
       dueFlashStorage.write(67, 1);
       dueFlashStorage.write(68, Perimeter_Wire_Enabled);
       Serial.println(F("Saved to DueFlash"));
       #endif

       #if defined(BOARD_MEGA)
       EEPROM.write(67, 1);
       EEPROM.write(68, Perimeter_Wire_Enabled);
       Serial.println(F("Saved to EEPROM"));
       #endif

       Serial.println(F(" "));
       }

     // Use_Charging_Station
      if (val_WIFI == 109)  {
        Serial.println(F(""));
        Serial.print(F("WIFI Command: "));
        Serial.print(val_WIFI);
        Serial.println(F("|Use_Charging_Station ON"));
        Use_Charging_Station = 1;
        val_WIFI = 0;   // reset val2 to zero so the command is only executed once

        #if defined(BOARD_DUE)
        dueFlashStorage.write(47, 1);
        dueFlashStorage.write(48, Use_Charging_Station);
        Serial.println(F("Saved to dueFlashStorage"));
        #endif

        #if defined(BOARD_MEGA)
        EEPROM.write(47, 1);
        EEPROM.write(48, Use_Charging_Station);
        Serial.println(F("Saved to EEPROM"));
        #endif

        Serial.println(F(" "));
        }

     // Use_Charging_Station
      if (val_WIFI == 110)  {
        Serial.println(F(""));
        Serial.print(F("WIFI Command: "));
        Serial.print(val_WIFI);
        Serial.println(F("|Use_Charging_Station OFF"));
        Use_Charging_Station = 0;
        val_WIFI = 0;   // reset val2 to zero so the command is only executed once

        #if defined(BOARD_DUE)
        dueFlashStorage.write(47, 1);
        dueFlashStorage.write(48, Use_Charging_Station);
        Serial.println(F("Saved to DueFlash"));
        #endif

        #if defined(BOARD_MEGA)
        EEPROM.write(47, 1);
        EEPROM.write(48, Use_Charging_Station);
        Serial.println(F("Saved to EEPROM"));
        #endif

        Serial.println(F(" "));
        }

#if defined(WEBSERVER)

// *** Fake values control ***

    // Fake_All_Settings
     if (val_WIFI == 120 || val_WIFI == 121)  {
       Serial.println(F(""));
       Serial.print(F("WIFI Command: "));
       Serial.print(val_WIFI);
       Serial.print(F("|Fake_All_Settings"));
       if (val_WIFI == 120) {
         Serial.println(F(" ON"));
         Fake_All_Settings = 1;
       }
       else if (val_WIFI == 121) {
         Serial.println(F(" OFF"));
           Fake_All_Settings = 0;
       }
       val_WIFI = 0;   // reset val2 to zero so the command is only executed once
       Serial.println(F(" "));
       }

     // Fake_Loops
      if (val_WIFI == 122 || val_WIFI == 123)  {
        Serial.println(F(""));
        Serial.print(F("WIFI Command: "));
        Serial.print(val_WIFI);
        Serial.print(F("|Fake_Loops"));
        if (val_WIFI == 122) {
         Serial.println(F(" ON"));
        Fake_Loops = 1;
        }
        else if (val_WIFI == 123) {
         Serial.println(F(" OFF"));
        Fake_Loops = 0;
        }
        val_WIFI = 0;   // reset val2 to zero so the command is only executed once
        Serial.println(F(" "));
        }

      // Fake_Wire
       if (val_WIFI == 124 || val_WIFI == 125)  {
         Serial.println(F(""));
         Serial.print(F("WIFI Command: "));
         Serial.print(val_WIFI);
         Serial.print(F("|Fake_Wire"));
         if (val_WIFI == 125) {
           Serial.println(F(" ON"));
         Fake_Wire = 1;
         }
         else if (val_WIFI == 125) {
           Serial.println(F(" OFF"));
         Fake_Wire = 0;
         }
         val_WIFI = 0;   // reset val2 to zero so the command is only executed once
         Serial.println(F(" "));
         }

       // Fake_WheelAmp
        if (val_WIFI == 126 || val_WIFI == 127)  {
          Serial.println(F(""));
          Serial.print(F("WIFI Command: "));
          Serial.print(val_WIFI);
          Serial.print(F("|Fake_WheelAmp"));
          if (val_WIFI == 126) {
           Serial.println(F(" ON"));
           Fake_WheelAmp = 1;
          }
          else if (val_WIFI == 127) {
           Serial.println(F(" OFF"));
           Fake_WheelAmp = 0;
          }
          val_WIFI = 0;   // reset val2 to zero so the command is only executed once
          Serial.println(F(" "));
          }

// *** Arduino software reset ***
        // Mega_SW_restart
  if (val_WIFI == 210)  {
    Serial.println(F(""));
    Serial.print(F("WIFI Command: "));
    Serial.print(val_WIFI);
    Serial.print(F("|Mega_SW_restart"));
    #if defined(WDT) and defined(BOARD_MEGA)
    Mega_SW_restart_En = true;
    #endif // -(WDT)-
    val_WIFI = 0;   // reset val2 to zero so the command is only executed once
    Serial.println(F(" "));
  }

    // TFT_SW_restart
  if (val_WIFI == 211)  {
    Serial.println(F(""));
    Serial.print(F("WIFI Command: "));
    Serial.print(val_WIFI);
    Serial.print(F("|TFT_SW_restart"));

    val_WIFI = 0;   // reset val2 to zero so the command is only executed once
    Serial.println(F(" "));
  }

#endif // -(WEBSERVER)-

}






void Execute_Manual_Blynk_Command_To_Mower() {
// insert wheel motions here.

 if (val_WIFI == 16)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Automatic Mode"));
   Manouver_Park_The_Mower(); 
   Turn_On_Relay(); 
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }

 if (val_WIFI == 23)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI Command: ")); 
   Serial.print(val_WIFI);
   Serial.println(F("|Setup Mode"));
   Turn_Off_Relay(); 
   Motor_Action_Stop_Spin_Blades();
   Manouver_Park_The_Mower(); 
   Manouver_Setup_Mode();  

   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   }

 if (val_WIFI == 17)  {    
   Serial.print(F("WIFI")); 
   Serial.print(val_WIFI);
   Serial.print(F("|Wheel Forward"));
   SetPins_ToGoForwards(); 
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   delay(300);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }

 if (val_WIFI == 18)  {    
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|Wheel Reverse"));
   SetPins_ToGoBackwards(); 
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   delay(300);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }

 if (val_WIFI == 19)  {    
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|Wheel Left"));
   SetPins_ToTurnLeft();
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed(); 
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   if (Robot_Type == 1) delay(200);
//   if (Robot_Type == 2) delay(800);
   
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }

 if (val_WIFI == 20)  {    
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|Wheel Right"));
   SetPins_ToTurnRight();
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   if (Robot_Type == 1) delay(200);
//   if (Robot_Type == 2) delay(800);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }


 
if (val_WIFI == 200)  {    
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|BLADE OFF"));
   Motor_Action_Stop_Spin_Blades();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
   }

if (val_WIFI == 201)  {    
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|BLADE ON"));
   Motor_Action_Spin_Blades();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
  }


if (val_WIFI == 80) {
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|MECANUM Left Shift"));
   Mecanum_Side_Movement_Left();
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   if (Robot_Type == 1) delay(200);
//   if (Robot_Type == 2) delay(800);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
}

if (val_WIFI == 81) {
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|MECANUM Right Shift"));
   Mecanum_Side_Movement_Right();
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   if (Robot_Type == 1) delay(200);
//   if (Robot_Type == 2) delay(800);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
}


if (val_WIFI == 82) {
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|MECANUM Left Diagonal"));
   Mecanum_Diagonal_Movement_Left();
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   if (Robot_Type == 1) delay(200);
//   if (Robot_Type == 2) delay(800);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
}

if (val_WIFI == 83) {
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.print(F("|MECANUM Right Diagonal"));
   Mecanum_Diagonal_Movement_Right();
   bool Temp_Ramp_Motor_ON = Ramp_Motor_ON;                                              // Save Ramp_Motor status
   Ramp_Motor_ON = 0;                                                                    // Turn off ramp motor option to get fast response
   Motor_Action_Go_Full_Speed();
   if (Temp_Ramp_Motor_ON == 1) (Ramp_Motor_ON = 1);                                     // Turn Ramp Motor ON or OFF again
   if (Temp_Ramp_Motor_ON == 0) (Ramp_Motor_ON = 0);
   if (Robot_Type == 1) delay(200);
//   if (Robot_Type == 2) delay(800);
   Motor_Action_Stop_Motors();
   val_WIFI = 0;   // reset val2 to zero so the command is only executed once
}


 if (val_WIFI == 56)  {    
   Serial.println(F(""));
   Serial.print(F("WIFI:")); 
   Serial.print(val_WIFI);
   Serial.println(F("Dock in this direction"));
   Serial.println(F(""));

   bool Keep_Compass_Value = 0;
   if (Compass_Activate == 1) {
    Compass_Activate = 0;
    Keep_Compass_Value = 1;
    }

   // Compass is deactivated before to ensure the mower docks in this direction.
   Manouver_Go_To_Charging_Station();
   
   if (Keep_Compass_Value == 1) {
    Compass_Activate = 1;
    }


  }

}

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Wheel_Amps
void Calculate_Wheel_Amps() {
  // Calculate Amp Value from Wheel Amp sensor
 int mVperAmp = 185;
 int ACSoffset = 2500; 
 double VoltageAmp = 0;
 //double Amps = 0;
 VoltageAmp = (RawWheelAmp / 1024.0) * 5000; // Gets you mV
 WheelAmps =  ((VoltageAmp - ACSoffset) / mVperAmp);
 if (WheelAmps < 0) WheelAmps = WheelAmps * - 1;
 if (WheelAmps > 10) WheelAmps = 0;
 if (Fake_WheelAmp) WheelAmps = 4.0; // RVES added
 Serial.print(("WA:"));
 Serial.print(WheelAmps);
 Serial.print(F("|"));

 }


void Check_Wheel_Amps() {
  Calculate_Wheel_Amps();
  if (Wheel_Amp_Sensor_ON == 1) {
    if (Mower_RunBack > 1) Wheel_Blocked_Count_Max = 10;  // CT during backward is very fast (<15ms), values are readed 2x than changed
    else Wheel_Blocked_Count_Max = 3;
      
        if (WheelAmps >= Max_Wheel_Amps) {
            Wheel_Blocked_Count = Wheel_Blocked_Count + 1;
            if (Wheel_Blocked_Count > Wheel_Blocked_Count_Max) {    
                Serial.println(F(""));
                Serial.print(F("   Wheel Amps: "));
                Serial.print(WheelAmps);
                Serial.print(F("   Wheel Blocked Count: "));
                Serial.print(Wheel_Blocked_Count);
                Serial.print(F("   Max Wheel Amps: "));
                Serial.println(Max_Wheel_Amps);                  
                Serial.println(F(""));
                Serial.println(F("!! Wheel_Blocked !!"));
                Serial.print(F("|")); 
                Wheel_Blocked = 4;
                Print_LCD_Wheel_Blocked(); // RVES added
        #if not defined(NODELAY_BACKWARD)
                Manouver_Turn_Around();
        #endif // -(NODELAY_BACKWARD)-
                }
           }
        else {
          Wheel_Blocked = 0;
          Wheel_Blocked_Count = 0;
          }

    #if defined(NODELAY_BACKWARD)
          if (Wheel_Blocked == 0) Wheel_Blocked_Status = false;
      if (Wheel_Blocked == 4) Wheel_Blocked_Status = true;
    #endif // -(NODELAY_BACKWARD)-
  }
}



void Test_Check_Wheel_Amps() {

        Serial.print(F("   Wheel Amps: "));
        Serial.print(WheelAmps);
        Serial.print(F("   Wheel Blocked Count: "));
        Serial.print(Wheel_Blocked_Count);
        Serial.print(F("   Max Wheel Amps: "));
        Serial.println(Max_Wheel_Amps);
        
        if (WheelAmps > Max_Wheel_Amps) {
            Wheel_Blocked_Count = Wheel_Blocked_Count + 1;
            if (Wheel_Blocked_Count > Wheel_Blocked_Count_Max) {
                Serial.println(F(""));
                Serial.print("!! Wheel_Blocked !!");
                Serial.print(F("|")); 
                Wheel_Blocked = 4;
                }
            }
        else {
          Wheel_Blocked = 0;
          Wheel_Blocked_Count = 0;
          }
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Wire_Detection
// Check the mower is inside (0) or outside (1) the perimeter wire
void Check_Wire_In_Out() {

  // If the perimeter wire sensor is de-activated
  if (Perimeter_Wire_Enabled == 0) {
      if (TFT_Screen_Menu == 1) {
        Outside_Wire = 0;
        //Data_Sent_Wire = 0;                                           // Resets the counter to send info to the TFT
        //Send_Mower_Running_Data();
        }    
      }


  // If the perimeter wire sensor is activated
  if (Perimeter_Wire_Enabled == 1) {
    UpdateWireSensor();                                               // Read the wire sensor and see of the mower is now  or outside the wire  
    ADCMan.run();
    PrintBoundaryWireStatus();        
    
    // OUTSIDE the wire
    if ( (perimeter.isInside(0)) == 0 && Fake_Wire == 0)  {                            // Mower is OUTSIDE the wire
      Outside_Wire = 1;                                               // Outside wire variable is tuend on.
      if (Mower_Running == 1) Motor_Action_Stop_Motors();             // Stop immediatley the wheel motors
      Print_LCD_Wire();                                               // Update the LCD screem
      Outside_Wire_Count = Outside_Wire_Count + 1;                    // Count the number of times the mower is consecutiley outside the wire
                                                                      // If a certain number is reached its assumed thw mower is lost outside the wire.
      }
    
    // INSIDE the wire
    if ( (perimeter.isInside(0)) == 1 )   {
      Outside_Wire = 0;                                               // Outside wire variable is tuend off
      Outside_Wire_Count = 0;                                         // The number of outside wire counts is reset to 0
      Wire_Refind_Tries = 0;
    }

    if ( Fake_All_Settings == 1) Execute_Fake_It();    
    if (Fake_Wire == 1) { Outside_Wire = 0; Outside_Wire_Count = 0; }
      
     
    }

  // If the mower consectivley detects its outside the wire a number of times
  // its assumed the mower is lost and a this search function is started to try and 
  // re-find the wire.  Otherwise the mower is hibernated.
  if (Outside_Wire_Count >= Outside_Wire_Count_Max) {
    if  (Action_On_Over_Wire_Count_Max == 1) Manouver_Hibernate_Mower();                  // Put the mower to sleep and wait
    if  (Action_On_Over_Wire_Count_Max == 2) Manouver_Outside_Wire_ReFind_Function();     // re-find Garden using Sonar 1 and wire detect
    
    if  (Action_On_Over_Wire_Count_Max == 3) {     // try to locate the wire using wire find function
      
      #if defined(LCD_KEYPAD)
      if (LCD_Screen_Keypad_Menu == 1) {
          lcd.clear();
          lcd.print("Wire Find");
          lcd.setCursor(0,1);
          lcd.print("Special Function");
          }
      #endif
      
      Loop_Cycle_Mowing = 999;
      if ((WIFI_Enabled == 1) && (Manual_Mode == 0)) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
      delay(2000);
      Outside_Wire_Count = 0;
      Specials_Find_Wire_Track();                  
      SetPins_ToGoBackwards();                                                              // Set the mower to back up
      Motor_Action_Go_Full_Speed(); 
      delay(1000);
      Motor_Action_Stop_Motors();  
      UpdateWireSensor();                                               // Read the wire sensor and see of the mower is now  or outside the wire  
      ADCMan.run();
      PrintBoundaryWireStatus(); 
      delay(1000);
      UpdateWireSensor();                                               // Read the wire sensor and see of the mower is now  or outside the wire  
      ADCMan.run();
      PrintBoundaryWireStatus(); 
      Wire_Refind_Tries = Wire_Refind_Tries + 1;
      Loop_Cycle_Mowing = 0;
      if (Manual_Mode == 0) Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
      Serial.println(F(""));
      Serial.print(F("|Wire Refind Atempts:"));
      Serial.print(Wire_Refind_Tries);
      Serial.print("|");
      Serial.println(F(""));
      if (Wire_Refind_Tries > 4) {
        Motor_Action_Stop_Motors(); 
        #if defined(LCD_KEYPAD) 
        lcd.clear(); 
        #endif
        Mower_Error = 1;
        Serial.println(F(""));
        Serial.println("Max refind tries exceeded - Parking the Mower");
        delay(2000);
      
      }
      
    }
  }
    
  }



//Check that boundary wire is turned on
//************************************************************************************
void Running_Test_for_Boundary_Wire()  {
  ADCMan.run();
  UpdateWireSensor();
  
  if (Perimeter_Wire_Enabled == 1) {                                               // Perimeter use is ON - Perimter_USE can be turned on or off in the setup.

    // Checks the MAG field of the boundary wire
    MAG_Now = perimeter.getMagnitude(0);

    // Only used for testing purposes
    if (Fake_All_Settings == 1) Execute_Fake_It();
    if (Fake_Wire == 1) MAG_Now = -500;
    Serial.print(F("|MAG:"));
    Serial.print(MAG_Now);



   
    // If the MAG field is very low between these values we can assume the wire is off
    if ( (MAG_Now > -20 ) && (MAG_Now < 20 )  ) {
      //Serial.print("WIRE Not Detected");
      Wire_Detected = 0;
      Print_LCD_NO_Wire();
      Wire_Off = Wire_Off + 1;
      }


    // if the MAG field is strong then the wire is on.
    if (   (MAG_Now <= -21) || (MAG_Now >= 20)  ) {
      Print_LCD_Wire_ON();  
      //Serial.print("WIRE Detected");
      Wire_Detected = 1;                                            // Wire is detected  
      Wire_Off = 0;                                                 // Resets the counter
      }
      
      // If the mower is docked or Parked then the TFT screen just shows a wire off warning
      if ( (Wire_Off > 5) && (Mower_Docked == 0) && (Mower_Parked == 0) ) {
        Serial.println(F("Wire Test Failed - Hibernating Mower"));
        Manouver_Hibernate_Mower();      // Send the mower to sleep        
        }
       

    Serial.print(F("|Wire"));
    Serial.print(F(":"));
    if (Wire_Detected == 0) Serial.print(F("OFF|"));
    if (Wire_Detected == 1) Serial.print(F("ON|"));
    }

  
  // If the wire is not enabled
  if (Perimeter_Wire_Enabled == 0) {   
    Serial.print(F("Wire"));
    Serial.print(F(":"));                                            
    Serial.print(F("DISABLED|"));
    Wire_Detected = 1;                // to override the wire we just always make it as ON
    Wire_Off = 0;

  }
}


void UpdateWireSensor()   {
  if (millis() >= nextTime)  {
    nextTime = millis() + 50;
    if (perimeter.isInside(0) != inside) {
      inside = perimeter.isInside(0);
      counter++;
    }
  }

  if ( Fake_All_Settings == 1 || Fake_Wire == 1) {
    inside = 1;
  }
}


void PrintBoundaryWireStatus() {

  Serial.print(F("|IN/OUT:"));
  Serial.print((perimeter.isInside(0)));
  Serial.print(F("|Mag:"));
  Serial.print((int)perimeter.getMagnitude(0));
  Serial.print(F("|Smag:"));
  Serial.print((int)perimeter.getSmoothMagnitude(0));
  Serial.print(F("|Q:"));
  Serial.print((perimeter.getFilterQuality(0)));
  Serial.print(F("|"));
    
  }



void Run_Initial_Boundary_Wire_Test() {
    
    Wire_Off      = 0;     // reset the wire off count.
    Wire_Detected = 0;     // reset wire detected
    
    if (Perimeter_Wire_Enabled == 1) {
        Mower_Track_To_Exit = 1;
        Serial.println(F("Testing Boundary Wire #1"));
        Running_Test_for_Boundary_Wire();
        delay(50);    
        Serial.print(F("MAG Detected = "));
        Serial.print(MAG_Now);   
        Serial.print(F(" | Wire Detected = "));
        Serial.print(Wire_Detected);           
        Serial.println(F(""));
        
        // If the boundary wire is turned off / not detected
        // Then the test boundary wire function will already put the mower into hibernate mode.
        
        if (Wire_Detected == 0) {
            Serial.println("Perimeter Wire not detected - Testing Again");
            Serial.println(F("Testing Boundary Wire #2"));
            delay(2000);
            Wire_Detected = 1;
            Running_Test_for_Boundary_Wire();                                     // Test again for the boundary wire
            Serial.print(F("MAG Detected = "));
            Serial.println(MAG_Now);                
                            
            if (Wire_Detected == 0) {                                 // if its still saying the wire is off then park the mower.
              Serial.println("Testing Boundary Wire #3");
              Serial.print(F("Wait .... 2secs"));
              delay(2000);      
              Wire_Detected = 1;
              Running_Test_for_Boundary_Wire();
              Serial.print(F("MAG Detected = "));
              Serial.println(MAG_Now);
              } 
              if (Wire_Detected == 0) {                                 // if its still saying the wire is off then park the mower.
                Serial.println("Perimeter Wire not detected - Last Try.....");
                Serial.print(F("Wait .... 2secs"));
                delay(2000);      
                Wire_Detected = 1;
                Running_Test_for_Boundary_Wire();
                Serial.print(F("MAG Detected = "));
                Serial.println(MAG_Now);
                } 
                
            }
         }      
      

  if (Perimeter_Wire_Enabled == 0) {
     Wire_Detected = 1;
     Serial.print(F("Wire"));
     Serial.print(F(":"));                                            
     Serial.print(F("DISABLED|"));
     Wire_Detected = 1;
     Wire_Off = 0;
  
  }

}

//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// Wire_Tracking
//  Prints a visual display of the wire tracking in the Serial Monitor
//  to see how well the wire is being followed.  Adjusting the P value in the settings
//  can improve the wire tracking ability of the mower.


void PrintWirePosition() {
  int PrintMAG_Now = MAG_Now / Scale;
  int PrintInMax = InMax / Scale;
  int PrintInMid = InMid / Scale;
  int PrintInMin = InMin / Scale;
  int PrintOutMin = OutMin / Scale;
  int PrintOutMid = OutMid / Scale;
  int PrintOutMax = OutMax / Scale;


  for (int i = PrintInMax; i <= PrintOutMax; i++) {
    if (i == PrintInMax) Serial.print(F("|"));
    if (i == PrintInMid) Serial.print(F("|"));
    if (i == PrintInMin) Serial.print(F("|"));
    if (i == PrintOutMin) Serial.print(F("|"));
    if (i == PrintOutMid) Serial.print(F("|"));
    if (i == PrintOutMax) Serial.print(F("|"));
    if (i == PrintMAG_Now) Serial.print(F("X"));     //maybe change to MAG_Lasr...
    if (i == 0) Serial.print(F("0"));

    else Serial.print(F("."));

  }
  Serial.print(F("|MAG_Now:"));  Serial.print(MAG_Now); Serial.print(F("|"));
}



// Function to follow the wire for a specific amount of time set by the itterations 'I'
// Robot tracks the wire until the itterations are exhausted.
void Track_Wire_From_Dock_to_Zone_X() {
  Serial.println(F("Tracking Wire to Zone X"));
 
  delay(100);
  ADCMan.run();
  Serial.println(F(""));
  Serial.println(F("Blade Override = "));
  Serial.print(Blade_Override);
  Serial.println(F(""));
  
  if (Blade_Override == 1) {
    Motor_Action_Spin_Blades();
    Blade_Override = 0;                                           // Cancels Blade Override for next time around
  }

      #if defined(LCD_KEYPAD)
          lcd.setCursor(0, 0);
          lcd.print(F("Exit Docking to"));                                             // into the garden at a good position to start Mowing
          lcd.setCursor(2, 1);
          if (Exit_Zone == 1) lcd.print(F("Zone 1"));
          if (Exit_Zone == 2) lcd.print(F("Zone 2"));
          delay(1000);                                           // Prints info to LCD display
          #endif
      
  Tracking_Turn_Right = 0;                                // resets the tracking errors for LH and RH.
  Tracking_Turn_Left = 0;
  
  //uses the PID settings in the setup
  Serial.print(F("P = "));
  Serial.print(P);
  Serial.print(F(" / D = "));
  Serial.print(D);
  Serial.println(F(""));
  Tracking_Wire = 1;
  Mower_Running = 0;
  MAG_Now = 0;                                                              // Reset Values
  MAG_Start = 0;
  MAG_Error = 0;
  MAG_Goal = 0;
  int Dock_Cycles = 0;
  delay(500);
  Get_WIFI_Commands();
  delay(5);
  
  #if defined(LCD_KEYPAD)
      lcd.setCursor(10,1);
      lcd.print("0");
      #endif

  while ((Mower_Parked == 0) && (Tracking_Wire == 1)) {
  
  for (I = 0; I < Track_Wire_Itterations; I++) {                                              // Iterations 'I' before mower leaves the wire.
      if (Mower_Parked == 0) {
      delay(5);
      ADCMan.run();
      MAG_Start = perimeter.getMagnitude(0);                                // Gets the signal strength of the sensor
      MAG_Now = MAG_Start;
      delay(5);
      MAG_Error = (MAG_Goal - MAG_Start);                                   // Calculates the Error to the center of the wire which is normally zero magnitude
      PrintWirePosition();                                                  // Prints the overview to the Serial Monitor.
      //Check_Wire_Blockage();                                              // homework idea is to avoid anything in the way

      // Tracks the wire from the docking station in a Counter-Clockwise direction to the start position

      if (CCW_Tracking_To_Start == 1) {
          if (MAG_Error > 0) {                                                  // If the MAG_Error > 0 then turn right for CCW Tracking. PWM_left is set to max to turn right.
            // TURN RIGHT
            PWM_Left = PWM_MaxSpeed_LH;                                         // sets the PWM to the max possible for the wheel
            PWM_Right = 255 - (MAG_Error * P);                                  // Mag_Error * P is the value reduced from the max value set PWM and sent to the PWM
            if (PWM_Right > 255)  PWM_Right = 255;                              // PWM_Right capped to Max PWM of 255.
            if (PWM_Right >= 0) {
              SetPins_ToGoForwards();
              
              #if defined(LCD_KEYPAD)
              lcd.setCursor(15, 1);
              lcd.print(" ");
              #endif
              
              }
    

       // Boost Turn RH
       if (Boost_Turn == 1) {
          if (PWM_Right < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnRight();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              } 
       }
    
            if (PWM_Right < 0) {                                                // sets the mower to sharp turn to the right (wheels spin opposite) if the Error to the wire is large enough.
              PWM_Right = (-1 * PWM_Right) + 220 ;
              if (PWM_Right > 255) PWM_Right = 255;
              if (PWM_Right >= 0) SetPins_ToTurnRight();
              delay(5);
    
              #if defined(LCD_KEYPAD)
              lcd.setCursor(15, 1);
              lcd.print(">");
              #endif
              
              }
            
            Motor_Action_Dynamic_PWM_Steering();                                      // Carries out the wheel PWM changes for steering on the wire
            Serial.print(F(" Turn Right "));
            Tracking_Turn_Left = 0;                                             // Failsafe if the mower looses the wire.  If the mower is commanded to turn left or right
            Tracking_Turn_Right = Tracking_Turn_Right + 1;                      // too many times it is assumed that the mower is spinning and cant get back on the wire.
            if (Tracking_Turn_Right > Max_Tracking_Turn_Right) {                // if this is detected then a function is ran to find the wire again.
              Motor_Action_Stop_Motors();
              
              #if defined(LCD_KEYPAD)
              lcd.clear();
              lcd.print(F("Right Wheel"));
              lcd.print(F("Tracking_Turn_Right"));
              delay(2000);
              lcd.clear();
              #endif
              
              Tracking_Restart_Blocked_Path();
            }
    
          }
          if (MAG_Error <= 0) {                                                 // If the MAG_Error < 0 then turn left for CCW Tracking
            // TURN LEFT
            PWM_Right = 255;                                                    // PWM_Right set to max to rotate the mower to the left.
            PWM_Left = 255 + (MAG_Error * P);                                   // + as mag_error is negative to adjust PWM
            if (PWM_Left > 255) PWM_Left = 255;                                 // PWM_Left capped to mex PWM of 255
            if (PWM_Left >= 0) {
              SetPins_ToGoForwards();   
              
              #if defined(LCD_KEYPAD)
              lcd.setCursor(0, 1);
              lcd.print(" "); 
              #endif
              
              }

       // Boost Turn LH
       if (Boost_Turn == 1) {
       if (PWM_Left < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnLeft();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              }
        }
    
            if (PWM_Left < 0) {                                                 // sets the mower to sharp turn to the left (wheels spin opposite) if the Error to the wire is large enough.
              PWM_Left = (-1 * PWM_Left) + 220 ;
              if (PWM_Left > 255) PWM_Left = 255;
              SetPins_ToTurnLeft();
              delay(5);
              
              #if defined(LCD_KEYPAD)
              lcd.setCursor(0, 1);
              lcd.print("<");
              #endif
              
              }
            
            Motor_Action_Dynamic_PWM_Steering();

            Serial.print(F(" Turn Left "));
            Tracking_Turn_Right = 0;
            Tracking_Turn_Left = Tracking_Turn_Left + 1;
            if (Tracking_Turn_Left > Max_Tracking_Turn_Left) {
              Motor_Action_Stop_Motors();
              
              #if defined(LCD_KEYPAD)
              lcd.clear();
              lcd.print("Left Wheel");
              lcd.print("Tracking_Turn_Left");
              delay(2000);
              lcd.clear();
              #endif
              
              Tracking_Restart_Blocked_Path();
            }
          }
          Serial.print(F(" : MAG_Error="));
          Serial.println(MAG_Error);
          Dock_Cycles = Dock_Cycles + 1;
          Loop_Cycle_Mowing = I;
          if (Dock_Cycles > 10) {
            Tracking_Wire = Tracking_Wire + 1;                            // Makes the wire tracking LED in the app blink.
            if (Tracking_Wire > 1) Tracking_Wire = 0;
            Get_WIFI_Commands();
            Dock_Cycles = 0;
            
            }
          }
    
  

  if (CW_Tracking_To_Start == 1) {
        // Add Code here for CW tracking to the exit zone.
        // Use the code above for CCW tracking to the docking station
        // as a template.
     if (MAG_Error > 0) {                                      // if MAG_Error > 0 then Turn left in CW tracking                      
          // Turn LEFT
          PWM_Right = 255;                                        // Set the right wheel to max PWMto turn left
          PWM_Left = 255 - (MAG_Error * P);
          if (PWM_Left > 255) PWM_Left = 255;                     //                   
          if (PWM_Left >= 0) {
            SetPins_ToGoForwards();                               // keep the mower moving forward
            
            #if defined(LCD_KEYPAD)
            lcd.setCursor(15, 0);
            lcd.print(" ");
            #endif
            
            }

       // Boost Turn LH
       if (Boost_Turn == 1) {
       if (PWM_Left < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnLeft();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              }
        }
         
          if (PWM_Left < 0) {
            PWM_Left = (-1 * PWM_Left) + 220;
            if (PWM_Left > 255) PWM_Left = 255;
  
            #if defined(LCD_KEYPAD)
            lcd.setCursor(15, 0);
            lcd.print("*");
            #endif
            
            SetPins_ToTurnLeft();
            delay(5);
            }
          
          Motor_Action_Dynamic_PWM_Steering();
          Serial.print(F(" Turn Left "));
          Tracking_Turn_Right = 0;
          Tracking_Turn_Left = Tracking_Turn_Left + 1;
          if (Tracking_Turn_Left > Max_Tracking_Turn_Left) {
            Motor_Action_Stop_Motors();
            Tracking_Restart_Blocked_Path();
          }
        }
        if (MAG_Error <= 0) {                                     // Turn the Mower to the right to get back on the wire.
          //Turn Right
          PWM_Left = 255;
          PWM_Right = 255 + (MAG_Error * P);          // + as mag_error is negative to adjust PWM
          if (PWM_Right > 255) PWM_Right = 255;
          if (PWM_Right >= 0) {
            SetPins_ToGoForwards();
            
            #if defined(LCD_KEYPAD)
            lcd.setCursor(15, 0);
            lcd.print(" ");
            #endif
            
            }

          
       // Boost Turn RH
       if (Boost_Turn == 1) {
          if (PWM_Right < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnRight();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              } 
       }
       
  
          if (PWM_Right < 0) {
            PWM_Right = (-1 * PWM_Right) + 220 ;
            if (PWM_Right > 255) PWM_Right = 255;
            if (PWM_Right >= 0) SetPins_ToTurnRight();
            
            #if defined(LCD_KEYPAD)
            lcd.setCursor(15, 0);
            lcd.print("*");
            #endif
            
          }
  
          Motor_Action_Dynamic_PWM_Steering();
          Serial.print(F(" Turn Right "));
          Tracking_Turn_Left = 0;
          Tracking_Turn_Right = Tracking_Turn_Right + 1;
          if (Tracking_Turn_Right > Max_Tracking_Turn_Right) {
            Motor_Action_Stop_Motors();
            Tracking_Restart_Blocked_Path();
            } 
      }
        Serial.print(F(" : MAG_Error="));
        Serial.println(MAG_Error);
        Dock_Cycles = Dock_Cycles + 1;
        Loop_Cycle_Mowing = I;
        if (Dock_Cycles > 10) {
          Tracking_Wire = Tracking_Wire + 1;                            // Makes the wire tracking LED in the app blink.
          if (Tracking_Wire > 1) Tracking_Wire = 0;
          Get_WIFI_Commands();
          Dock_Cycles = 0;
        }
    }
 
 }



  #if defined(LCD_KEYPAD)
  lcd.setCursor(10,1);
  lcd.print(I);
  #endif
//  Check_Bumper();
//  if ((Bump_Frnt_LH == true) || (Bump_Frnt_RH == true)) I = Track_Wire_Itterations; 
  }  

#if defined(LCD_KEYPAD)
lcd.clear();
#endif

Tracking_Wire = 0;
Loop_Cycle_Mowing = 0;

delay(5);

}

}


//  Track the Perimeter wire using a PID type method
//  This code tracks the boundary wire and by calculating the Perimeter Magnitude and calculating the distance to the center of the wire.
//  This error value of (position - center of wire) is mulitplied by the P value in the setup to send a PWM change to the left or right
//  wheel to bring the sensor back over the wire.*/


void Track_Perimeter_Wire_To_Dock()  {
  
  #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print(F("Tracking to.."));
      lcd.setCursor(0, 1);
      lcd.print(F("Charging Station"));                                            // Prints info to LCD display
      #endif
  
  Motor_Action_Stop_Spin_Blades();
  Docked_Hits = 0;
  Check_if_Charging();                                                        // Checks if an amperage is detected on the charge wire
  Check_if_Docked();
  

  Serial.println(F(" Tracking the wire to the Garage: "));                     // Prints the PID values used.
  Serial.print(F("P = "));
  Serial.print(P);
  Serial.print(F(" / D = "));
  Serial.print(D);
  Serial.println(F(""));

  Tracking_Wire = 1;
  Loop_Cycle_Mowing = 0;
  Mower_Running = 0;
  
  MAG_Now = 0;                                                              // Reset Values
  MAG_Start = 0;
  MAG_Error = 0;
  MAG_Goal = 0;
  int Dock_Cycles = 0;
  delay(5);
  if (CCW_Tracking_To_Charge == 1)  {                                                   // Mower tracks the wire in a Counter Clockwise Direction
    Serial.println(F("TRACKING COUNTER-CLOCKWISE"));
    while ((Mower_Docked == 0) && (Mower_Parked == 0)) {
      
      #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print(F("Tracking CCW to"));
      lcd.setCursor(0, 1);
      lcd.print(F("Charging Station"));                                            // Prints info to LCD display
      #endif
      
      delay(5);
      ADCMan.run();
      MAG_Start = perimeter.getMagnitude(0);                                    // Gets the signal strength of the sensor
      MAG_Now = MAG_Start;
      delay(5);
      MAG_Error = (MAG_Goal - MAG_Start);                                       // Calculates the Error to the center of the wire which is normally zero magnitude (remember - - is + )
      PrintWirePosition();                                                      // Prints the overview to the Serial Monitor.
      Loop_Cycle_Mowing = Loop_Cycle_Mowing + 1;

      if (MAG_Error > 0) {                                                      // Trun the mower to the right if MAG_Error > 0 with a CCW track direction.
        // RIGHT TURN
        PWM_Left = 255;                                                         // Set PWM_Left to maximum
        PWM_Right = 255 - (MAG_Error * P);                                      // Mag_Error * P is the value reduced from the max value set PWM and sent to the PWM                 
        if (PWM_Right > 255) PWM_Right = 255;                                   // Caps the PWM_Right to 255
        if (PWM_Right >= 0) {
          SetPins_ToGoForwards();
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print(" ");
          #endif
          
          }

       // Boost Turn RH
       if (Boost_Turn == 1) {
          if (PWM_Right < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnRight();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              } 
       }
       
        
        if (PWM_Right < 0) {                                                    // Set the wheels to rotate around the axis if a sharp turn is required.
          PWM_Right = (-1 * PWM_Right) + 220 ;                                  // change the negative value to a positive for the PWM input to the motor bridge.
          if (PWM_Right > 255) PWM_Right = 255;                                 // cap the maximum PWM to 255
          SetPins_ToTurnRight();                                                // set the motor bridge pins to turn left 
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print("*");
          #endif
          
          }
       
        Motor_Action_Dynamic_PWM_Steering();                                          // Carries out the wheel PWM changes for steering on the wire
        Serial.print(F(" Turn Right "));
        Tracking_Turn_Left = 0;                                                 // Failsafe if the mower looses the wire.  If the mower is commanded to turn left or right
        Tracking_Turn_Right = Tracking_Turn_Right + 1;                          // too many times it is assumed that the mower is spinning and cant get back on the wire.
        if (Tracking_Turn_Right > Max_Tracking_Turn_Right) {                    // if this is detected then a function is ran to find the wire again.
          Tracking_Restart_Blocked_Path();
        }

      }
      if (MAG_Error <= 0) {                                                     // Turn the mower to the left if MAG_Error < 0 with a CCW track direction
        //LEFT TURN
        PWM_Right = 255;                                                        // Set the PWM_Right to maximum
        PWM_Left = 255 + (MAG_Error * P);                                       // + as mag_error is negative to adjust PWM
        if (PWM_Left > 255) PWM_Left = 255;                                     // cap PWM_Left to the maximum 
        if (PWM_Left >= 0) {
          SetPins_ToGoForwards();

          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print(" ");
          #endif
          
          }
        
       // Boost Turn LH
       if (Boost_Turn == 1) {
       if (PWM_Left < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnLeft();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              }
        }

        if (PWM_Left < 0) {                                                     // if the PWM calulated is below zero set the mower for a sharp turn
          PWM_Left = (-1 * PWM_Left) +220 ;                                          // make the negative value a positive one to input to the motor bridge
          if (PWM_Left > 255) PWM_Left = 255;                                   // if again the PWM is above 255 then cap it to 255
          SetPins_ToTurnLeft();                                                 // set the pins to sharp turn left  
          delay(5);
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print("*");
          #endif
          
          }
          
        Motor_Action_Dynamic_PWM_Steering();
        Serial.print(F(" Turn Left "));
        Tracking_Turn_Right = 0;
        Tracking_Turn_Left = Tracking_Turn_Left + 1;
        if (Tracking_Turn_Left > Max_Tracking_Turn_Left) {
          Tracking_Restart_Blocked_Path();
          if (Mower_Parked == 1) Mower_Docked = 1;
        }
      }
      Serial.print(F(" : MAG_Error="));
      Serial.print(MAG_Error);
      Read_Serial1_Nano();
      Check_if_Charging();
      Check_if_Docked();
      Dock_Cycles = Dock_Cycles + 1;
      if (Dock_Cycles > 10) {
        Tracking_Wire = Tracking_Wire + 1;                            // Makes the wire tracking LED in the app blink.
        if (Tracking_Wire > 1) Tracking_Wire = 0;
        Mower_Running = 0;
        Get_WIFI_Commands();
        Dock_Cycles = 0;
        }
      }
  Loop_Cycle_Mowing = 0;  
  }

  if (CW_Tracking_To_Charge == 1)  {                             // Mower tracks the wire in a Counter Clockwise Direction
    Serial.println(F("TRACKING ---  CLOCKWISE"));               // With the same functions as above
    while ((Mower_Docked == 0) && (Mower_Parked == 0)) {
      delay(5);
      
      #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print(F("Tracking CW to"));
      lcd.setCursor(0, 1);
      lcd.print(F("Charging Station"));                            // Prints info to LCD display
      #endif
      
      ADCMan.run();
      MAG_Start = perimeter.getMagnitude(0);
      MAG_Now = MAG_Start;
      delay(5);
      MAG_Error = (MAG_Goal - MAG_Start);
      PrintWirePosition();
      Loop_Cycle_Mowing = Loop_Cycle_Mowing + 1;

      // Turn the Mower to the left to get back on the wire. Clock Wise Motion around the wire
      // Power down the left wheel and full power right wheel to turn left
      if (MAG_Error > 0) {                                      // if MAG_Error > 0 then Turn left in CW tracking                      
        // Turn LEFT
        PWM_Right = 255;                                        // Set the right wheel to max PWMto turn left
        PWM_Left = 255 - (MAG_Error * P);
        if (PWM_Left > 255) PWM_Left = 255;                     // 
        if (PWM_Left >= 0) {
          SetPins_ToGoForwards();                               // keep the mower moving forward
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print(" ");
          #endif
          
          }

       // Boost Turn LH
       if (Boost_Turn == 1) {
       if (PWM_Left < Min_Track_PWM) {
              Serial.print(F("XX"));
              SetPins_ToTurnLeft();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              }
        }
        
        if (PWM_Left < 0) {
          PWM_Left = (-1 * PWM_Left) + 220;
          if (PWM_Left > 255) PWM_Left = 255;
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print("*");
          #endif
          
          SetPins_ToTurnLeft();
          delay(5);
          }
        
        Motor_Action_Dynamic_PWM_Steering();                          // send the PWM values to the motor driver.
        Serial.print(F(" Turn Left "));
        Tracking_Turn_Right = 0;
        Tracking_Turn_Left = Tracking_Turn_Left + 1;
        if (Tracking_Turn_Left > Max_Tracking_Turn_Left) {
          Tracking_Restart_Blocked_Path();
          Tracking_Turn_Left = 0;
        }
      }
      if (MAG_Error <= 0) {                                     // Turn the Mower to the right to get back on the wire.
        //Turn Right
        PWM_Left = 255;
        PWM_Right = 255 + (MAG_Error * P);          // + as mag_error is negative to adjust PWM
        if (PWM_Right > 255) PWM_Right = 255;
        if (PWM_Right >= 0) {
          SetPins_ToGoForwards();
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print(" ");
          #endif
          
          }

       // Boost Turn RH
       if (Boost_Turn == 1) {
          if (PWM_Right < Min_Track_PWM) {
              Serial.print(F(""));
              SetPins_ToTurnRight();
              Ramp_Motor_ON = 0;
              Motor_Action_Go_Full_Speed();
              delay(Hard_Track_Turn_Delay);
              Ramp_Motor_ON = 1;
              SetPins_ToGoForwards();
              } 
       }

        if (PWM_Right < 0) {
          PWM_Right = (-1 * PWM_Right) + 220 ;
          if (PWM_Right > 255) PWM_Right = 255;
          if (PWM_Right >= 0) SetPins_ToTurnRight();
          
          #if defined(LCD_KEYPAD)
          lcd.setCursor(15, 0);
          lcd.print("*");
          #endif
        }

        Motor_Action_Dynamic_PWM_Steering();
        Serial.print(F(" Turn Right "));
        Tracking_Turn_Left = 0;
        Tracking_Turn_Right = Tracking_Turn_Right + 1;
        if (Tracking_Turn_Right > Max_Tracking_Turn_Right) {
          Tracking_Restart_Blocked_Path();
          Tracking_Turn_Right = 0;
          }

      }
      Serial.print(F(" : MAG_Error="));
      Serial.print(MAG_Error);
      Read_Serial1_Nano();
      Check_if_Charging();
      Check_if_Docked();
      Dock_Cycles = Dock_Cycles + 1;
      if (Dock_Cycles > 10) {
        Tracking_Wire = Tracking_Wire + 1;                            // Makes the wire tracking LED in the app blink.
        if (Tracking_Wire > 1) Tracking_Wire = 0;
        Mower_Running = 0;
        Get_WIFI_Commands();
        Dock_Cycles = 0;
        }
    }
  Loop_Cycle_Mowing = 0;  
  }
 
 if (Mower_Docked == 1) Tracking_Wire = 0;
 Loop_Cycle_Mowing = 0;

//Check_Bumper(); 
//if (Bumper_Activate_Frnt == true) {
//    if ((Bump_Frnt_LH == true) || (Bump_Frnt_RH == true)) Tracking_Restart_Blocked_Path(); 
//    }
}


//Starts an algorithym to find the wire again after it is lost in tracking
void Tracking_Restart_Blocked_Path() {
  if (Mower_Docked != 1) {  
      Tracking_Turn_Left = 0;                                       // Resets the tracking error counters
      Tracking_Turn_Right = 0;                                      // Resets the tracking error counters
      Motor_Action_Stop_Motors();
      Serial.println(F(""));
      Serial.println(F("Possible Blocked Path - Trying to Avoid"));
      Serial.println(F(""));
      Mower_Running = 1;
      Tracking_Wire = 1;
      Get_WIFI_Commands();                                          // TX and RX data from NodeMCU
      delay(1000);
      Mower_Running = 0;
      Tracking_Wire = 0;
      Get_WIFI_Commands();                                          // TX and RX data from NodeMCU
      delay(1000);
      Mower_Running = 1;
      Tracking_Wire = 1;
      Get_WIFI_Commands();                                          // TX and RX data from NodeMCU
      delay(1000);
      Mower_Running = 0;
      Tracking_Wire = 0;
      Get_WIFI_Commands();                                          // TX and RX data from NodeMCU
      delay(1000);
      
      #if defined(LCD_KEYPAD)
      lcd.clear();
      lcd.print(F("Wire Lost."));
      lcd.setCursor(0, 1);
      lcd.print(F("Recovering....."));       
      #endif
      
      // Prints info to TFT display
      Get_WIFI_Commands();                                   // TX and RX data from NodeMCU
      if (Mower_Parked != 1) {                                                      // If Pause has been pressed dont carry on.
        SetPins_ToGoBackwards();
        delay(500);
        Motor_Action_Go_Full_Speed();
        delay (5000);                                                               //Reversing Time in seconds
        Motor_Action_Stop_Motors();
        delay(2000);
        Mower_Running = 0;
        Tracking_Wire = 0;
        Get_WIFI_Commands();                                                        // TX and RX data from NodeMCU
        if (Compass_Activate == 1) Compass_Turn_Mower_To_Home_Direction();
        Manouver_Find_Wire_Track();
        //Track_Perimeter_Wire_To_Dock();
        }
  }
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// General
/*
 * General function for control of cycle or measuring of loop cycle time
 *
 */

signed long resultCTime;  // result of loop cycle time
signed long startCTime;   // variable where time of measuring start is stored


// function for measuring of loop cycle time, for usage just call it
void RuntimeMeasuring() {
  resultCTime = (signed long)millis() - startCTime;
  Serial.print(F("|CT:")); Serial.print(resultCTime); Serial.print(F("|"));
  startCTime = (signed long)millis();
}


// Function Timer with delay on, auto reset when condition are met
// usage: if (TimerDelayOn(T10, 10000)) Print_Time_TFT(); -> this will call function Print_Time_TFT() each 10s
bool TimerDelayOn(struct timerVar_t &T, int T_PRE)//, int T_PRE)
{
  bool ret = false;

  if (!T.flagRun) {
    T.PRE = T_PRE;
    T.REF = (signed long)millis();
    T.flagRun = true;
  }

  T.ACC =(signed long)millis() - T.REF;
  if (T.ACC > T.PRE)
  {
    T.flagRun = false;
    ret = true;
  }

  return ret;
}


// Function Retentive timer with delay on, reset should be done by function TimerReset()
bool TimerDelayOnRet(struct timerVar_t &T)//, int T_PRE)
{
  bool ret = false;

  if (!T.flagRun) {
    //T.PRE = T_PRE;
    T.REF = (signed long)millis();
    T.flagRun = true;
  }

  T.ACC =(signed long)millis() - T.REF;
  if (T.ACC > T.PRE)
  {
    ret = true;
  }

  return ret;
}



void TimerReset(struct timerVar_t &T)
{
  T.REF = (signed long)millis();
  T.flagRun = false;
}


void MEGA_WDT() {
#if defined(WDT) and defined(BOARD_MEGA)
  if (Mower_Running == 1 && !wdt_enable_flagRun) {
    //wdt_enable(WDTO_2S); // reset Mega when freeze longer then 2 seconds
    //wdt_enable(WDTO_4S); // reset Mega when freeze longer then 4 seconds
    wdt_enable(WDTO_8S); // reset Mega when freeze longer then 8 seconds
    wdt_enable_flagRun = true;
  }
  else if (Mower_Running == 0) {
    wdt_disable();
    if (wdt_enable_flagRun) Serial.println(F("\n !!! WDT disabled !!!"));
    wdt_enable_flagRun = false;
  }
  else {
    wdt_reset(); // reset HW WatchDog timer
  }


  // remote software restart of Arduino Mega
  if (Mega_SW_restart_En) {
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while (1) {}
  }
#endif // -(WDT)-
}
