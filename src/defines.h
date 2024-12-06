//-----------------------------------------------------------------------
// myFocuserPro2 Definitions
// (c) R Brown, 2014-2024, All rights reserved.
//-----------------------------------------------------------------------
#ifndef myDefines_h
#define myDefines_h

#include <Arduino.h>


//-----------------------------------------------------------------------
// GLOBAL DEFINES - DO NOT CHANGE
//-----------------------------------------------------------------------

// Machine states
enum StateMachineStates { State_Idle, 
                          State_InitMove, 
                          State_ApplyBacklash, 
                          State_Moving, 
                          State_SetHomePosition, 
                          State_FindHomePosition, 
                          State_MoveEnded, 
                          State_DelayAfterMove, 
                          State_FinishedMove };

enum tempprobes { DS18B20, DHT11, DHT21, DHT22, DHT33 };

enum MotorType { MOTOR_28BYJ, MOTOR_28BYJ_S8, MOTOR_35BYJ_412B};

// myFocuserPro2 struct for holding all controller values
struct config_t {
  int validdata;                    // if this is 99 then data is valid
  long fposition;                   // last focuser position
  long maxstep;                     // max steps
  double stepsize;                  // the step size in microns, ie 7.2, minimum value is 0.001 microns
  int  tempcoefficient;             // steps per degree temperature coefficient value [0-255]
  int  stepmode;                    // stepping mode, full, half, 1/4, 1/8. 1/16. 1/32 [1.2.4.8.16.32.64.128.256]
  byte reversedirection;            // reverse direction
  byte coilpower;                   // coil pwr
  byte tempmode;                    // temperature display mode, Celcius=1, Fahrenheit=0
  byte display_pagetime;                 // refresh rate of display - time each page is displayed for 2-7 seconds
  byte stepsizeenabled;             // if 1, controller returns step size
  byte display_updateonmove;             // update position on lcd when moving
  byte tempresolution;              // resolution of DS18B20 temperature probe
  byte delayaftermove;              // delay after movement is finished
  byte backlashsteps_in;            // number of backlash steps to apply for IN moves
  byte backlashsteps_out;           // number of backlash steps to apply for OUT moves
  byte focuserdirection;            // keeps track of last focuser move direction
  byte backlash_in_enabled;         // enable or disable backlash compensation for IN
  byte backlash_out_enabled;        // enable or disable backlash compensation for OUT
  byte tcdirection;                 // temperature compensation in or out
  byte motorspeed;                  // speed of stepper motor
  byte display_enabled;              // status of display, enabled or disabled
  byte pbsteps;                     // for 1 push button press, move this amount of motor steps
  byte display_option;               // which display pages to show. pg0-pg6 [7 pages]
  int stepdelay;
};

extern struct config_t myfocuser;

#define BTRX 11                     // bluetooth adapter HC05 RX
#define BTTX 10                     // bluetooth adapter HC05 RX
#define BUZZERPIN A3                // buzzer
#define ENCODERSWPIN 11             // rotary encoder push sw on pin 11
#define HPSWPIN 12                  // home position switch is on D12
#define INLED A1                    // in led
#define OUTLED A2                   // out led
#define PBSWITCHESPIN A0            // push button switches
#define RECV_PIN 9                  // infrared controller signal must be connected to pin 9
#define ROTENCPIN9 9                // rotary encoder is connected to pins 9 and 10
#define ROTENCPIN10 10              // rotary encoder is connected to pins 9 and 10
#define STEPPERDETECTPIN A7         // determine if 12V stepper power in ON or
#define TEMPPIN 2                   // temperature probe on pin 2, use 4.7k pullup

#define DS18B20   1                 // DO NOT CHANGE: For ST6128 board
#define DHTxx     2                 // DO NOT CHANGE: For ST6128 board

#define FOCUSERUPPERLIMIT 400000L   // arbitary focuser limit up to 400,000
#define FOCUSERLOWERLIMIT 1024L     // lowest value that maxStep can be
#define DEFAULTMAXSTEP 10000L       // default value for maxStep
#define DEFAULTPOSITION 5000L       // default value for Position
#define ENDMOVEDELAY 5000L          // Time to wait after a move before disabling coil power

#define DEFAULTSTEPMODE 1           // Default step mode
#define DEFAULTSTEPSIZE 50.0        // This is the default setting for the step size in microns
#define MAXIMUMSTEPSIZE 100.0       // This is the maximum for stepsize, 100 microns
#define HOMESTEPS 200               // Prevent searching for home position switch never returning, this should be > than # of steps between closed and open

#define MOVINGIN 0                  // direction of focuser moving inwards towards 0
#define MOVINGOUT 1                 // direction of focuser moving outwards towards maxstep
#define MOTORSPEEDDELAY 5000L       // delay between motor pulses
#define SLOWSPEEDJOGDELAY 250       // milliseconds delay for slow speed jogging

#define EEPROMSIZE 1024             // ATMEGA328P 1024 EEPROM
#define EEPROMWRITEINTERVAL 10000L  // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
#define VALIDDATAFLAG 99            // valid eeprom data flag

// DISPLAY
#define DISPLAY_LCD1602 1           // 16 character, 2 lines
#define DISPLAY_LCD1604 2           // 16 character, 4 lines
#define DISPLAY_LCD2004 3           // 20 character, 4 lines
#define DISPLAY_OLED12864 4
#define DISPLAY_OLED6432 5
#define DISPLAY_NOKIA 6
#define DEFAULT_PAGETIME 5          // in seconds, the time that a page is displayed before a new page is shown
#define DISPLAY_PAGETIME_MAX 7      // 7s
#define DISPLAY_PAGETIME_MIN 2      // 2s
#define DISPLAY_STEPCOUNT 15        // the number of steps moved which triggers an lcd update when moving, do not make too small
#define DISPLAY_LINEBUFFER 16       // for scroll up of LCD1602
#define LCDADDRESS 0x27             // some LCD displays maybe at 0x3F, use I2Cscanner to find the correct address
#define OLED_ADDR 0x3C              // some OLED displays maybe at 0x3F, use I2Cscanner to find the correct address


#define TEMP_REFRESHRATE 1000L       // refresh rate between temperature conversions unless an update is requested via serial command
#define TEMPRESOLUTION 10           // Set the default DS18B20 resolution to 0.25 of a degree 9=0.5, 10=0.25, 11=0.125, 12=0.0625
#define CELCIUS 1
#define FAHRENHEIT 0

#define EOC '#'                     // end of command
#define QUEUELENGTH 4               // number of commands that can be saved in the serial queue
#define BTPORTSPEED 9600            // define bluetooth speed, DO NOT CHANGE
#define SERIALPORTSPEED 9600        // 9600, 14400, 19200, 28800, 38400, 57600


//-----------------------------------------------------------------------
// DISPLAY INFO: DO NOT CHANGE
//-----------------------------------------------------------------------

/*
//-----------------------------------------------------------------------
// NOKIA 5110
//-----------------------------------------------------------------------
84×48-pixel monochrome 
Philips PCD8544 display controller
5 lines 

#include <LCD5110_Basic.h>
LCD5110* _Display;
extern uint8_t SmallFont[];
#endif  // #if defined(DISPLAY_NOKIA)

_Display = new LCD5110(8, 9, 10, 11, 12);
// The NOKIA PCB requires a Nokia 5110 LCD module.
// It is assumed that the LCD module is connected to
// the following pins using a levelshifter to get the
// correct voltage to the module.
//      SCK  - Pin 8
//      MOSI - Pin 9
//      DC   - Pin 10
//      RST  - Pin 11
//      CS   - Pin 12

Graphics
#include <LCD5110_Graph.h>

LCD5110 myGLCD(8, 9, 10, 11, 12);

*/

//-----------------------------------------------------------------------
// DEBUGGING
//-----------------------------------------------------------------------
// do not change - leave this commented out
//#define DEBUG 1

#if defined(DEBUG)                                     //Macros are usually in all capital letters.
#define DebugPrint(...) Serial.print(__VA_ARGS__)      //DPRINT is a macro, debug print
#define DebugPrintln(...) Serial.println(__VA_ARGS__)  //DPRINTLN is a macro, debug print with new line
#else
#define DebugPrint(...)  //now defines a blank line
#define DebugPrintln(...)
#endif

// do not change - leave this commented out
//#define HPSWDEBUG 1

#if defined(HPSWDEBUG)
#define HPSWDebugPrint(...) Serial.print(__VA_ARGS__)
#define HPSWDebugPrintln(...) Serial.println(__VA_ARGS__)
#else
#define HPSWDebugPrint(...)
#define HPSWDebugPrintln(...)
#endif

#endif