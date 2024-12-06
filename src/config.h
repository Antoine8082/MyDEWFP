//-----------------------------------------------------------------------
// myFocuserPro2 Focuser Config File TMC2209
// (c) R Brown, 2014-2024, All rights reserved.
//-----------------------------------------------------------------------
#if !defined(config_h)
#define config_h

#include <Arduino.h>
#include "myBoardDefs.h"
#include "defines.h"


//-----------------------------------------------------------------------
// WARNING: SKETCH AND GLOBAL VARIABLE SIZE LIMITS
//-----------------------------------------------------------------------
// When enabling options in this file, you will need to try and keep the 
// Arduino sketch and Global variables to allow the Arduino chip to 
// work without running out of space, which results in random reboots and 
// the Serial interface not working.
// 
// When the sketch is compiled, these messages are found in the Output 
// Window and looks like
//      Sketch uses 26548 bytes (86%) of program storage space. Maximum is 30720 bytes.
//      Global variables use 973 bytes (47%) of dynamic memory
//
// The sketch usage should be less that 83% most of the time (varies on 
// what is enabled.) Higher sketch usage will result in errors and 
// unexpected values being returned.
//
// The Global variables should be less than 60-70% (if debugging is enabled
// then the Global variable usage will exceed the safety limit and make 
// the sketch unusable). 
// 
// If either these are too high, you will need to disable an option to 
// decrease the Sketch size and Global var size to recommended levels
// 
// Refer to the spreadsheet Options Program Size for more information


//-----------------------------------------------------------------------
// BOARDS SUPPORTED
//-----------------------------------------------------------------------
// PCB TMC2225_TMC2209
// BUZZ.LEDs.LCD.OLED.PB.HPSW.SPWR.SG


//-----------------------------------------------------------------------
// SPECIFY DRIVER BOARD HERE
//-----------------------------------------------------------------------
#define DRVBRD TMC2209


//-----------------------------------------------------------------------
// SPECIFY HOME POSITION SWITCH OPTIONS HERE
//-----------------------------------------------------------------------
// do NOT uncomment HOMEPOSITIONSWITCH if you do not have the switch fitted
// To enable the HOMEPOSITION SWITCH, uncomment the nextline
//#define HOMEPOSITIONSWITCH 1

// If using HOMEPOSITIONSWITCH, use one of the following (not both)
//#define USE_STALL_GUARD 1
// or
// use a physical switch to detect Home Position
//#define USE_PHYSICAL_SWITCH 2


//-----------------------------------------------------------------------
// INPUT DEVICES
// PUSH BUTTONS, STEPPERPWRDETECT, TEMPERATURE PROBE 
//-----------------------------------------------------------------------
// To enable the Push Buttons for manual focusing, uncomment the next line
//#define PUSHBUTTONS 1

// To specify the number of motor steps to move for 1 push button press
// change the 1 value for PB_STEPS below. The max value should be 1/2 the
// focuser step size, [1-255 is the range]. If you set PB_STEPS to 0
// then the push buttons will not move the focuser.
//#define PB_STEPS 1

// This prevents the stepper motor moving when 12V to the stepper is OFF
// and needs special circuitry or has no effect. To enable the 12V power
// detect to the stepper motor, uncomment the next line (only available on some boards)
//#define STEPPERPWRDETECT 1

// To enable the temperature probe, uncomment next line
//#define TEMPERATURE_PROBE 1


//-----------------------------------------------------------------------
// OUTPUT DEVICES
// BUZZER AND LEDS
//-----------------------------------------------------------------------
// Buzzer is used as a power on boot test, and with push-buttons as a
// feedback for push button operation
// To enable the buzzer, uncomment the next line
//#define BUZZER 1

// To enable the IN-OUT LEDS, uncomment the next line
//#define INOUTLEDS 1


//-----------------------------------------------------------------------
// SOFTWARE OPTIONS
//-----------------------------------------------------------------------
// To enable the super slow jogging, uncomment the next line
//#define SUPERSLOWJOG 1


//-----------------------------------------------------------------------
// LCD LIQUID CRYSTAL DISPLAY, I2C
//-----------------------------------------------------------------------
// Uncomment one of the following LCDxxxx lines depending upon your lcd type
// #define DISPLAYTYPE  DISPLAY_LCD1602 // 16 character, 2 lines
//#define DISPLAYTYPE  DISPLAY_LCD1604 // 16 character, 4 lines
//#define DISPLAYTYPE  DISPLAY_LCD2004 // 20 character, 4 lines


//-----------------------------------------------------------------------
// OLED DISPLAY I2C
//-----------------------------------------------------------------------
//#define DISPLAYTYPE DISPLAY_OLED12864 // 128x64, 0.96" 16 character, 8 lines

// Select the correct driver chip for the OLED
//#define USE_SSD1306 1  // For the OLED 128x64 0.96" display using the SSD1306 driver, uncomment  this line
//#define USE_SSH1106 2 // For the OLED 128x64 1.3" display using the SSH1106 driver, uncomment this line


//-----------------------------------------------------------------------
// USER CONFIGURATION END: DO NOT EDIT BELOW THIS LINE
//-----------------------------------------------------------------------
// check DRVBRD
#if !defined(DRVBRD)
#error "DRVBRD is not defined in config.h"
#endif

#if !defined(PB_STEPS)
#define PB_STEPS 1
#endif

#if (DRVBRD == TMC2225_TMC2209)
#if defined(HOMEPOSITIONSWITCH)
#if !defined(USE_STALL_GUARD) && !defined(USE_PHYSICAL_SWITCH)
#error "HPSW requires either USE_STALL_GUARD or USE_PHYSICAL_SWITCH"
#endif
#endif

#if defined(USE_STALL_GUARD) && defined(USE_PHYSICAL_SWITCH)
#error "You cannot have both USE_STALL_GUARD and USE_PHYSICAL SWITCH defined - must be one or the other"
#endif 
#endif // DRVBRD ==

// check OLED display
#if defined(DISPLAYTYPE)
#if (DISPLAYTYPE == OLED12864)
#if defined(USE_SSD1306) && defined(USE_SSH1106)
#error "Both USE_SSD1306 and USE_SSH1106 is defined, only enable one if using DISPLAY_OLED12864"
#endif
#if !defined(USE_SSD1306) && !defined(USE_SSH1106)
#error "You must enable either USE_SSD1306 and USE_SSH1106 for DISPLAY_OLED12864"
#endif
#endif
#endif

#endif
