//-----------------------------------------------------------------------
// myFocuserPro2 Driver Board Definitions
// DEFINES FOR BOARD TYPES
//-----------------------------------------------------------------------

#ifndef myBoardDefs_h
#define myBoardDefs_h

//-----------------------------------------------------------------------
// myFP2 Boards
// https://sourceforge.net/projects/arduinofocuscontrollerpro/boards
// Videos (31) https://www.youtube.com/playlist?list=PL2iOz0v_d52gYrfMuXr2Mg1OJa5XtqHXg
//-----------------------------------------------------------------------

// General Boards (users choose the options they want or install all components for all features)
// For a minimal board user would fit components for the Buzzer and Speaker
// M 	 Buzzer, LED's
// MT  Buzzer, LED's, Temperature Probe
// F 	 Buzzer, LED's, Temperature Probe, Pushbuttons,

// Options supported by the PCB and Stripboards
// for A4998_M_MT_F_BT example (BUZ.LEDs.LCD.OLED.PB.TMP)

// BT 	  Bluetooth
// BUZZ   Buzzer
// HPSW   Home Position Switch
// IR 	  InfraRed Remote
// LEDs   In=Out move status LED's
// LCD	  Liquid Crystal Display
// NOK 	  Nokia Display
// OLED   OLED display
// PB 	  Pushbuttons
// RE 	  Rotary Encoder
// SG     Stall Guard TMC2209
// SPWR   Stepper Power
// TMP    Temperature Probe

// NOTE: Motor Shields
// Motor Shields requires the following
// 1: Arduino UNO
// 2: Motor Shield
// 3: Stripboard that connects Motor Sheild pins to the Stripboard and then to devices
//    Temperature Probe, Push Buttons, LEDs, Displays etc

// myFP2_A4998 supports
#define A4998_M_MT_F_BT 100 // A4998_M_MT_F_BT PCB. BT.BUZ.LEDs.LCD.OLED.HPSW.SPWR.TMP
#define A4988_MT 101        // A4988_MT Stripboard, BUZ.LEDs.TMP

// myFP2_AdafruitMotorShieldV2_334 supports
// Arduino UNO + ADAfruit motor shield v2 + Stripboard
#define Adafruit_MotorShield_V2 102 // Stripboard: BUZ.LEDs.LCD.OLED.PB.TMP

// myFP2_CLOSEDLOOP supports
#define CLOSEDLOOP 103 // Closed Loop PCB CT42T: unit for zero lost steps

// myFP2_DRV8825 supports
#define DRV8825_M_MT_F_BT 104 // DRV8825_M_MT_F_BT PCB: BT.BUZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP
#define DRV8825_RE 105        // DRV8825_RE Stripboard: BUZ.LEDs.LCD.OLED.TMP.RE

// myFP2_DRV8825_SOLDERLESS supports
#define DRV8825_SOLDERLESS 107 // DRV8825SOLDERLESS Stripboard: OLED.TMP.IR (StepMode set by jumpers)

// myFP2_DRV8825HW203 supports
#define DRV8825HW203_M_MT_F_BT 108 // DRV8825HW203_M_MT_F_BT PCB: BT.BUZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP
#define DRV8825HW203_FRE 109       // DRV8825HW203_FRE PCB: BUZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP.RE
#define DRV8825HW203_FIRBT 110     // DRV8825HW203_FIRBT PCB: BT.BUZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP.IR
#define DRV8825_HW203_FIR 112      // DRV8825_HW203_FIR Stripboard: BUZ.LEDs.LCD.OLED.TMP.IR
#define DRV8825_HW203_FRE 113      // DRV8825_HW203_FRE Stripboard: BUZ.LEDs.TMP.RE
#define DRV8825_HW203_MT 114       // DRV8825_HW203_M_MT Stripboard: BUZ.LEDs.TMP

// myFP2_DRV8825HW203_KEYPAD supports
#define DRV8825HW203_KEYPAD4x4 116 // DRV8825HW203_M_MT_F_BT PCB + DRV8825HW203_KEYPAD4x4 sub-board
                                   // LCD1602.HPSW.TMP.4x4Keys

// myFP2_EASYDRIVER_M_MT_F_BT supports
#define EASYDRIVER_M_MT_F_BT 117 // EasyDriver PCB M_MT_F_BT: BT.BUZZ.LEDs.LCD.OLED.PB.HPSW.TMP.SPWR
#define EASYDRIVER_M_MT_F 118    // EasyDriver Stripboard: BUZ.LEDs.LCD.OLED.PB.TMP

// myFP2_EASYDRIVER_RE supports
#define EASYDRIVER_FRE 119 // EasyDriver PCB + Rotary Encoder: BUZZ.LEDs.LCD.OLED.HPSW.SPWR.TMP.RE
#define EASYDRIVER_RE 120  // EasyDriver Stripboard: BUZ.LEDs.LCD.OLED.TMP.RE

// myFP2_EASYDRIVER_PK supports
#define EASYDRIVER_PK 121 // EASYDRIVER_PK Stripboard for user PK: Nothing

// myFP2_L293D_SHIELD_UNO supports
#define L293D_SHIELD_UNO_M_MT_F 122 // L293D_SHIELD_UNO + Stripboard: BUZZ.LCD.OLED.PB.TMP

// myFP2_L293DMINI supports
#define L293DMINI_M_MT_F_BT 123 // L293DMINI_M_MT_F_BT PCB: BT.BUZZ.LEDs.LCD.OLED.HPSW.PB.TMP.SPWR
#define L293DMINI_NOKIA 124     // L293DMINI_NOKIA PCB: BUZZ.LEDs.NOKIA.PB.SPWR.TMP
#define L293DMINI_M_MT_F 125    // L293DMINI_M_MT_F Stripboard: BUZZ.LEDs.LCD.OLED.PB.TMP

// myFP2_L298N supports
#define L298N_M_MT_F_BT 126 // L298N_M_MT_F_BT PCB: BT.BUZZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP
#define L298N_M_MT_F 127    // L298N_M_MT_F Stripboard: BUZZ.LEDs.LCD.OLED.PB.TMP

// myFP2_L298P_SHIELD_UNO supports
#define L298P_SHIELD_UNO_M_MT_F 128 // L298P_SHIELD_UNO Stripboard: BUZ.LEDs.LCD.OLED.PB.TMP

// myFP2_L9110S supports
#define L9110S_M_MT_F_BT 129  // L9110S_M_MT_F_BT PCB: BT.BUZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP
#define L9110S_M_MT_NOKIA 130 // L9110S_M_MT_NOKIA PCB: BUZ.LEDs.NOKIA.PB.SPWR.TMP
#define L9110S_M_MT_F 131     // L9110S_M_MT_F Stripboard: BUZ.LEDs.LCD.OLED.PB.TMP

// myFP2_ST6128 supports
#define ST6128_M_MT_F 132 // ST6128_M_MT_F Stripboard: BUZZ.LEDs.LCD.OLED.PB.TMP

// myFP2_TB6612FNG supports
#define TB6612FNG_M_MT_F_BT 133 // TB6612FNG_M_MT_F_BT PCB: BT.BUZZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP
#define TB6612FNG 134           // TB6612FNG Stripboard: BUZZ.LEDs.LCD.OLED.PB.TMP

// myFP2_TMC2209SG supports (One PCB supports both TMC2209 and TMC2225 versions)
#define TMC2209 135 // TMC2225-TMC2209 PCB: BUZZ.LEDs.LCD.OLED.PB.HPSW.SPWR.TMP.SG
// myFP2_TMC2225 supports
#define TMC2225 136 // TMC2225-TMC2209 PCB: BUZZ.LEDs.LCD.OLED.PB.HPSW.SPWR.TMP

// myFP2_ULN2003 supports
#define ULN2003_M_MT_F_BT 137          // ULN2003_M_MT_F_BT PCB: BT.BUZZ.LEDs.LCD.OLED.HPSW.PB.TMP.SPWR
#define ULN2003_M_MT_F_BT_IR_RE_BT 138 // ULN2003_M_MT_F_BT_IR_RE_BT PCB: BT.BUZZ.LEDs.LCD.OLED.HPSW.IR.PB.RE.TMP.SPWR
#define ULN2003_NOKIA 139              // ULN2003_NOKIA PCB: BUZZ.LEDs.NOKIA.PB.TMP.SPWR
#define ULN2003_M_MT_F 140             // ULN2003_M_MT_F Stripboard: BUZZ.LEDs.LCD.OLED.PB.TMP

//-----------------------------------------------------------------------
// myFP2M boards MUST be programmed using myFP2M firmware
// https://sourceforge.net/projects/arduinoascomfocuserpro2diy/files/myFocuserPro2M/
// Videos
// https://www.youtube.com/watch?v=SYFPOzRNO2I&list=PL2iOz0v_d52iF6KmGnQuLn_-YU8TCTtQZ
// DRV8825 https://www.youtube.com/watch?v=Pgm9UNhY1oo
// L298N   https://www.youtube.com/watch?v=NYa4qjsf1uc
// ULN2003 https://www.youtube.com/watch?v=5dNatyl2yk0
//-----------------------------------------------------------------------
#define MYFP2M_DRV8825 150   // myFP2M DRV8825 PCB: BT.BUZZ.LEDs.OLED.TMP
#define MYFP2M_L293DMINI 151 // myFP2M L283DMini PCB: BT.BUZZ.LEDs.OLED.TMP
#define MYFP2M_L298N 152     // myFP2M L298N PCB: BT.BUZZ.LEDs.OLED.TMP
#define MYFP2M_L9110S 153    // myFP2M L9110S PCB: BT.BUZZ.LEDs.OLED.TMP
#define MYFP2M_ULN2003 154   // myFP2M ULN2003 PCB: BT.BUZZ.LEDs.OLED.TMP

//-----------------------------------------------------------------------
// myFP2N Nextion boards (requires the correct Board below - not one above)
// and MUST be programmed using myFP2N firmware
// https://sourceforge.net/projects/arduinoascomfocuserpro2diy/files/myFocuserPro2N%20NEXTION%20TFT%20DISPLAY/
// https://www.youtube.com/watch?v=NYa4qjsf1uc
//-----------------------------------------------------------------------
#define MYFP2N_DRV8825 160 // Nextion + DRV8825, PCB: BUZZ.LEDs.NEXTION.PB.TMP.SPWR, myFP2N_NX4832K035_3xx
#define MYFP2N_TMC2209 161 // Nextion + TMC2209, PCB: BUZZ.LEDs.NEXTION.PB.TMP.SPWR, myFP2N_NX4832K035_TMC2225_xxx
#define MYFP2N_TMC2225 162 // Nextion + TMC2225, PCB: BUZZ.LEDs.NEXTION.PB.TMP.SPWR, myFP2N_NX4832K035_TMC2209_xxx
#define MYFP2N_ULN2003 163 // Nextion + ULN2003, PCB: BUZZ.LEDs.NEXTION.PB.TMP.SPWR, myFP2N_NX4832K035_3xx

//-----------------------------------------------------------------------
// myFP2ESP8266 boards
// https://sourceforge.net/projects/myfp2esp8266-focus-controller/
// Videos https://www.youtube.com/watch?v=o1UVqP6QM2Q&list=PL2iOz0v_d52in30kmGdzZmvqTRZZBYcfd&pp=iAQB
//-----------------------------------------------------------------------
#define WEMOSDRV8825 35      // WEMOS + DRV8825 you also need to set DRV8825STEPMODE in myBoards.h
#define PRO2EDRV8825 36      // DRV8825 you also need to set DRV8825STEPMODE in myBoards.h
#define PRO2EULN2003 38      // ULN2003
#define PRO2EL293DNEMA 39    // L293D Motor shield ESP8266 with NEMA motor
#define PRO2EL293D28BYJ48 40 // L293D Motor shield ESP8266 with 28BYJ48 motor
#define PRO2EL298N 41        // L298N uses PCB layout for ULN2003
#define PRO2EL293DMINI 42    // L293DMini uses PCB layout for ULN2003
#define PRO2EL9110S 43       // L9110S uses PCB layout for ULN2003

//-----------------------------------------------------------------------
// myFP2ESP32 boards
// https://sourceforge.net/projects/myfocuserpro2-esp32/
// Videos https://www.youtube.com/playlist?list=PL2iOz0v_d52gQ6OrqP5WFlZofQt_yGAW7
// Old Videos https://www.youtube.com/watch?v=o1UVqP6QM2Q&list=PL2iOz0v_d52in30kmGdzZmvqTRZZBYcfd&pp=iAQB
//-----------------------------------------------------------------------
#define PRO2ESP32DRV8825 44   // DRV8825
#define PRO2ESP32ULN2003 45   // ULN2003
#define PRO2ESP32L298N 46     // L298N
#define PRO2ESP32L293DMINI 47 // L293DMini uses PCB layout for ULN2003
#define PRO2ESP32L9110S 48    // L9110S uses PCB layout for ULN2003
#define PRO2ESP32R3WEMOS 49   // https://www.ebay.com/itm/R3-Wemos-UNO-D1-R32-ESP32-
#define WEMOSDRV8825H 50      // Wemos + DRV8825 this is for Holger
#define PRO2ESP32TMC2225 56   // TMC2225 Board for BigTree TMC2225
#define PRO2ESP32TMC2209 57   // TMV2209 Board for BigTree TMC2209
#define PRO2ESP32TMC2209P 58  // This is board for Paul using TMC2209 for testing
#define PRO2ESP32ST6128 59    // ST6128 board
// Set target as LOLIN S2 Mini
#define PRO2ESP32LOLINS2MINI 60 // This is board for LOLIN ESP32S2 MINI
#define PRO2ESP32TMC2209LGTM 61 // This is a board created for testing LilyGo
// Set target to ESP32 PICO-D4
#define PRO2ESP32TMC2209XIAOC3 62 // This is board for LiliGo-T-Motor

//-----------------------------------------------------------------------
// myROTATOR (Arduino Nano) boards
// https://sourceforge.net/projects/arduino-myrotator/
//-----------------------------------------------------------------------
#define ROTATOR_DRV8825 70      // DRVBRD_DRV8825
#define ROTATOR_DRV8825_SBRD 71 // DRVBRD_DRV8825_STRIPBRD
#define ROTATOR_L298N 72        // DRVBRD_L298N
#define ROTATOR_ULN2003 73      // DRVBRD_ULN2003
// myROTATOR (ESP32) boards
#define ROTATOR_ESP32_DRV8825 74 // DRVBRD_ESP32_DRV8825, NODE MCU LUA, ESp32 DEV
// Set target to ESP32 PICO-D4
#define ROTATOR_ESP32_LILYGO 75       // DRVBRD_ESP32_LILYGO
#define ROTATOR_ESP32_DRV8825_SBRD 76 // DRVBRD_ESP32_DRV8825_STRIPBRD, NODE MCU LUA, ESp32 DEV
#define ROTATOR_ESP32_ULN2003 77      // DRVBRD_ESP32_ULN2003, NODE MCU LUA, ESp32 DEV
#define ROTATOR_ESP32_L298N 78        // DRVBRD_ESP32_L298N, NODE MCU LUA, ESp32 DEV

//-----------------------------------------------------------------------
// mySQM+ (ESP32 Dev) boards
// https://sourceforge.net/projects/mysqmproesp32/
//-----------------------------------------------------------------------
#define mySQMPLUS_ESP32 80      // https://sourceforge.net/projects/mysqmproesp32/files/PCB%20mySQMPLUS-ESP32/
#define WEMOS_SOLDERLESS 81     // https://sourceforge.net/projects/mysqmproesp32/files/WEMOS%20R3/
#define Petes_ESP32_Pi_Board 82 // https://sourceforge.net/projects/mysqmproesp32/files/PCB-Petes-ESP32-PiZero/

//-----------------------------------------------------------------------
// myESP32-DB boards
// https://sourceforge.net/projects/mysqmproesp32/
//-----------------------------------------------------------------------
// PCB https://sourceforge.net/projects/myesp-boards/files/myESP32DB/PCB/
#define ESP32_DB 83

//-----------------------------------------------------------------------
// CUSTOM BOARDS
//-----------------------------------------------------------------------
#define CUSTOMBRD 99 // For a user custom board see 0.jsn in /data/boards folder

//-----------------------------------------------------------------------
// DEPRECATED BOARDS
//-----------------------------------------------------------------------
// #define DRV8825_TFT22 106               // DRV8825_TFT22 Stripboard: BUZ.LEDs.TFT.PB.TMP
// #define DRV8825_HW203_TFT22 115         // DRV8825_HW203_TFT22 Stripboard: BUZ.LEDs.TFT.PB.TMP
// #define DRV8825_HW203_OLED_RE_REV4 170  // DRV8825_HW203_OLED_RE_REV4 DIY PCB: BUZ.LEDs.LCD.OLED.HPSW.PB.SPWR.TMP.RE
// #define EASYDRIVER_M 171				  // EasyDriver Stripboard: BUZ.LEDs
// #define L293DMINI_TFT 172				  // L293DMini_TFT Stripboard: BUZZ.LEDs.TFT22.PB.TMP
// #define L293DMINI_TFT22 174             // L293DMINI_TFT Stripboard: BUZ.LEDs.TFT.PB.TMP
// #define RAPS128 173					  // RAPS128 PCB:
// #define PRO2EDRV8825BIG 12              // if using a DRV8825 you also need to set DRV8825STEPMODE in myBoards.h

#endif
