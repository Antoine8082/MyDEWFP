//-----------------------------------------------------------------------
// myFocuserPro2 Display Routines
// (c) R Brown, 2014-2024, All rights reserved.
//-----------------------------------------------------------------------
#ifndef display_h
#define display_h

#include <Arduino.h>
#include "config.h"

#if defined(DISPLAYTYPE)

#if (DISPLAYTYPE == DISPLAY_LCD1602) || (DISPLAYTYPE == DISPLAY_LCD1604) || (DISPLAYTYPE == DISPLAY_LCD2004)
// LCD1602 16 characters (0-15), 2 lines (0-1)
// LCD1604 16 characters (0-15), 4 lines (0-3)
// LCD2004 20 characters (0-19), 4 lines (0-3)
// I2C Interface
// LCDADDRESS 0x27
// some LCD displays maybe at 0x3F, use I2Cscanner to find the correct address
#include <Wire.h>  // needed for I2C, installed when installing the Arduino IDE
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C* _Display;
char _clearstr[] = "               ";
char _postxt[] = "POS=";
char _targtxt[] = "TAR=";
char _coilpwr[] = "CPwr =";
char _reverse[] = "RDir =";
char _maxstep[] = "MaxSt=";
char _stepmode[] = "StepM=";
char _blin[] = "BLIN =";
char _blout[] = "BLOUT=";
char _mspeed[] = "MSpd =";
char _temp[] = "Temp =";
char _dash[] = "--";
char _tempcomp[] = "TComp=";
char _tempmode[] = "TMode=";
char _tcdir[] = "TCDir=";
char _tccoefficient[] = "TCCo#=";
char _Celsius[] = " c";
char _Fahrenheit[] = " f";
#endif

#if (DISPLAYTYPE == DISPLAY_OLED12864)
// 128x64 pixels
// 16 chars, 8 lines
// I2C Interface
// OLED_ADDR 0x3C         // some OLED displays maybe at 0x3F, use I2Cscanner to find the correct address
#include <Wire.h>
#include <mySSD1306Ascii.h>
#include <mySSD1306AsciiWire.h>
SSD1306AsciiWire* _Display;
char _clearstr[] = "               ";
char _postxt[] = "POS=";
char _targtxt[] = "TAR=";
char _coilpwr[] = "CPwr =";
char _reverse[] = "RDir =";
char _maxstep[] = "MaxSt=";
char _stepmode[] = "StepM=";
char _blin[] = "BLIN =";
char _blout[] = "BLOUT=";
char _mspeed[] = "MSpd =";
char _temp[] = "Temp =";
char _dash[] = "--";
char _tempcomp[] = "TComp=";
char _tempmode[] = "TMode=";
char _tcdir[] = "TCDir=";
char _tccoefficient[] = "TCCo#=";
char _Celsius[] = " c";
char _Fahrenheit[] = " f";
#endif

#if (DISPLAYTYPE == DISPLAY_NOKIA)
// 84x48 Pixels
// 14 chars, 6 lines (using 6x8 Fonts)
// 6x8 Fonts
// 12x16 Medium Numbers
// 14x24 Big Numbers
// SPI Interface
#include <LCD5110_Basic.h>
LCD5110* _Display;
extern uint8_t SmallFont[];
char yesstr[] = "Yes";
char nostr[] = "No";
#endif


//-----------------------------------------------------------------------
// EXTERN DATA AND ROUTINES
//-----------------------------------------------------------------------
extern bool display_found;
extern byte tprobe1;
extern float temp;
extern byte tempcompstate;
extern long fcurrentposition;  // current position
extern long ftargetposition;   // target position
extern char firmwareName[];
extern char firmwareVersion[];
extern struct config_t myfocuser;

//-----------------------------------------------------------------------
// VARS
//-----------------------------------------------------------------------
char T_NOTFOUND[] = "Display NOT found";


//-----------------------------------------------------------------------
// CODE
//-----------------------------------------------------------------------
#if (DISPLAYTYPE == DISPLAY_LCD1602) || (DISPLAYTYPE == DISPLAY_LCD1604) || (DISPLAYTYPE == DISPLAY_LCD2004)

void DisplayInit(void) {

  Wire.begin();

  // check if display is present
  Wire.beginTransmission(LCDADDRESS);
  if (Wire.endTransmission() != 0) {
    DebugPrintln(T_NOTFOUND);
    display_found = false;
    return;
  }

  display_found = true;

  _Display = new LiquidCrystal_I2C(LCDADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#if (DISPLAYTYPE == DISPLAY_LCD1602)
  _Display->begin(16, 2);
#endif

#if (DISPLAYTYPE == DISPLAY_LCD1604)
  _Display->begin(16, 4);
#endif

#if (DISPLAYTYPE == DISPLAY_LCD2004)
  _Display->begin(20, 4);
#endif

  _Display->setBacklight(HIGH);
  _Display->clear();
  _Display->print(firmwareName);
  _Display->setCursor(0, 1);
  _Display->print(firmwareVersion);
  delay(250);
}

//-----------------------------------------------------------------------
// LCD2004
//-----------------------------------------------------------------------
#if (DISPLAYTYPE == DISPLAY_LCD2004)
void displaypg1() {
  _Display->print(_postxt);
  _Display->print(fcurrentposition);
  _Display->setCursor(0, 1);
  _Display->print(_targtxt);
  _Display->print(ftargetposition);
  _Display->setCursor(0, 2);
  _Display->print(_coilpwr);
  _Display->print(myfocuser.coilpower);

  _Display->setCursor(10, 2);
  _Display->print(_reverse);
  _Display->print(myfocuser.reversedirection);

  _Display->setCursor(0, 3);
  _Display->print(_temp);
  if (tprobe1 == 0) {
    _Display->print(_dash);
  } else {
    float tempval = temp;
    if (myfocuser.tempmode == 0) {
      tempval = (tempval * 1.8) + 32;
    }
    _Display->print(tempval);
  }
}

void displaypg2() {
  _Display->print(_mspeed);
  _Display->print(myfocuser.motorspeed);

  _Display->setCursor(0, 1);
  _Display->print(_tccoefficient);
  _Display->print(myfocuser.tempcoefficient);

  _Display->setCursor(0, 2);
  _Display->print(_tempcomp);
  _Display->print(tempcompstate);

  _Display->setCursor(0, 3);
  _Display->print(_tempmode);
  if (myfocuser.tempmode == CELCIUS) {
    _Display->print(_Celsius);
  } else {
    _Display->print(_Fahrenheit);
  }
}

void displaypg3() {
  _Display->print(_maxstep);
  _Display->print(myfocuser.maxstep);
  _Display->setCursor(0, 1);
  _Display->print(_stepmode);
  _Display->print(myfocuser.stepmode);
  _Display->setCursor(0, 2);
  _Display->print(_blin);
  _Display->print(myfocuser.backlash_in_enabled);
  _Display->setCursor(9, 2);
  _Display->print(myfocuser.backlashsteps_in);
  _Display->setCursor(0, 3);
  _Display->print(_blout);
  _Display->print(myfocuser.backlash_out_enabled);
  _Display->setCursor(9, 3);
  _Display->print(myfocuser.backlashsteps_out);
}
#endif  // LCD2004


//-----------------------------------------------------------------------
// LCD1604
//-----------------------------------------------------------------------
#if (DISPLAYTYPE == DISPLAY_LCD1604)

void displaypg1() {
  _Display->print(_postxt);
  _Display->print(fcurrentposition);
  _Display->setCursor(0, 1);
  _Display->print(_targtxt);
  _Display->print(ftargetposition);

  _Display->setCursor(0, 2);
  _Display->print(_coilpwr);
  _Display->print(myfocuser.coilpower);
  _Display->setCursor(7, 2);
  _Display->print(_reverse);
  _Display->print(myfocuser.reversedirection);

  _Display->setCursor(0, 3);
  _Display->print(_mspeed);
  _Display->print(myfocuser.motorspeed);
  _Display->setCursor(7, 3);
  _Display->print(_stepmode);
  _Display->print(myfocuser.stepmode);
}

void displaypg2() {
  _Display->print(_temp);
  if (tprobe1 == 0) {
    _Display->print(_dash);
  } else {
    float tempval = temp;
    if (myfocuser.tempmode == FAHRENHEIT) {
      tempval = (tempval * 1.8) + 32;
    }
    _Display->print(tempval);
  }
  _Display->setCursor(0, 1);
  _Display->print(_tccoefficient);
  _Display->print(myfocuser.tempcoefficient);

  _Display->setCursor(0, 2);
  _Display->print(_tempcomp);
  _Display->print(tempcompstate);

  _Display->setCursor(0, 3);
  _Display->print(_tempmode);
  if (myfocuser.tempmode == 1) {
    _Display->print(_Celsius);
  } else {
    _Display->print(_Fahrenheit);
  }
}

void displaypg3() {
  _Display->print(_maxstep);
  _Display->print(myfocuser.maxstep);
  _Display->setCursor(0, 1);
  _Display->print(_blin);
  _Display->print(myfocuser.backlash_in_enabled);
  _Display->setCursor(9, 1);

  _Display->print(myfocuser.backlashsteps_in);
  _Display->setCursor(0, 2);
  _Display->print(_blout);
  _Display->print(myfocuser.backlash_out_enabled);
  _Display->setCursor(9, 2);
  _Display->print(myfocuser.backlashsteps_out);
}
#endif  // LCD1604


//-----------------------------------------------------------------------
// LCD1602
//-----------------------------------------------------------------------
#if (DISPLAYTYPE == DISPLAY_LCD1602)

void displaypg1() {
  _Display->print(_postxt);
  _Display->print(fcurrentposition);

  _Display->setCursor(0, 1);
  _Display->print(_targtxt);
  _Display->print(ftargetposition);
}

void displaypg2() {
  _Display->print(_coilpwr);
  _Display->print(myfocuser.coilpower);

  _Display->setCursor(0, 1);
  _Display->print(_reverse);
  _Display->print(myfocuser.reversedirection);
}

void displaypg3() {
  // display values on LCD1602
  _Display->print(_temp);
  if (tprobe1 == 0) {
    _Display->print(_dash);
  } else {
    float tempval = temp;
    if (myfocuser.tempmode == FAHRENHEIT) {
      tempval = (tempval * 1.8) + 32;
    }
    _Display->print(tempval);
  }
  _Display->setCursor(0, 1);
  _Display->print(_maxstep);
  _Display->print(myfocuser.maxstep);
}

void displaypg4() {
  _Display->print(_mspeed);
  _Display->print(myfocuser.motorspeed);

  _Display->setCursor(0, 1);
  _Display->print(_stepmode);
  _Display->print(myfocuser.stepmode);
}

void displaypg5() {
  _Display->print(_blin);
  _Display->print(myfocuser.backlash_in_enabled);
  _Display->setCursor(9, 0);
  _Display->print(myfocuser.backlashsteps_in);

  _Display->setCursor(0, 1);
  _Display->print(_blout);
  _Display->print(myfocuser.backlash_out_enabled);
  _Display->setCursor(9, 1);
  _Display->print(myfocuser.backlashsteps_out);
}

void displaypg6() {
  _Display->print(_tempcomp);
  _Display->print(tempcompstate);

  _Display->setCursor(0, 1);
  _Display->print(_tempmode);
  if (myfocuser.tempmode == 1) {
    _Display->print(_Celsius);
  } else {
    _Display->print(_Fahrenheit);
  }
}

#endif

//-----------------------------------------------------------------------
// UPDATE FOCUSER POSITION AND TARGET POSITION
//-----------------------------------------------------------------------
void DisplayUpdatePosition() {
  if (display_found == false) {
    return;
  }

  _Display->setCursor(0, 0);
  _Display->print(_clearstr);

  _Display->setCursor(0, 0);
  _Display->print(_postxt);
  _Display->print(fcurrentposition);

  _Display->setCursor(0, 1);
  _Display->print(_clearstr);

  _Display->setCursor(0, 1);
  _Display->print(_targtxt);
  _Display->print(ftargetposition);
}

//-----------------------------------------------------------------------
// UPDATE DISPLAY PAGE
//-----------------------------------------------------------------------
void DisplayUpdatePages(void) {
  unsigned static int displaybitmask = 1;

  if (display_found == false) {
    DebugPrintln("display NOT Found");
    return;
  }

  DebugPrintln("DIS002");

  _Display->clear();  // clrsc
  unsigned int page = 0;
  String mypage = String(myfocuser.display_option, BIN);

#if (DISPLAYTYPE == DISPLAY_LCD1602)
  for (unsigned int i = 0; i < mypage.length(); i++) {
    page *= 2;
    if (mypage[i] == '1') {
      page++;
    }
  }
  // find the next page to display
  // mask off one bit at a time
  while ((page & displaybitmask) == 0) {
    displaybitmask *= 2;  // skip this page as it is not enabled
    // pages 1 to 6 [note: displaybitmask is multiples of 2]
    if (displaybitmask > 32) {
      displaybitmask = 1;
      break;
    }
  }  // while((screenmask & displaybitmask) == 0)

  // displaybitmask is now the page to display, 1=pg1, 2=pg2, 4=pg3, 8=pg4 etc
  switch (displaybitmask) {
    case 1: displaypg1(); break;
    case 2: displaypg2(); break;
    case 4: displaypg3(); break;
    case 8: displaypg4(); break;
    case 16: displaypg5(); break;
    case 32: displaypg6(); break;
    default: displaypg1(); break;
  }
  displaybitmask *= 2;  // next page
#endif                  // lcd 1602

#if (DISPLAYTYPE == DISPLAY_LCD1604)
  for (unsigned int i = 0; i < mypage.length(); i++) {
    page *= 2;
    if (mypage[i] == '1') {
      page++;
    }
  }
  // find the next page to display
  while ((page & displaybitmask) == 0)  // mask off one bit at a time
  {
    displaybitmask *= 2;  // skip this page as it is not enabled
    // pages 1 to 3
    if (displaybitmask > 4) {
      displaybitmask = 1;
      break;
    }
  }                        // while((screenmask & displaybitmask) == 0)
  switch (displaybitmask)  // displaybitmask is now the page to display, 1=pg1, 2=pg2, 4=pg3, 8=pg4 etc
  {
    case 1: displaypg1(); break;
    case 2: displaypg2(); break;
    case 4: displaypg3(); break;
    default: displaypg1(); break;
  }
  displaybitmask *= 2;  // next page
#endif                  // lcd1604

#if (DISPLAYTYPE == DISPLAY_LCD2004)
  for (unsigned int i = 0; i < mypage.length(); i++) {
    page *= 2;
    if (mypage[i] == '1') {
      page++;
    }
  }
  // find the next page to display
  while ((page & displaybitmask) == 0)  // mask off one bit at a time
  {
    displaybitmask *= 2;     // skip this page as it is not enabled
    if (displaybitmask > 4)  // pages 1 to 3
    {
      displaybitmask = 1;
      break;
    }
  }                        // while((screenmask & displaybitmask) == 0)
  switch (displaybitmask)  // displaybitmask is now the page to display, 1=pg1, 2=pg2, 4=pg3, 8=pg4 etc
  {
    case 1: displaypg1(); break;
    case 2: displaypg2(); break;
    case 4: displaypg3(); break;
    default: displaypg1(); break;
  }
  displaybitmask *= 2;  // next page
#endif                  // lcd2004
}
#endif


//-----------------------------------------------------------------------
// OLED DISPLAY ROUTINES
//-----------------------------------------------------------------------
#if (DISPLAYTYPE == DISPLAY_OLED12864)

void DisplayInit(void) {

  Wire.begin();

  // check if display is present
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    DebugPrintln(T_NOTFOUND);
    display_found = false;
    return;
  }

  // Setup the OLED, screen size is 128 x 64, so using characters at 6x8 this gives 21chars across and 8 lines down
  display_found = true;
  _Display = new SSD1306AsciiWire();  // instantiate the OLED object
#ifdef USE_SSD1306
  // For the OLED 128x64 0.96" display using the SSD1306 driver
  _Display->begin(&Adafruit128x64, OLED_ADDR);
#endif  // #ifdef USE_SSD1306
#ifdef USE_SSH1106
  // For the OLED 128x64 1.3" display using the SSH1106 driver
  _Display->begin(&SH1106_128x64, OLED_ADDR);
#endif  // #ifdef USE_SSH1106
  _Display->set400kHz();
  _Display->setFont(Adafruit5x7);
  _Display->clear();            // clrscr OLED
  _Display->Display_Normal();   // black on white
  _Display->Display_On();       // display ON
  _Display->Display_Rotate(0);  // portrait, not rotated
  _Display->Display_Bright();
  _Display->println(firmwareName);  // print startup screen
  _Display->println(firmwareVersion);
  delay(250);
}

void displaypg1(void) {
  _Display->home();
  _Display->print(_postxt);
  _Display->print(fcurrentposition);
  _Display->clearToEOL();

  _Display->println();
  _Display->print(_targtxt);
  _Display->print(ftargetposition);
  _Display->clearToEOL();
  _Display->println();

  _Display->print(_coilpwr);
  _Display->print(myfocuser.coilpower);
  _Display->clearToEOL();
  _Display->println();

  _Display->print(_reverse);
  _Display->print(myfocuser.reversedirection);
  _Display->clearToEOL();
  _Display->println();

  // stepmode setting
  _Display->print(_stepmode);
  _Display->print(myfocuser.stepmode);
  _Display->clearToEOL();
  _Display->println();

  // Temperature
  _Display->print(_temp);
  _Display->print(temp);
  _Display->print(_Celsius);
  _Display->clearToEOL();
  _Display->println();

  // Motor Speed
  _Display->print(_mspeed);
  _Display->print(myfocuser.motorspeed);
  _Display->clearToEOL();
  _Display->println();

  // MaxStep
  _Display->print(_maxstep);
  _Display->print(myfocuser.maxstep);
  _Display->clearToEOL();
  _Display->println();
}

void displaypg2(void) {
  // temperature compensation
  _Display->print(_tccoefficient);
  _Display->print(myfocuser.tempcoefficient);
  _Display->clearToEOL();
  _Display->println();
  _Display->print(_tempcomp);
  _Display->print(tempcompstate);
  _Display->clearToEOL();
  _Display->println();
  _Display->print(_tcdir);
  _Display->print(myfocuser.tcdirection);
  _Display->clearToEOL();
  _Display->println();
  _Display->print(_blin);
  _Display->print(myfocuser.backlash_in_enabled);
  _Display->clearToEOL();
  _Display->println();
  _Display->print(_blout);
  _Display->print(myfocuser.backlash_out_enabled);
  _Display->clearToEOL();
  _Display->println();
  _Display->print(_blin);
  _Display->print(myfocuser.backlashsteps_in);
  _Display->clearToEOL();
  _Display->println();
  _Display->print(_blout);
  _Display->print(myfocuser.backlashsteps_out);
  _Display->clearToEOL();
  _Display->println();
#ifndef BLUETOOTH
  _Display->print("Serial Mode");
#else
  _Display->print("Bluetooth Mode");
#endif  // #ifndef BLUETOOTH
  _Display->clearToEOL();
}

//-----------------------------------------------------------------------
// UPDATE FOCUSER POSITION AND TARGET POSITION
//-----------------------------------------------------------------------
void DisplayUpdatePosition() {
  if (display_found == false) {  
      return;
  }
  _Display->setCursor(0, 0);
  _Display->print(_postxt);
  _Display->print(fcurrentposition);
  _Display->clearToEOL();
  _Display->println();

  _Display->print(_targtxt);
  _Display->print(ftargetposition);
  _Display->clearToEOL();
  _Display->println();
}

//-----------------------------------------------------------------------
// UPDATE DISPLAY PAGE
//-----------------------------------------------------------------------
void DisplayUpdatePages(void) {
  unsigned static int displaybitmask = 1;

  if (display_found == false) {
    return;
  }

  DebugPrintln("DIS002");
  _Display->clear();  // clrscr OLED
  unsigned int page = 0;
  String mypage = String(myfocuser.display_option, BIN);
  for (unsigned int i = 0; i < mypage.length(); i++) {
    page *= 2;
    if (mypage[i] == '1') {
      page++;
    }
  }
  // find the next page to display, mask off one bit at a time
  while ((page & displaybitmask) == 0) {
    displaybitmask *= 2;     // skip this page as it is not enabled
    if (displaybitmask > 2)  // pages 1 to 2
    {
      displaybitmask = 1;
      break;
    }
  }                        // while((screenmask & displaybitmask) == 0)
  switch (displaybitmask)  // displaybitmask is now the page to display, 1=pg1, 2=pg2, 4=pg3, 8=pg4 etc
  {
    case 1: displaypg1(); break;
    case 2: displaypg2(); break;
    default: displaypg1(); break;
  }
  displaybitmask *= 2;  // next page
}
#endif  // OLED12864


//-----------------------------------------------------------------------
// NOKIA DISPLAY ROUTINES
//-----------------------------------------------------------------------
#if (DISPLAYTYPE == DISPLAY_NOKIA)

void DisplayInit(void) {
  _Display = new LCD5110(8, 9, 10, 11, 12);
  _Display->InitLCD();
  _Display->setFont(SmallFont);
  _Display->clrScr();  // print the title of program and copyright message to lcd
  display_found = true;
  _Display->print(firmwareName, LEFT, 0);
  _Display->print(firmwareVersion, LEFT, 8);
  delay(250);
}

void displaypg1() {
  String str;
  char tempString[8];
  float tempval = 20.0;

  str = "Posn= " + String(fcurrentposition);
  _Display->print(str, LEFT, 0);
  str = "Targ= " + String(ftargetposition);
  _Display->print(str, LEFT, 8);
  str = "CPwr= ";
  if (myfocuser.coilpower == 1) {
    str = str + yesstr;
  } else {
    str = str + nostr;
  }
  _Display->print(str, LEFT, 16);
  str = "RDir= ";
  if (myfocuser.reversedirection == 1) {
    str = str + yesstr;
  } else {
    str = str + nostr;
  }
  _Display->print(str, LEFT, 24);

  str = "Temp= ";
  if (tprobe1 == 0) {
    str = str + "--";
  } else {
    tempval = temp;
    if (myfocuser.tempmode == 1) {
      /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
      dtostrf(tempval, 4, 2, tempString);
      str = str + tempString;
    } else {
      tempval = (tempval * 1.8) + 32;
      dtostrf(tempval, 4, 2, tempString);
      str = str + tempString;
    }
  }
  _Display->print(str, LEFT, 32);

  switch (myfocuser.motorspeed) {
    case SLOW: str = "MSpd= Slow"; break;
    case MED: str = "MSpd= Med"; break;
    case FAST: str = "MSpd= Fast"; break;
  }
  _Display->print(str, LEFT, 40);
}

void displaypg2() {
  String str;
  char tempString[8];

  str = "StepMd=" + String(myfocuser.stepmode);
  _Display->print(str, LEFT, 0);

  // stepsize
  str = "StepSi= ";
  /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
  dtostrf(myfocuser.stepsize, 4, 2, tempString);
  str = str + tempString;
  _Display->print(str, LEFT, 8);

  str = "Maxpos=" + String(myfocuser.maxstep);
  _Display->print(str, LEFT, 16);

  // temp comp state
  if (tempcompstate == 1) {
    str = "TComp =Yes";
  } else {
    str = "TComp =No";
  }
  _Display->print(str, LEFT, 24);

  // temp coefficient
  str = "TCoeff= ";
  /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
  dtostrf(myfocuser.tempcoefficient, 4, 2, tempString);
  str = str + tempString;
  _Display->print(str, LEFT, 32);
}

void displaypg3() {
  String str;
  char tempString[8];

  str = "BLIn = ";
  if (myfocuser.backlash_in_enabled == 0) {
    str = str + '0';
  } else {
    str = str + '1';
  }
  _Display->print(str, LEFT, 0);

  str = "BLOut= ";
  if (myfocuser.backlash_out_enabled == 0) {
    str = str + '0';
  } else {
    str = str + '1';
  }
  _Display->print(str, LEFT, 8);

  str = "BLIn#= " + String(myfocuser.backlashsteps_in);
  _Display->print(str, LEFT, 16);

  str = "BLOu#= " + String(myfocuser.backlashsteps_out);
  _Display->print(str, LEFT, 24);
}

//-----------------------------------------------------------------------
// UPDATE FOCUSER POSITION AND TARGET POSITION
//-----------------------------------------------------------------------
void DisplayUpdatePosition() {
  if (display_found == false) {
    return;
  }

  String str;
  char tmpStr[] = "               ";

  _Display->print(tmpStr, LEFT, 0);
  str = "POS=" + String(fcurrentposition);
  _Display->print(str, LEFT, 0);

  _Display->print(tmpStr, LEFT, 0);
  str = "TAR=" + String(ftargetposition);
  _Display->print(str, LEFT, 8);
}

//-----------------------------------------------------------------------
// UPDATE DISPLAY PAGE
//-----------------------------------------------------------------------
void DisplayUpdatePages(void) {
  unsigned static int displaybitmask = 1;

  if (display_found == false) {
    return;
  }

  DebugPrintln("DIS002");
  _Display->clrScr();           // clrscr
  unsigned int page = 0;
  String mypage = String(myfocuser.display_option, BIN);
  for (unsigned int i = 0; i < mypage.length(); i++) {
    page *= 2;
    if (mypage[i] == '1') {
      page++;
    }
  }
  // find the next page to display
  // mask off one bit at a time
  while ((page & displaybitmask) == 0) {
    // skip this page as it is not enabled
    displaybitmask *= 2;
    // pages 1 to 3
    if (displaybitmask > 4) {
      displaybitmask = 1;
      break;
    }
  }  // while((screenmask & displaybitmask) == 0)

  // displaybitmask is now the page to display, 1=pg1, 2=pg2, 4=pg3, 8=pg4 etc
  switch (displaybitmask) {
    case 1: displaypg1(); break;
    case 2: displaypg2(); break;
    case 4: displaypg3(); break;
    default: displaypg1(); break;
  }
  displaybitmask *= 2;  // next page
}
#endif  // NOKIA


#endif

#endif
