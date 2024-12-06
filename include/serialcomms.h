//-----------------------------------------------------------------------
// myFocuserPro2 Comms Routines
// (c) R Brown, 2014-2024, All rights reserved.
// (c) Holger Manz, 2019-2022, All rights reserved.
// (c) Joel Collet, move-timer, 2021-2022, All rights reserved
//-----------------------------------------------------------------------
#ifndef serialcomms_h
#define serialcomms_h


//-----------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
#include <myQueue.h>  // Steven de Salas


//-----------------------------------------------------------------------
// EXTERNS
//-----------------------------------------------------------------------
extern void software_Reboot();
extern void setfocuserdefaults();
extern void writeEEPROMNow();
extern void updatestepperpowerdetect();

extern long ftargetposition;
extern long fcurrentposition;
extern byte isMoving;
extern volatile bool hpswstate;
extern byte stepperpower;
extern byte jogging;
extern byte joggingDirection;

extern byte display_pages;
extern byte display_option;

extern byte tprobe1;
extern float temp;
extern byte tempcompstate;

extern byte writenow;
extern char firmwareName[];
extern char firmwareVersion[];

extern Queue<String> queue;
extern String line;

#if defined(DISPLAYTYPE)
extern void DisplayUpdatePosition();
#endif


//-----------------------------------------------------------------------
// SERIAL COMMUNICATIONS
//-----------------------------------------------------------------------
void SendMessage(const char *str) {
  DebugPrint("SER002 ");
  DebugPrintln(str);
  // for serial mode
  Serial.print(str);
}

void SendPacket(const char token, const char *str) {
  char buff[32];
  snprintf(buff, sizeof(buff), "%c%s%c", token, str, EOC);
  SendMessage(buff);
}

void SendPacket(const char token, const int val) {
  char buff[32];
  snprintf(buff, sizeof(buff), "%c%d%c", token, val, EOC);
  SendMessage(buff);
}

void SendPacket(const char token, double val) {
  char result[8];
  dtostrf(val, 2, 2, result);
  SendPacket(token, result);
}

void SendPacket(const char token, const long val) {
  char buff[32];
  snprintf(buff, sizeof(buff), "%c%lu%c", token, val, EOC);
  SendMessage(buff);
}

void SendPacket(const char token, const unsigned long val) {
  char buff[32];
  snprintf(buff, sizeof(buff), "%c%lu%c", token, val, EOC);
  SendMessage(buff);
}

//-----------------------------------------------------------------------
// COMMAND HANDLER
//-----------------------------------------------------------------------
void ser_comms() {
  int cmdval;
  String receiveString = "";
  String WorkString = "";
  byte bval;
  int ival;
  long tmppos;

  if (queue.count() == 0) {
    return;
  }

  receiveString = (String)queue.pop();
  cmdval = receiveString.substring(0, 2).toInt();
  WorkString = receiveString.substring(2, receiveString.length());
  DebugPrint("recstr=" + receiveString);
  DebugPrint(", cmdval=" + String(cmdval));
  DebugPrintln(", WorkStr=" + WorkString);

  switch (cmdval) {
    case 0:  // Get focuser position
      SendPacket('P', fcurrentposition);
      break;

    case 1:  // ismoving
      SendPacket('I', isMoving);
      break;

    case 2:  // Get controller status
      SendPacket('E', "OK");
      break;

    case 3:  // Get firmware version
      SendPacket('F', firmwareVersion);
      break;

    case 4:  // Get firmware name
      {
        char buff[32];
        snprintf(buff, sizeof(buff), "%s%c%c%s", firmwareName, '\r', '\n', firmwareVersion);
        SendPacket('F', buff);
      }
      break;

    case 5:  // Set new target position to xxxxxx (and focuser initiates immediate move to xxxxxx)
      // only if not already moving
      if (isMoving == 0) {
        if (stepperpower == 1) {
          tmppos = (long) WorkString.toInt();
          tmppos = (tmppos < 0) ? 0 : tmppos;
          tmppos = (tmppos > myfocuser.maxstep) ? myfocuser.maxstep : tmppos;
          ftargetposition = tmppos;
          DebugPrint("Move to ");
          DebugPrintln(tmppos);
#if defined(DISPLAYTYPE)
          DisplayUpdatePosition();
#endif
          writenow = 1;
          isMoving = 1;
      }
    }
    break;
            
    case 6:  // Get temperature
      SendPacket('Z', temp);
      break;

    case 7:  // Set MaxStep between 1000 and 2000000
      tmppos = (long) WorkString.toInt();
      tmppos = (tmppos >= FOCUSERUPPERLIMIT) ? FOCUSERUPPERLIMIT : tmppos;
      tmppos = (tmppos < fcurrentposition) ? fcurrentposition : tmppos;
      tmppos = (tmppos < FOCUSERLOWERLIMIT) ? FOCUSERLOWERLIMIT : tmppos;
      myfocuser.maxstep = tmppos;
      writenow = 1;
      break;

    case 8:  // Get maxStep
      SendPacket('M', myfocuser.maxstep);
      break;

    // Note: Case 9 are not used in ASCOM, Windows, INDI and Linux app
    //case 9:  // Get inoutled mode, pulse or move
      // ignore, for ESP32
      //  SendPacket('$', 0);
      //  break;

    case 10:  // Get maxIncrement
      SendPacket('Y', myfocuser.maxstep);
      break;

    case 11:  // Get coilpower setting, 0 off, 1 on
      SendPacket('O', myfocuser.coilpower);
      break;

    case 12:  // Set coil power
      myfocuser.coilpower = (byte) WorkString[0] - '0';
      (myfocuser.coilpower == 1) ? driverboard->set_motorpower(true) : driverboard->set_motorpower(false);
      writenow = 1;
      break;

    case 13:  // Get reverse direction setting, 0 off, 1 on
      SendPacket('R', myfocuser.reversedirection);
      break;

    case 14:  // Set reverse direction
      if (isMoving == 0) {
        myfocuser.reversedirection = (byte) WorkString[0] - '0';
        writenow = 1;
      }
      break;

    case 15:  // Set motorspeed
      myfocuser.motorspeed = (byte) WorkString.toInt() & 3;
      writenow = 1;
      break;

    case 16: // Set display to celsius
      myfocuser.tempmode = 1; 
      writenow = 1;
      break;

    case 17: // Set display to fahrenheit
      myfocuser.tempmode = 0;
      writenow = 1;
      break;

    case 18:
      // :180#    None    set the return of user specified stepsize to be OFF - default
      // :181#    None    set the return of user specified stepsize to be ON - reports what user specified as stepsize
      myfocuser.stepsizeenabled = WorkString[0] - '0';
      writenow = 1;
      break;

    case 19:  // Set the step size value - double type, eg 2.1
      {
        double tempstepsize = WorkString.toDouble();
        tempstepsize = (tempstepsize < 0) ? 0.0 : tempstepsize;
        tempstepsize = (tempstepsize > MAXIMUMSTEPSIZE) ? MAXIMUMSTEPSIZE : tempstepsize;
        myfocuser.stepsize = tempstepsize;
      }
      writenow = 1;
      break;

    case 20:  // Set the temperature resolution setting for the DS18B20 temperature probe
      ival = WorkString.toInt();
      ival = (ival < 9) ? 9 : ival;
      ival = (ival > 12) ? 12 : ival;
      myfocuser.tempresolution = (byte)ival;
#if defined(TEMPERATURE_PROBE)
      // to reprogram the temperature probe with new resolution
      sensor1.setResolution(tpAddress, myfocuser.tempresolution);
#endif
      writenow = 1;
      break;

    case 21:  // Get temp probe resolution
      SendPacket('Q', myfocuser.tempresolution);
      break;

    case 22:  // Set the temperature coefficient to 0-1000
      ival = WorkString.toInt();  // 0-511
      if (ival < 0)
        ival = 0;
      if (ival > 1000)
        ival = 1000;
      myfocuser.tempcoefficient = ival;
      writenow = 1;
      break;

    case 23:  // Set the temperature compensation state, ON (1) or OFF (0)
      if (tprobe1 != 0) {
        tempcompstate = (byte) WorkString[0] - '0';
        writenow = 1;
      }
      break;

    case 24:  // Get status of temperature compensation (enabled | disabled)
      SendPacket('1', tempcompstate);
      break;

    case 25:  // Get IF temperature compensation is available
      SendPacket('A', tprobe1);  
      break;

    case 26:  // Get temperature coefficient steps/degree
      SendPacket('B', myfocuser.tempcoefficient);
      break;

    case 27:  // stop a move - like a Halt
      ftargetposition = fcurrentposition;
      isMoving = 0;
      writenow = 1;
      break;

    case 28:  // home the motor to position 0
      if (isMoving == 0) {
        if (stepperpower == 1) {
          HPSWDebugPrintln(":28 Go HOME");
          ftargetposition = 0;
          isMoving = 1;
        }
      }
      break;

    case 29:  // Get stepmode
      SendPacket('S', myfocuser.stepmode);
      break;

    case 30:  // Set step mode
      ival = WorkString.toInt();
      ival = (ival < STEP1) ? STEP1 : ival;
      ival = (ival > MAXSTEPMODE) ? MAXSTEPMODE : ival;
      myfocuser.stepmode = ival;
      driverboard->set_stepmode();
      writenow = 1;
      break;

    case 31:  // Set focuser position
      if (isMoving == 0) {
          tmppos = (long) WorkString.toInt();
          tmppos = (tmppos < 0) ? 0 : tmppos;
          tmppos = (tmppos > myfocuser.maxstep) ? myfocuser.maxstep : tmppos;
          myfocuser.fposition = tmppos;
          fcurrentposition = ftargetposition = tmppos;
      }
      writenow = 1;
      break;

    case 32:  // Get if stepsize is enabled
      SendPacket('U', myfocuser.stepsizeenabled);
      break;

    case 33:  // Get stepsize
      SendPacket('T', myfocuser.stepsize);
      break;

    case 34:  // Get the time that an LCD screen is displayed for
      SendPacket('X', (myfocuser.display_pagetime * 1000));
      break;

    case 35:  // Set length of time an LCD page is displayed for [2 - 10 meaning 2s to 10s]
      // page display time is now ready only, set value in config.h
      // range is 2 - 10 seconds, default is 5 seconds
      bval = (byte) WorkString.toInt();
      bval = (bval < DISPLAY_PAGETIME_MIN) ? DISPLAY_PAGETIME_MIN : bval;
      myfocuser.display_pagetime = (bval > DISPLAY_PAGETIME_MAX) ? DISPLAY_PAGETIME_MAX : bval;
      writenow = 1;
      break;

    case 36:
      // :360#    None    Disable Display
      // :361#    None    Enable Display
      myfocuser.display_enabled = (byte) WorkString[0] - '0';
#if defined(DISPLAYTYPE)
#if (DISPLAYTYPE == DISPLAY_LCD1602) || (DISPLAYTYPE == DISPLAY_LCD1604) || (DISPLAYTYPE == DISPLAY_LCD2004)
      if (myfocuser.display_enabled == 1) {
        // Set the text color to white
        _Display->display();
        _Display->backlight();
      } else {
        // disable the screen by setting clearing the screen and then the text color to black
        // clear the screen
        _Display->noDisplay();
        _Display->noBacklight();
      }
#elif (DISPLAYTYPE == DISPLAY_OLED12864)
      if (myfocuser.display_enabled == 1) {
        _Display->Display_On();
      } else {
        _Display->Display_Off();
      }
#endif
#endif 
      writenow = 1;
      break;

    case 37:  // Get displaystatus
      SendPacket('D', myfocuser.display_enabled);
      break;

    case 38:  // Get Temperature mode 1=Celsius, 0=Fahrenheight
      SendPacket('b', myfocuser.tempmode);
      break;

    case 39:  // Get the new motor position (target) XXXXXX
      SendPacket('N', ftargetposition);
      break;

    case 40:  // Reset Arduino controller
      software_Reboot();
      break;

      //case 41:  // Set in-out LED mode (pulsed or move)
      // ignore, for ESP32
      //break;

    case 42:  // Reset focuser defaults
      DebugPrintln("CFG002");
      setfocuserdefaults();
      ftargetposition = myfocuser.fposition;
      fcurrentposition = myfocuser.fposition;
      software_Reboot();
      break;

    case 43:  // Get motorspeed
      SendPacket('C', myfocuser.motorspeed);
      break;

    //case 44:  // Get Park enable state
      // ignore, for ESP32
      //SendPacket('G', 0);
      //break;

    //case 45:  // Set Park enable state
      // ignore, for ESP32
      //break;

    //case 46:  // Get in-out LED enable state
      // ignore, for ESP32
      //SendPacket('J', 0);
      //break;

    //case 47:  //  Set in-out LED enable state
      // ignore, for ESP32
      //break;

    case 48:  // save settings to EEPROM
      myfocuser.validdata = VALIDDATAFLAG;
      myfocuser.fposition = fcurrentposition;
      writeEEPROMNow();
      break;

    case 49:  // :49#         aXXXXX
      SendPacket('a', "b552efd");
      break;

    case 50:  // Get if HOMEPOSITIONSWITCH is enabled in firmware
#if defined(HOMEPOSITIONSWITCH)
      SendPacket('l', 1);
#else
      SendPacket('l', 0);
#endif
      break;

    //case 51:  // Get ESP32 WiFi Controller IP Address
      //esp32
      //SendPacket('$', "0.0.0.0");
      //break;

    //case 52:  // Get Park status (Not Parked, Parked)
      //esp32
      //SendPacket('$', 0);
      //break;

    case 53:  // Get number of pages to display
      SendPacket('$', display_pages);
      break;

    //case 54:  // Get ESP32 WiFi Controller SSID
      //SendPacket('$', "N/A");
      //break;

    case 55: // Get stepdelay
      SendPacket('0', myfocuser.stepdelay);
      break;

    case 56:  // set motorspeed delay for current speed setting
      {
        unsigned long msdelay = (unsigned long)WorkString.toInt();
        msdelay = (msdelay < 800) ? 800 : msdelay;
        msdelay = (msdelay > 12000) ? 12000 : msdelay;
        myfocuser.stepdelay = msdelay;
        writenow = 1;
      }
      break;

    //case 57:  // Get pushbutton enable state
      // ignore, for ESP32
      //SendPacket('$', 0);
      //break;

    //case 58:  // Set pushbutton enable state (0 or 1)
      // ignore, for ESP32
      //break;

    //case 59:  // Get Park time
      //SendPacket('$', 0);
      //break;

    //case 60:  // Set Park time (0-600)
      // ignore, for ESP32
      //break;

    case 61:  // Set update of position on lcd when moving (0=disable, 1=enable)
      myfocuser.display_updateonmove = (byte) WorkString[0] - '0';
      writenow = 1;
      break;

    case 62:  // Get update of position on lcd when moving (0=disable, 1=enable)
      // SendPacket('L', myfocuser.display_updateonmove);
      SendPacket('L', 0L);
      break;

    case 63:  // Get status of home position switch. high = closed, low = open
#if defined(HOMEPOSITIONSWITCH)
      hpswstate = driverboard->get_hpsw();
      HPSWDebugPrint("serialcomms: :63, get HPSW state: ");
      HPSWDebugPrintln(hpswstate);
#endif
      SendPacket('H', hpswstate);
      break;

    case 64:  // Move a specified number of steps
      if (isMoving == 0) {
        if (stepperpower == 1) {        
          tmppos = (long) WorkString.toInt() + fcurrentposition;
          tmppos = (tmppos < 0) ? 0 : tmppos;
          tmppos = (tmppos > myfocuser.maxstep) ? myfocuser.maxstep : tmppos;
          ftargetposition = tmppos;
#if defined(DISPLAYTYPE)
          DisplayUpdatePosition();
#endif
          writenow = 1;
          isMoving = 1;
        }
      }
      break;

    case 65:  // Set jogging state enable/disable
      if (stepperpower) {
        jogging = (byte) WorkString[0] - '0';
        writenow = 1;
      }
      break;

    case 66:  // Get jogging state enabled/disabled
      SendPacket('K', jogging);
      break;

    case 67:  // Set jogging direction, 0=IN, 1=OUT
      if (stepperpower) {
        joggingDirection = (byte) WorkString[0] - '0';
        writenow = 1;
      }
      break;

    case 68:  // Get jogging direction, 0=IN, 1=OUT
      SendPacket('V', joggingDirection);
      break;

    case 69:  // Get push button steps (0-255)
      SendPacket('?', myfocuser.pbsteps);      
      break;

    case 70:  // Set push button steps (0-255)
      bval = (byte) WorkString.toInt() & 255;
      myfocuser.pbsteps = (bval < 0) ? 0 : bval;
      writenow = 1;
      break;

    case 71:  // Set delayaftermove in milliseconds
      myfocuser.delayaftermove = (byte) WorkString.toInt() & 255;
      writenow = 1;
      break;

    case 72:  // Get delayaftermove
      SendPacket('3', myfocuser.delayaftermove);
      break;

    case 73:  // Set Disable/enable backlash IN (going to lower focuser position)
      myfocuser.backlash_in_enabled = (byte) WorkString[0] - '0';
      writenow = 1;
      break;

    case 74:  // Get backlash in enabled status
      SendPacket('4', myfocuser.backlash_in_enabled);
      break;

    case 75:  // Set Disable/enable backlash OUT (going to lower focuser position)
      myfocuser.backlash_out_enabled = (byte) WorkString[0] - '0';
      writenow = 1;
      break;

    case 76:  // Get backlash OUT enabled status
      SendPacket('5', myfocuser.backlash_out_enabled);
      break;

    case 77:  // Set backlash in steps
      myfocuser.backlashsteps_in = (byte) WorkString.toInt() & 255;
      writenow = 1;
      break;

    case 78:  // Get number of backlash steps IN
      SendPacket('6', myfocuser.backlashsteps_in);
      break;

    case 79:  // Set backlash OUT steps
      myfocuser.backlashsteps_out = (byte) WorkString.toInt() & 255;
      writenow = 1;
      break;

    case 80:  // Get number of backlash steps OUT
      SendPacket('7', myfocuser.backlashsteps_out);
      break;

    case 81:  // Get STALL_VALUE (for TMC2209 stepper modules), not used for myFP2M
      SendPacket('8', driverboard->get_stallguard());
      break;

    case 82:  // set STALL_VALUE (for TMC2209 stepper modules)
      driverboard->set_stallguard((byte)WorkString.toInt());
      break;

    case 83:  // Get if there is a temperature probe
      SendPacket('c', tprobe1);
      break;

    //case 84:  // Set Nextion Page
      // ignore
      //break;

    //case 85:  // Set Serial Port Speed [Nextion only]
      // ignore
      //  break;

    //case 86:  // Get Serial Port Speed [Nextion only]
      //SendPacket('$', 9600);
      //break;

    case 87:  // Get tc direction
      SendPacket('k', myfocuser.tcdirection);
      break;

    case 88:  // Set tc direction
      myfocuser.tcdirection = (byte) WorkString[0] - '0';
      writenow = 1;
      break;

    case 89:  // Get stepper power
      DebugPrintln("STP001");
      updatestepperpowerdetect();
      // for ULN2003 powered from  9V with 4.7VZ, reading was 3.72V = 763
      // for DRV8825 powered from 12V with 4.7VZ, reading was 4.07V = 834
      // Each digit = .00488millivolts
      SendPacket('9', stepperpower);
      break;

    //case 90:  // Set preset x [0-9] with position value yyyy [unsigned long]
      // ignore
      //  break;

    //case 91:  // Get focuserpreset x [0-9] value
      //SendPacket('$', 0);
      //break;

    case 92:  // Set page display option, input=string, output=decimal
      // if no display is enabled in firmware, display_pages = 0;
      if (display_pages == 0) {
        myfocuser.display_option = 0;
        break;
      } else {
        // truncate the string to be within the range of pages for the display type
        if (WorkString.length() > display_pages) {
          WorkString = WorkString.substring(0, display_pages);
        }
        // generate the decimal from the display option string
        int num = 0;
        for (int idx = 0; idx < (int) WorkString.length(); idx++) {
          num *= 2;
          if (WorkString[idx] == '1') num++;
        }
        myfocuser.display_option = (byte)num;
      }
      writenow = 1;
      break;

    case 93:  // Get page display option
      {
        // convert the decimal to a string of 0's and 1's
        char buff[10];
        memset(buff, 0, 10);
        if (display_pages == 0) {
          buff[0] = '0';
          buff[1] = 0x00;
          SendPacket('l', buff);
        } else {
          String answer = String(myfocuser.display_option, BIN);
          String reply = answer.substring(answer.indexOf('1'));
          reply.toCharArray(buff, reply.length() + 1);
          SendPacket('l', buff);
        }
      }
      break;

    //case 94:  // Set DelayedDisplayUpdate (0=disabled, 1-enabled)
      // deprecated
      // break;

    case 95:  // Get DelayedDisplayUpdate (0=disabled, 1-enabled)
      // deprecated
      SendPacket('n', 0);
      break;    
  }
}

//-----------------------------------------------------------------------
// USB SERIAL CLEAR BUFFERS
//-----------------------------------------------------------------------
void clearSerialPort() {
  while (Serial.available())
    Serial.read();
}

//-----------------------------------------------------------------------
// USB SERIAL EVENT HANDLER
// SerialEvent occurs whenever new data comes in the serial RX.
//-----------------------------------------------------------------------
void serialEvent() {
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character

  while (Serial.available()) {
    char inChar = Serial.read();
    switch (inChar) {
      case ':':  // start
        line = "";
        break;
      case EOC:  // eoc
        queue.push(line);
        break;
      default:  // anything else
        line = line + inChar;
        break;
    }
  }
}


// (IL: Needed on Pro Micro Clone otherwise serial input is never processed) -------------------------------------------------------------------------------
// On some boards serialEvent() is never called as serialEventRun is not defined
// properly by the IDE. If so this function provides the missing definition.
// Typically you will get a timeout error when you try to connect to the focuser
// using the ASCOM driver or focuser app then you need to uncomment FORCE_SERIALEVENT
// Checks to see if there is data in the Hardware Serial and if so calls serialEvent()
// to process it. This is called automatically between each loop() run and calls
// serialEvent() if there is data in the Hardware Serial that needs processing.
//-------------------------------------------------------------------------------
#if defined(CONTROLLERISAMICRO)
void serialEventRun() {
  if (Serial.available()) {
    serialEvent();
  }
}
#endif
// (IL: End of extra Pro Micro function) -------------------------------------------------------------------------------

#endif
