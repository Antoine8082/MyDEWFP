//-----------------------------------------------------------------------
// myFocuserPro2 Temperature Routines
// (c) R Brown, 2014-2024, All rights reserved.
// (c) Holger Manz, 2019-2021, All rights reserved.
//-----------------------------------------------------------------------

#ifndef temperature_h
#define temperature_h


//-----------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"

#if defined(TEMPERATURE_PROBE)

#include <OneWire.h> 
#include <myDallasTemperature.h> 


//-----------------------------------------------------------------------
// EXTERNS
//-----------------------------------------------------------------------
extern byte tprobe1;  // indicates if a probe was found
extern float temp;
extern byte tempcompstate;
extern byte tempcompavailable;
extern long ftargetposition;


//-----------------------------------------------------------------------
// LOCAL DATA
//-----------------------------------------------------------------------
OneWire oneWirech1(TEMPPIN);  // assign temperature probe pin to use
DallasTemperature sensor1(&oneWirech1);
DeviceAddress tpAddress;  // holds address of the temperature probe


//-----------------------------------------------------------------------
// TEMPERATURE PROBE ROUTINES
// Initialise temperature probe
//-----------------------------------------------------------------------
void TemperatureInit(void) {
  pinMode(TEMPPIN, INPUT);   // Configure GPIO pin for temperature probe
  sensor1.begin();           // start the temperature sensor1
  sensor1.getDeviceCount();  // should return 1 if probe connected
  if (sensor1.getAddress(tpAddress, 0) == true) {
    // Serial.println("tprobe found");
    tprobe1 = 1;
    tempcompavailable = 1;
    // Set probe resolution
    sensor1.setResolution(tpAddress, TEMPRESOLUTION);  // Set probe resolution
    sensor1.requestTemperatures();
  } else {
    tprobe1 = 0;
    tempcompstate = 0;
    tempcompavailable = 0;
    // Serial.println("tprobe notfound");
  }
}

//-----------------------------------------------------------------------
// READ TEMPERATURE
//-----------------------------------------------------------------------
float read_temp() {
  float result = temp;
  if (tprobe1 )
  // Get channel 1 temperature, always in celsius
  result = sensor1.getTempCByIndex(0);
  // check if reading is valid
  if ((result > -40.0) && (result < 80.0)) {
    temp = result;
  } else {
    result = temp;
  }
  return result;
}

//-----------------------------------------------------------------------
// UPDATE TEMPERATURE READINGS AND HANDLE TEMPERATURE COMPENSATION
//-----------------------------------------------------------------------
void update_temp(void) {
  static byte tcchanged = tempcompstate;

  if (tprobe1 == 1) {
    static byte requesttempflag = 0;  // start with request

    static float tempval;
    static float starttemp;  // start temperature to use when temperature compensation is enabled

    if (tcchanged != tempcompstate) {
      tcchanged = tempcompstate;
      if (tcchanged == 1) {
        starttemp = read_temp();
      }
    }

    // split between request and then read
    // this avoids waiting 750ms between request and read
    if (requesttempflag) {
      tempval = read_temp();
    } else {
      sensor1.requestTemperatures();
    }

    requesttempflag ^= 1;  // toggle flag

    // check for temperature compensation
    if (tempcompstate == 1) 
    {
        // calculate if temp has moved by more than 1 degree
        if ((abs)(starttemp - tempval) >= 1) {
          long newPos;
          byte temperaturedirection;  // did temperature fall (1) or rise (0)?
          temperaturedirection = (tempval < starttemp) ? 1 : 0;
          // check if tc direction for compensation is inwards
          if (myfocuser.tcdirection == 0) {
            // temperature compensation direction is in, if a fall then move in else move out
            
            // check if temperature is falling
            if (temperaturedirection == 1) {
              // then move inwards
              newPos = ftargetposition - myfocuser.tempcoefficient;
            } else {
              // else move outwards
              newPos = ftargetposition + myfocuser.tempcoefficient;
            }
          } else {
            // temperature compensation direction is out, if a fall then move out else move in
            if (temperaturedirection == 1) {
              newPos = ftargetposition + myfocuser.tempcoefficient;
            } else {
              newPos = ftargetposition - myfocuser.tempcoefficient;
            }
          }
          newPos = (newPos > myfocuser.maxstep) ? myfocuser.maxstep : newPos;
          newPos = (newPos < 0) ? 0 : newPos;
          ftargetposition = newPos;
          starttemp = tempval;  // save this current temp point for future reference
        } // end of check for tempchange >= 1
    } // end of check for tempcomp enabled
  } // end of if tprobe
}

#endif
#endif