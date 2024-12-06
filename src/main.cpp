//-----------------------------------------------------------------------
// myFOCUSERPRO2 OFFICIAL FIRMWARE RELEASE 336 (22-Jul-2024)
// BIGTREETECH TMC2209 DRIVER CHIP WITH STALL GUARD
// (c) Robert Brown 2014-2024. All Rights Reserved.
// (c) Holger, 2019-2021. All Rights Reserved.
// (c) Joel Collet, move-timer, 2021-2022, All rights reserved.
// (c) Copyright Paul Porters, 2020-2021. All rights reserved.
//-----------------------------------------------------------------------
// Supports board TMC2225_TMC2209
// Requires library TMCStepper
// The serial interface for this board runs at 57600
// home position implemented via Stall Guard, cannot be disabled

//-----------------------------------------------------------------------
// BOARDS SUPPORTED
//-----------------------------------------------------------------------
// PCB TMC2225_TMC2209
// BUZZ.LEDs.LCD.OLED.PB.HPSW.SPWR.TMP.SG
// https://sourceforge.net/projects/arduinoascomfocuserpro2diy/files/BOARDS%20PCB/PCB%20TMC2225_TMC2209/

// -----------------------------------------------------------------------
// DONATING A CUP OF COFFEE
// -----------------------------------------------------------------------
// If you wish to express your thanks for this project, please donate the amount of
// a Coffee in thanks for this project, please use PayPal.com and send the amount to
// user rbb1brown@gmail.com (Robert Brown). All contributions are gratefully accepted.

// -----------------------------------------------------------------------
// INSTRUCTIONS : SPECIFY FOCUSER CONFIGURATION
// -----------------------------------------------------------------------
// Go to file config.h and enable the hardware options you have

// -----------------------------------------------------------------------
// INSTRUCTIONS : SPECIFY ARDUINO IDE SETTINGS
// -----------------------------------------------------------------------
// Set Tools-Board as Arduino Nano
// Set Tools-Processor-ATmega328P-ATMega328P (Old Bootloader)
// Set File-Preferences
//   Show Verbose output during compile (checked), upload (checked)
//   Compiler warnings - Default
// MAKE SURE that the libaries for myFP2 firmware have been imported

//-----------------------------------------------------------------------
// FIRMWARE CODE START
// Timer1 - Moves Motor
// Timer2 - InfraRed Remote - Cannot run with Rotary Encoder
// Timer2 - Rotary Encoder - Cannot run with InfraRed Remote
//-----------------------------------------------------------------------
#include "Arduino.h"
#include <myQueue.h>
#include <myEEPROM.h>
#include <myeepromanything.h>
#include <TimerOne.h>

#include "defines.h"
#include "config.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include <myBoards.h>
DriverBoard *driverboard;
extern void timer_stepmotor(void);
extern volatile bool hpswstate;

#if defined(DISPLAYTYPE)
extern void DisplayUpdatePosition(void);
extern void DisplayUpdatePages(void);
extern void DisplayInit(void);
#include "display.h"
#endif

#if defined(TEMPERATURE_PROBE)
extern void TemperatureInit(void);
extern float read_temp(void);
extern void update_temp();
#include "temperature.h"
#endif

#include "serialcomms.h"

//-----------------------------------------------------------------------
// DEW HEATER
//-----------------------------------------------------------------------

// Définir les broches
#define ONE_WIRE_BUS A5
#define MOSFET_PIN 5
#define TEMP_SENSOR_PIN A0 // Broche analogique pour le capteur de température de la résistance

// Initialiser les objets OneWire et DallasTemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variables pour stocker les températures
float tempExterieur;
float tempResistance;
float tempCible;

// Variables pour gérer le timing
unsigned long previousMillis = 0;
const long interval = 1000; // Intervalle de 1 seconde

//-----------------------------------------------------------------------
// NEW PUSHBUTTON DATA - DO NOT CHANGE
//-----------------------------------------------------------------------
enum btnval
{
  Inbtn,
  Outbtn,
  Bothbtn,
  Nobtn
};
btnval btn;

//-----------------------------------------------------------------------
// GLOBAL DATA - DO NOT CHANGE
//-----------------------------------------------------------------------
struct config_t myfocuser;
char firmwareVersion[] = "336";
char ProgramAuthor[] = "R BROWN 2020";
char firmwareName[] = "TMC2209SG";

byte MainStateMachine;

long fcurrentposition; // current position
long ftargetposition;  // target position
byte isMoving;         // is the motor currently moving
byte writenow;         // save settings in EEPROM
byte movedirection;    // direction of new planned move
byte jogging;          // jogging in Windows application
byte joggingDirection; // defined jogging direction, 0 = IN, 1 = OUT
byte stepperpower;     // stepperpower state 0 or 1

// TEMPERATURE
byte tprobe1;           // was a temperature probe found?
float temp;             // temperature
byte tempcompstate;     // temperature compensation state, 0=off, 1=on
byte tempcompavailable; // temperature compensation available
unsigned long lasttempconversion;

// DISPLAY
byte display_pages;              // number of pages for current defined display type
bool display_found;              // true if display is found
unsigned long lastdisplayupdate; // time that display was last updated

int currentaddr;        // address in EEPROM of the data stored
Queue<String> queue(4); // queue FIFO for commands from Serial port
String line;            // received data from the Serial port

// -----------------------------------------------------------------------
// STEPPER MOTOR ROUTINES
// -----------------------------------------------------------------------
void MotorStepClockwise()
{
  (!myfocuser.reversedirection) ? driverboard->movemotor(1) : driverboard->movemotor(0);
}

void MotorStepAntiClockwise()
{
  (!myfocuser.reversedirection) ? driverboard->movemotor(0) : driverboard->movemotor(1);
}

// -----------------------------------------------------------------------
// EEPROM stuff
// -----------------------------------------------------------------------
void writeEEPROMNow()
{
  EEPROM_writeAnything(currentaddr, myfocuser); // update values in EEPROM
  writenow = 0;
}

// -----------------------------------------------------------------------
// RESET DISPLAY OPTIONS
// -----------------------------------------------------------------------
void resetdisplay_option()
{
  // display_option: determines which pages are enabled to display
  // display_pages: the total number of display pages for that display type
#if (DISPLAYTYPE == DISPLAY_LCD1602)
  myfocuser.display_option = 63; // 111111
  display_pages = 6;             // pg1-pg6
#elif (DISPLAYTYPE == DISPLAY_LCD1604)
  myfocuser.display_option = 7; // 111
  display_pages = 3;            // pg1-pg3
#elif (DISPLAYTYPE == DISPLAY_LCD2004)
  myfocuser.display_option = 7; // 111
  display_pages = 3;            // pg1-pg3
#elif (DISPLAYTYPE == DISPLAY_OLED12864)
  myfocuser.display_option = 3; // 11
  display_pages = 2;            // pg1-pg2
#elif (DISPLAYTYPE == DISPLAY_TFT)
  myfocuser.display_option = 3; // 11
  display_pages = 2;            // pg1-pg2
#elif (DISPLAYTYPE == DISPLAY_UTFT)
  myfocuser.display_option = 3; // 11
  display_pages = 2;            // pg1-pg2
#elif (DISPLAYTYPE == DISPLAY_NOKIA)
  myfocuser.display_option = 7; // 111
  display_pages = 3;            // pg1-pg3
#else
  // no display
  myfocuser.display_option = 1;
  display_pages = 0;
#endif
}

// -----------------------------------------------------------------------
// SET FOCUSER SETTINGS TO DEFAULT
// -----------------------------------------------------------------------
void setfocuserdefaults()
{
  myfocuser.validdata = VALIDDATAFLAG;
  myfocuser.fposition = DEFAULTPOSITION;
  myfocuser.maxstep = DEFAULTMAXSTEP;
  myfocuser.stepsize = DEFAULTSTEPSIZE;
  myfocuser.tempcoefficient = 20;
  myfocuser.stepmode = DEFAULTSTEPMODE;
  myfocuser.reversedirection = 0;
  myfocuser.coilpower = 0;
  myfocuser.tempmode = CELCIUS; // default is celsius
  myfocuser.display_pagetime = DEFAULT_PAGETIME;
  myfocuser.stepsizeenabled = 1;
  myfocuser.display_updateonmove - 1;
  myfocuser.tempresolution = TEMPRESOLUTION;
  myfocuser.delayaftermove = 0;
  myfocuser.backlashsteps_in = 0;
  myfocuser.backlashsteps_out = 0;
  myfocuser.focuserdirection = MOVINGIN;
  myfocuser.backlash_in_enabled = 0;
  myfocuser.backlash_out_enabled = 0;
  myfocuser.tcdirection = MOVINGIN;
  myfocuser.motorspeed = FAST;
  myfocuser.display_enabled = 1;
  myfocuser.pbsteps = 1;
  resetdisplay_option();
  myfocuser.stepdelay = MSDELAY;
  writeEEPROMNow();
  ftargetposition = fcurrentposition = myfocuser.fposition;
}

// -----------------------------------------------------------------------
// REBOOT THE ARDUINO
// -----------------------------------------------------------------------
void software_Reboot()
{
  asm volatile("jmp 0");
}

// -----------------------------------------------------------------------
// READ PUSH BUTTONS
// -----------------------------------------------------------------------
btnval readpushbuttons(void)
{
#if defined(PUSHBUTTONS)
  static int sampletable[5]; // 0, 1, 2, 3, 4
  static int pbtotal;
  static int pbval;
  digitalWrite(PBSWITCHESPIN, HIGH);
  delay(5);
  pbtotal = 0;
  for (int sample = 0; sample < 5; sample++)
  {
    delay(5);
    pbval = analogRead(A0);
    sampletable[sample] = pbval;
    pbtotal += pbval;
  }

  // 0000 - 1719 Ignore
  // 1720 - 1920 OUT
  // 1921 - 2199 Ignore
  // 2200 - 2700 BOTH
  // 2701 - 3299 Ignore
  // 3300 - 3800 IN
  // 3801+ --- Ignore

  if (pbtotal > 1720 && pbtotal < 1920)
  {
    // sw1 ON and SW2 OFF
    // Serial.println("updatepushbuttons IN:");
    return Inbtn;
  }
  else if (pbtotal > 2200 && pbtotal < 2700)
  {
    // sw1 ON and sw2 ON
    // Serial.println("updatepushbuttons ZERO:");
    return Bothbtn;
  }
  else if (pbtotal > 3300 && pbtotal < 3800)
  {
    // sw2 ON and SW1 OFF
    // Serial.println("updatepushbuttons OUT:");
    return Outbtn;
  }
#endif
  // for any other value do nothing
  return Nobtn;
}

// -----------------------------------------------------------------------
// UPDATE PUSH BUTTONS
// enum btnval btn;
// -----------------------------------------------------------------------
void updatepushbuttons(void)
{
#if defined(PUSHBUTTONS)
  btn = readpushbuttons();
  if (btn == Nobtn)
  {
    return;
  }
  delay(5);
  if (readpushbuttons() == btn)
  {
    switch (btn)
    {
    case Inbtn:
      ftargetposition = ftargetposition - myfocuser.pbsteps;
      ftargetposition = (ftargetposition < 0) ? 0 : ftargetposition;
      break;
    case Outbtn:
      ftargetposition = ftargetposition + myfocuser.pbsteps;
      ftargetposition = (ftargetposition > myfocuser.maxstep) ? myfocuser.maxstep : ftargetposition;
      break;
    case Bothbtn:
#if defined(BUZZER)
      digitalWrite(BUZZERPIN, 1); // turn on buzzer
#endif
      while (readpushbuttons() == Bothbtn) // wait for pb to be released
        ;
      fcurrentposition = 0;
      ftargetposition = 0;
      break;
    case Nobtn:
      //
      break;
    }
#if defined(DISPLAYTYPE)
    if (display_found)
    {
      DisplayUpdatePosition();
    }
#endif
  }
#endif
}

// -----------------------------------------------------------------------
// UPDATE JOGGING
// -----------------------------------------------------------------------
void updatejogging(void)
{
  if (joggingDirection == 0)
  {
    ftargetposition = ftargetposition - 1;
    ftargetposition = (ftargetposition < 0) ? 0 : ftargetposition;
    writenow = 1;
  }
  else
  {
    ftargetposition = ftargetposition + 1;
    ftargetposition = (ftargetposition > myfocuser.maxstep) ? myfocuser.maxstep : ftargetposition;
    writenow = 1;
  }
#if defined(DISPLAYTYPE)
  if (display_found)
  {
    DisplayUpdatePosition();
  }
#endif
#if defined(SUPERSLOWJOGGING)
  delay(SLOWSPEEDJOGDELAY);
#endif // SUPERSLOWJOGGING
}

// -----------------------------------------------------------------------
// UPDATE STEPPER POWER
// -----------------------------------------------------------------------
void updatestepperpowerdetect()
{
#if defined(STEPPERPWRDETECT)
  stepperpower = (analogRead(STEPPERDETECTPIN)) > 600 ? 1 : 0;
  // for ULN2003 powered from  9V with 4.7VZ, reading was 3.72V = 763
  // for DRV8825 powered from 12V with 4.7VZ, reading was 4.07V = 834
  // Each digit = .00488millivolts
#else
  stepperpower = 1;
#endif
}

// -----------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------
void setup()
{

  // -----------------------------------------------------------------------
  // DEW HEATER
  // -----------------------------------------------------------------------

  // Initialiser la communication série
   // Serial.begin(9600);

  // Initialiser les capteurs de température
  sensors.begin();

  // Initialiser la broche du MOSFET
  pinMode(MOSFET_PIN, OUTPUT);

  // -----------------------------------------------------------------------

  int datasize;   // will hold size of the struct myfocuser - 6 bytes
  int nlocations; // number of storage locations available in EEPROM
  byte found;

  line = "";

  Serial.begin(SERIALPORTSPEED); // initialize serial port
  clearSerialPort();             // clear any garbage from serial buffer

#if defined(BUZZER)
  pinMode(BUZZERPIN, OUTPUT); // turn ON the Buzzer - provide power ON beep
  digitalWrite(BUZZERPIN, 1);
#endif

#if defined(INOUTLEDS)
  DebugPrintln("LED001");
  pinMode(INLED, OUTPUT); // turn ON both LEDS as power on cycle indicator
  pinMode(OUTLED, OUTPUT);
  digitalWrite(INLED, 1);
  digitalWrite(OUTLED, 1);
#endif

  stepperpower = 1;
#if defined(STEPPERPWRDETECT)
  pinMode(STEPPERDETECTPIN, INPUT);
  DebugPrintln("STP001");
  updatestepperpowerdetect();
#endif

  isMoving = writenow = jogging = joggingDirection = tprobe1 = 0;

  hpswstate = false;

  currentaddr = 0; // start at 0 if not found later
  found = 0;
  datasize = sizeof(myfocuser);
  nlocations = EEPROMSIZE / datasize;
  for (int lp1 = 0; lp1 < nlocations; lp1++)
  {
    int addr = lp1 * datasize;
    EEPROM_readAnything(addr, myfocuser);
    if (myfocuser.validdata == VALIDDATAFLAG) // check to see if the data is valid
    {
      currentaddr = addr; // data was erased so write some default values
      found = 1;
      break;
    }
  }
  if (found == 1)
  {
    // Set the focuser back to the previous settings
    // done after this in one hit
    // mark current eeprom address as invalid and use next one
    // each time focuser starts it will read current storage, set it to invalid, goto next location and
    // write values to there and set it to valid - so it doesnt always try to use same locations over and
    // over and destroy the eeprom
    // using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
    EEPROM_readAnything(currentaddr, myfocuser);
    myfocuser.validdata = 0;
    EEPROM_writeAnything(currentaddr, myfocuser);
    currentaddr += datasize; // goto next free address and write data
    // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if (currentaddr >= (nlocations * datasize))
    {
      currentaddr = 0;
    }
    myfocuser.validdata = VALIDDATAFLAG;
    writeEEPROMNow(); // update values in EEPROM
  }
  else
  {
    DebugPrintln("CFG002");
    setfocuserdefaults(); // Set defaults because not found
  }

  // temperature probe
  DebugPrintln("TMP001");
  temp = 20.0;           // dummy temp value
  tprobe1 = 0;           // disable tprobe state
  tempcompstate = 0;     // disable temperature compensation on
  tempcompavailable = 0; // disable temperature compensation off
  lasttempconversion = millis();
#if defined(TEMPERATURE_PROBE)
  // Init will set tprobe1, tempcompstate and tempcompavailable
  TemperatureInit(); // start temperature probe
  if (tprobe1 == 1)
  {
    read_temp(); // read temperature
  }
#endif

  // check display option and reset if necessary
  if (myfocuser.display_option == 0)
  {
    resetdisplay_option();
  }

  display_found = false;
#if defined(DISPLAYTYPE)
  // If display not found, then DisplayInit()
  // will set display_found to 0
  DisplayInit();
  if (display_found)
  {
    DebugPrintln("display found");
  }
  else
  {
    DebugPrintln("display NOT found");
  }
  lastdisplayupdate = millis();
#endif

  movedirection = myfocuser.focuserdirection;

  DebugPrintln("BRD001");
  driverboard = new DriverBoard(DRVBRD);
  DebugPrintln("BRD002");

  if (myfocuser.coilpower == 0)
  {
    driverboard->set_motorpower(false);
  }

  fcurrentposition = ftargetposition = myfocuser.fposition;
  writenow = 1;

  // Set up timer
  driverboard->init_motortimer();

#if defined(INOUTLEDS)
  digitalWrite(INLED, 0);
  digitalWrite(OUTLED, 0);
#endif

#if defined(BUZZER)
  digitalWrite(BUZZERPIN, 0);
#endif

  MainStateMachine = State_Idle;

  DebugPrintln("Setup end#");
}

// -----------------------------------------------------------------------
// DEW HEATER
// -----------------------------------------------------------------------

// Constantes pour la thermistance CTN10K
const float SERIES_RESISTOR = 10000.0;    // 10KΩ résistance en série
const float ADC_MAX = 1023.0;
const float VCC = 5.0; // Tension d'alimentation du circuit

// Coefficients de Steinhart-Hart pour CTN10K
const float A = 1.009249522e-03;
const float B = 2.378405444e-04;
const float C = 2.019202697e-07;

float readAnalogTemperature(int pin)
{
  // Lire la valeur analogique
  int analogValue = analogRead(pin);

  // Calculer la tension sur la thermistance
  float voltage = analogValue * (VCC / ADC_MAX);

  // Calculer la résistance de la thermistance
  float resistance = SERIES_RESISTOR * (VCC / voltage - 1);

  // Appliquer la formule de Steinhart-Hart pour obtenir la température en Kelvin
  float steinhart;
  steinhart = log(resistance); // ln(R)
  steinhart = 1.0 / (A + B * steinhart + C * steinhart * steinhart * steinhart); // 1 / (A + B*ln(R) + C*(ln(R))^3)
  steinhart -= 273.15; // Convertir en °C
  steinhart += 2.0;   // Ajouter un offset pour matcher l'autre sonde

  return steinhart; // Retourner la température en °C
}


// Main Loop
void loop()
{
  // -----------------------------------------------------------------------
  // DEW HEATER
  // -----------------------------------------------------------------------

  // Obtenir le temps actuel
  unsigned long currentMillis = millis();

  // Vérifier si l'intervalle est écoulé
  if (currentMillis - previousMillis >= interval)
  {
    // Sauvegarder le temps actuel
    previousMillis = currentMillis;

    // Demander les températures
    sensors.requestTemperatures();

    // Lire les températures
    tempExterieur = sensors.getTempCByIndex(0);
    if (tempExterieur == DEVICE_DISCONNECTED_C)
    {
      //Serial.println("Erreur : Capteur non connecté ou défectueux.");
    }
    // else
    // {
    //   Serial.print("Température extérieure: ");
    //   Serial.println(tempExterieur);
    // }

    tempResistance = readAnalogTemperature(TEMP_SENSOR_PIN); // Sonde de la résistance

    // Calculer la température cible
    tempCible = tempExterieur + 4;

    // Réguler le PWM
    if (tempResistance <= tempCible)
    {
      analogWrite(MOSFET_PIN, 255); // Allumer la résistance
      // Serial.println("Allumer la résistance");
    }
    else
    {
      analogWrite(MOSFET_PIN, 0); // Éteindre la résistance
      // Serial.println("Éteindre la résistance");
    }

    // Afficher les températures pour le débogage
    // Serial.print("Température extérieure: ");
    // Serial.print(tempExterieur);
    // Serial.print(" °C, Température résistance: ");
    // Serial.print(tempResistance);
    // Serial.print(" °C, Température cible: ");
    // Serial.println(tempCible);
  }
  // -----------------------------------------------------------------------

  static unsigned long timestampdelayaftermove = millis();
  static unsigned long previousMillis = millis();
  // static unsigned long DisplayUpdate = millis();
  // static unsigned long TempUpdate = millis();
  static int backlash_count = 0;
  static byte backlash_enabled = 0;
  // static byte updatecount = 0;

#if defined(HOMEPOSITIONSWITCH)
#if defined(USE_PHYSICAL_SWITCH)
  static int stepstaken = 0;
#endif
#endif

  DebugPrintln("STP001");
  updatestepperpowerdetect();

  if (queue.count() >= 1)
  {
    ser_comms();
  }

  switch (MainStateMachine)
  {
  case State_Idle:
    if (fcurrentposition != ftargetposition)
    {
      isMoving = 1; // due to timing issue with TFT must be set here
      driverboard->set_motorpower(true);
      driverboard->set_motortimerstate(false);
      MainStateMachine = State_InitMove;
    }
    else
    {
      isMoving = 0;
      // update temperature
#if defined(TEMPERATURE_PROBE)
      if (tprobe1 == 1)
      {
        TempUpdate = millis();
        if (((TempUpdate - lasttempconversion) > TEMP_REFRESHRATE) || (TempUpdate < lasttempconversion))
        {
          update_temp();
          lasttempconversion = TempUpdate; // update time stamp
        }
      }
#endif
#if defined(DISPLAYTYPE)
      if (display_found)
      {
        unsigned long DisplayUpdate = millis();
        if ((DisplayUpdate - lastdisplayupdate) > ((unsigned long)(myfocuser.display_pagetime * 1000)) || (DisplayUpdate < lastdisplayupdate))
        {
          // update timestamp
          lastdisplayupdate = DisplayUpdate;
          DebugPrintln("DIS002");
          DisplayUpdatePages();
        }
      } // if (display_found)
#endif
      if (stepperpower == 1)
      {
        updatepushbuttons();

        if (jogging == 1)
        {
          updatejogging();
        }
      }

      // is it time to update EEPROM settings?
      if (writenow == 1)
      {
        // decide if we have waited 10s after the last move, if so, update the EEPROM
        static unsigned long currentMillis;
        currentMillis = millis();
        if (((currentMillis - previousMillis) > EEPROMWRITEINTERVAL) || (currentMillis < previousMillis))
        {
          // Serial.println("Writing to eeprom");
          previousMillis = currentMillis;
          // copy current settings and write the data to EEPROM
          myfocuser.validdata = 99;
          myfocuser.fposition = fcurrentposition;
          // update values in EEPROM
          EEPROM_writeAnything(currentaddr, myfocuser);
          writenow = 0;
        }
      }

    } // if (fcurrentposition != ftargetposition)
    break;

  case State_InitMove:
    isMoving = 1;
    if (ftargetposition < fcurrentposition)
    {
      movedirection = MOVINGIN;
      backlash_count = myfocuser.backlashsteps_in;
      backlash_enabled = myfocuser.backlash_in_enabled;
    }
    else
    {
      movedirection = MOVINGOUT;
      backlash_count = myfocuser.backlashsteps_out;
      backlash_enabled = myfocuser.backlash_out_enabled;
    }
    // enable leds
#if defined(INOUTLEDS)
    if (movedirection == MOVINGIN)
    {
      (!myfocuser.reversedirection) ? digitalWrite(OUTLED, 1) : digitalWrite(INLED, 1);
    }
    else // moving out
    {
      (!myfocuser.reversedirection) ? digitalWrite(INLED, 1) : digitalWrite(OUTLED, 1);
    }
#endif
    if (movedirection != myfocuser.focuserdirection)
    {
      if (backlash_enabled == 1)
      {
        // apply backlash
        myfocuser.focuserdirection = movedirection;
        MainStateMachine = State_ApplyBacklash;
        DebugPrintln("MAS001");
      }
      else
      {
        // do not apply backlash, go straight to moving
        MainStateMachine = State_Moving;
        DebugPrintln("MAS002");
      }
    }
    else
    {
      MainStateMachine = State_Moving;
      DebugPrintln("MAS002");
    }
    break;

  case State_ApplyBacklash:
    if (backlash_count)
    {
      (movedirection == MOVINGIN) ? MotorStepAntiClockwise() : MotorStepClockwise();
      delayMicroseconds(myfocuser.stepdelay);
      backlash_count--;
    }
    else
    {
      MainStateMachine = State_Moving;
      DebugPrintln("MAS002");
    }
    break;

  case State_Moving:
    if (driverboard->get_motortimerstate() == false)
    {
      // enable motor timer, start moving
      driverboard->set_motorpower(true);
      driverboard->start_motortimer();
    }
#if defined(DISPLAYTYPE)
    if (display_found)
    {
      if (myfocuser.display_updateonmove == 1)
      {
        updatecount++;
        if (updatecount > DISPLAY_STEPCOUNT)
        {
          updatecount = 0;
          DisplayUpdatePosition();
        }
      } // if (myfocuser.display_updateonmove == 1)
    }
#endif
    break;

  // for handling a physical switch
  case State_FindHomePosition: // move in till home position switch closes
#if defined(HOMEPOSITIONSWITCH)
#ifdef USE_PHYSICAL_SWITCH
    stepstaken = 0;
    HPSWDebugPrintln("State_FindHomePosition: MoveIN till closed");
    static bool swstate = driverboard->get_hpsw();
    while (swstate == false)
    {
      // step IN till switch closes
      MotorStepAntiClockwise();
      delayMicroseconds(myfocuser.stepdelay);
      stepstaken++;
      HPSWDebugPrintln(".");
      // this prevents the endless loop if the hpsw is not connected or is faulty
      if (stepstaken > HOMESTEPS)
      {
        HPSWDebugPrintln("HPSW MoveIN ERROR: HOMESTEPS exceeded");
        break;
      }
      swstate = driverboard->get_hpsw();
    }
    HPSWDebugPrintln();
    HPSWDebugPrint("HPSW state=");
    HPSWDebugPrint(driverboard->get_hpsw());
    HPSWDebugPrint("HP MoveIN stepstaken=");
    HPSWDebugPrintln(stepstaken);
    HPSWDebugPrintln("HP MoveIN finished");
#endif // USE_PHYSICAL_SWITCH
#endif // HOMEPOSITIONSWITCH
    MainStateMachine = State_SetHomePosition;
    DebugPrintln("MAS007");
    break;

  // for handling a physical switch
  case State_SetHomePosition: // move out till home position switch opens
#if defined(HOMEPOSITIONSWITCH)
#ifdef USE_PHYSICAL_SWITCH
    stepstaken = 0;
    HPSWDebugPrintln("State_SetHomePosition Move out till HPSW is OPEN");
    // if the previous moveIN failed at HOMESTEPS and HPSWITCH is still open then the
    // following while() code will drop through and have no effect and position = 0
    myfocuser.focuserdirection = !movedirection;
    while (driverboard->get_hpsw() == true)
    {
      // step out till switch opens
      MotorStepClockwise();
      delayMicroseconds(myfocuser.stepdelay);
      stepstaken++;
      HPSWDebugPrintln(".");
      if (stepstaken > HOMESTEPS) // this prevents the endless loop if the hpsw is not connected or is faulty
      {
        HPSWDebugPrintln("HP MoveOUT ERROR: HOMESTEPS exceeded");
        break;
      }
    }
    HPSWDebugPrintln();
    HPSWDebugPrint("HP MoveOUT stepstaken=");
    HPSWDebugPrintln(stepstaken);
    HPSWDebugPrintln("HP MoveOUT finished");
    HPSWDebugPrintln("State -> State_MoveEnded");
#endif // USE_PHYSICAL_SWITCH
#endif // HOMEPOSITIONSWITCH
    timestampdelayaftermove = millis();
    MainStateMachine = State_MoveEnded;
    DebugPrintln("MAS012");
    break;

  case State_MoveEnded:
    timestampdelayaftermove = millis();
    MainStateMachine = State_DelayAfterMove;
    break;

  case State_DelayAfterMove:
  {
    static unsigned long timenow;
    timenow = millis();
    if (((timenow - timestampdelayaftermove) > myfocuser.delayaftermove) || (timenow < timestampdelayaftermove))
    {
      timestampdelayaftermove = timenow;
      MainStateMachine = State_FinishedMove;
      DebugPrintln("MAS013");
    }
  }
  break;

  case State_FinishedMove:
    // turn off leds
#if defined(INOUTLEDS)
    digitalWrite(INLED, 0);
    digitalWrite(OUTLED, 0);
#endif
    if (myfocuser.coilpower == 0)
    {
      DebugPrintln("PAR000");
      driverboard->set_motorpower(false);
    }
    isMoving = 0;
    writenow = 1;
    previousMillis = millis();
    MainStateMachine = State_Idle;
    DebugPrintln("MAS014");
    break;

  default:
    MainStateMachine = State_Idle;
    break;
  }
} // end Loop()
