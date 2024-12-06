//---------------------------------------------------------------------
// myFocuserPro2 Driver Board Code
// (c) R Brown, 2014-2024, All rights reserved.
// (c) Joel Collet, move-timer, 2021-2022, All rights reserved
// (c) Paul Porters, 2020-2021. All rights reserved.
//---------------------------------------------------------------------

#include <Arduino.h>
#include "defines.h"
#include "myBoardDefs.h"
#include "myBoards.h"
#include "config.h"
#if defined(HOMEPOSITIONSWITCH)
#include <Bounce2.h>           // needed to debounce Home Position switch (HPSW)
Bounce hpswbounce = Bounce();  // Setup debouncer for HPSW
#endif
// v1 of the HPSW which has switch normally open to D12 and ground
// when HPSW is NOT activated (contacts open), D12 = 5V = HIGH
// when HPSW is activated (contacts closed), D12 = GND = LOW
// Normal operation, HPSW not activated, contacts open, D12 high, motor can move
// Fault operation, D12 (disconnected switch or power) floats high, motor can move
#include <TimerOne.h>


//-----------------------------------------------------------------------
// DEFINES
//-----------------------------------------------------------------------
// TMC22xx definitions
#define TMCRX               10            // tmc2209 interface
#define TMCTX               11
#define STALL_VALUE         40            // [0... 255]
#define TMC2209CURRENT      1000          // 600mA for 8HS15-0604S NEMA8 stepper motor
#define TMC2209PORTSPEED    57600         // define tmc2209 serial speed, DO NOT CHANGE
#define TOFF_VALUE          4             // [1... 15]
#define DRIVER_ADDRESS      0b00          // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE             0.11f         // Match to your driver
                                          // SilentStepStick series use 0.11
                                          // UltiMachine Einsy and Archim2 boards use 0.2
                                          // Panucatt BSD2660 uses 0.1
                                          // Watterott TMC5160 uses 0.075

//-----------------------------------------------------------------------
// EXTERNS
//-----------------------------------------------------------------------
extern void MotorStepClockwise(void);
extern void MotorStepAntiClockwise(void);
extern struct config_t myfocuser;
extern long fcurrentposition;   // current focuser position
extern long ftargetposition;    // target position
extern byte isMoving;           // is the motor currently moving
extern byte MainStateMachine;
extern byte movedirection;      // holds direction of new planned move
extern DriverBoard* driverboard;


//-----------------------------------------------------------------------
// LOCAL DATA
//-----------------------------------------------------------------------
volatile bool hpswstate;


//-----------------------------------------------------------------------
// void timer_stepmotor(void)
// Run by Timer1, steps motor till fcurrentposition == ftargetposition
//-----------------------------------------------------------------------
void timer_stepmotor() {
  if (fcurrentposition == ftargetposition)  // must come first else cannot halt
  {
    driverboard->stop_motortimer();
    DebugPrint("MAS013");
    MainStateMachine = State_MoveEnded;
    DebugPrint("MAS010");
  } else {
    // focuser not finished the move yet
    (movedirection == MOVINGIN) ? MotorStepAntiClockwise() : MotorStepClockwise();
    (movedirection == MOVINGIN) ? fcurrentposition-- : fcurrentposition++;

#if defined(HOMEPOSITIONSWITCH)
#ifdef USE_STALL_GUARD
    // if moving in. check if hpsw closed
    if (movedirection == MOVINGIN) {
      // is there a stall - if so then stop and set position 0
      if (hpswstate == true) {
        isMoving = 1;
        fcurrentposition = ftargetposition = 0;
        MainStateMachine = State_SetHomePosition;
        HPSWDebugPrintln("->SetHomePosition");
      }
    }
#endif

#ifdef USE_PHYSICAL_SWITCH
    // only check home position if moving in
    if (movedirection == MOVINGIN) {
      // if switch state = CLOSED and position >= 0
      // need to back OUT a little till switch opens and then set position to 0
      if ((hpswstate == true) && (fcurrentposition >= 0)) {
        driverboard->stop_motortimer();
        //isMoving = 1;
        fcurrentposition = ftargetposition = 0;
        MainStateMachine = State_SetHomePosition;
        DebugPrintln("MAS014");
        HPSWDebugPrintln("hpswstate=closed, pos >= 0");
      }
      // else if switchstate = OPEN and Position = 0
      // need to move IN a little till switch CLOSES then
      else if ((hpswstate == false) && (fcurrentposition == 0)) {
        driverboard->stop_motortimer();
        //isMoving = 1;
        fcurrentposition = ftargetposition = 0;
        MainStateMachine = State_FindHomePosition;
        DebugPrintln("MAS015");
        HPSWDebugPrintln("HPSW=Open, position=0");
        HPSWDebugPrintln("State -> State_FindHomePosition");
      }
    }   // if (movedirection == MOVINGIN)
#endif  // if USE_PHYSICAL_SWITCH
#endif  // home position switch
  }
}

//-----------------------------------------------------------------------
// DRIVERBOARD CLASS
//-----------------------------------------------------------------------
DriverBoard::DriverBoard(byte brdtype)
  : boardtype(brdtype) {
  hpswstate = false;

  pinMode(TMC2209ENABLE, OUTPUT);  // init the control pins
  pinMode(TMC2209DIR, OUTPUT);
  pinMode(TMC2209STEP, OUTPUT);
  digitalWrite(TMC2209ENABLE, LOW);  // Low enables the driver board
  init_TMC2209();
  HPSWDebugPrintln("init HPSW");
  init_hpsw();
  HPSWDebugPrintln("Read HPSW");
  hpswstate = get_hpsw();
  set_stepmode();
  _motortimerstate = false;
}


//---------------------------------------------------------------
// INITIALISE TMC2209 DRIVER CHIP
//---------------------------------------------------------------
void DriverBoard::init_TMC2209(void) {
  mystepper = new TMC2209Stepper(TMCRX, TMCTX, R_SENSE, DRIVER_ADDRESS);  // Specify the serial interface to the tmc2209
  mystepper->beginSerial(TMC2209PORTSPEED);
  mystepper->pdn_disable(1);       // Use PDN/UART pin for communication
  mystepper->mstep_reg_select(1);  // Adjust stepMode from the registers
  mystepper->I_scale_analog(0);    // Adjust current from the registers
  mystepper->toff(TOFF_VALUE);     // use TMC22xx Calculations sheet to get these and change TOFF_VALUE
  mystepper->tbl(2);
  mystepper->rms_current(TMC2209CURRENT);  // Set driver current
  int sm = myfocuser.stepmode;             // stepmode set
  sm = (sm == STEP1) ? 0 : sm;             // handle full steps
  mystepper->microsteps(sm);               // step mode = 1/4
  // stall guard settings
  mystepper->semin(0);
  // lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  mystepper->TCOOLTHRS(0xFFFFF);   // 20bit max
  mystepper->hysteresis_end(0);    // use TMC22xx Calculations sheet to get these and replace the 0
  mystepper->hysteresis_start(0);  // use TMC22xx Calculations sheet to get these and replace the 0
  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT falls below this value.
  mystepper->SGTHRS(STALL_VALUE);
  DebugPrintln(mystepper->test_connection() == 0 ? "OK" : "NOT OK");
  DebugPrint("Motor is ");
  DebugPrintln(digitalRead(TMC2209ENABLE) ? "DISABLED" : "ENABLED");
  DebugPrint("stepMode is ");
  DebugPrintln(mystepper->microsteps());
}


//-----------------------------------------------------------------------
// INITIALISE HPSW
//-----------------------------------------------------------------------
void DriverBoard::init_hpsw(void) {
#if defined(HOMEPOSITIONSWITCH)
#if defined(USE_STALL_GUARD)
  pinMode(TMC2209DIAG, INPUT_PULLUP);  // for stall guard
#elif defined(USE_PHYSICAL_SWITCH)
  pinMode(HPSWPIN, INPUT_PULLUP);  // Init Home Position Switch pin as an input
  hpswbounce.attach(HPSWPIN);      // Setup defaults for debouncing hp Switch
  hpswbounce.interval(5);          // Sets debounce time
  hpswbounce.update();
#endif
#endif
}


//-----------------------------------------------------------------------
// GET HOME POSITION SWITCH STATE
// hpsw pin has 470K pullup, hpsw open = high, hpsw closed = low
// return state of hpswstate needs to return high = sw closed, low = sw open
//-----------------------------------------------------------------------
bool DriverBoard::get_hpsw(void) {
  // avoid using debug statements because this is called for every step
#if defined(HOMEPOSITIONSWITCH)
#if defined(USE_STALL_GUARD)
  // diag goes HIGH when there is a stall, (low=no stall, high=stall)
  return !(digitalRead(TMC2209DIAG)); // invert state to match physical switch(low=closed, high=open)
#elif defined(USE_PHYSICAL_SWITCH)
  hpswbounce.update();
  return !(hpswbounce.read());  // physical switch, open = high/true, closed = low/false;
#else
  return false;  // keep Arduino compiler happy
#endif
#endif
  return false;
}


//-----------------------------------------------------------------------
// SET MOTOR POWER
// turn coil power on, turn coil power off
//-----------------------------------------------------------------------
void DriverBoard::set_motorpower(byte state) {
  if (state == true) {
    // power on
    digitalWrite(TMC2209ENABLE, LOW);
    delay(1);  // need to wait 1ms before driver chip is ready for stepping
  } else {
    // power off
    digitalWrite(TMC2209ENABLE, HIGH);
  }
}


//-----------------------------------------------------------------------
// INITIALISE MOTOR TIMER
//-----------------------------------------------------------------------
void DriverBoard::init_motortimer(void) {
  Timer1.initialize();
  Timer1.attachInterrupt(timer_stepmotor);
  set_motortimerstate(false);
}


//-----------------------------------------------------------------------
// ENABLE MOTOR TIMER, STEPPER STARTS MOVING
//-----------------------------------------------------------------------
void DriverBoard::start_motortimer(void) {
  set_motortimerstate(true);
}


//-----------------------------------------------------------------------
// STOP MOTOR TIMER, STEPPER STOPS MOVING
//-----------------------------------------------------------------------
void DriverBoard::stop_motortimer(void) {
  set_motortimerstate(false);
}


//-----------------------------------------------------------------------
// GET MOTOR TIMER STATE
//-----------------------------------------------------------------------
bool DriverBoard::get_motortimerstate(void) {
  return _motortimerstate;
}


//-----------------------------------------------------------------------
// SET STATE OF MOTOR TIMER
//-----------------------------------------------------------------------
void DriverBoard::set_motortimerstate(bool state) {
  if (state == false) {
    Timer1.setPeriod(get_motortimerinterval());
    Timer1.stop();
    _motortimerstate = false;
    DebugPrintln("BRD005");
  } else {
    Timer1.setPeriod(get_motortimerinterval());
    Timer1.start();
    _motortimerstate = true;
  }
}


//-----------------------------------------------------------------------
// MOVE STEPPER MOTOR IN THE SPECIFIED DIRECTION
//-----------------------------------------------------------------------
void DriverBoard::movemotor(byte ddir) {
  digitalWrite(TMC2209DIR, ddir);
  digitalWrite(TMC2209STEP, HIGH);  // step motor on rising edge
  delayMicroseconds(MOTORPULSETIME);
  digitalWrite(TMC2209STEP, LOW);
#if defined(HOMEPOSITIONSWITCH)
  if (ddir == MOVINGIN) {
    hpswstate = get_hpsw();
  }
#endif
}


//-----------------------------------------------------------------------
// GET THE DELAY INTERVAL FOR THE MOTOR TIMER
//-----------------------------------------------------------------------
int DriverBoard::get_motortimerinterval() {
  int sdelay = myfocuser.stepdelay;
  switch (myfocuser.motorspeed) {
    case SLOW:
      sdelay = sdelay + sdelay + sdelay;
      break;
    case MED:
      sdelay += sdelay;
      break;
    case FAST:
      //
      break;
    default:
      //
      break;
  }
  return sdelay;
}


//-----------------------------------------------------------------------
// SET STALL GUARD VALUE DERIVED FROM MOTOR SPEED
//-----------------------------------------------------------------------
void DriverBoard::set_motorspeed() {
  switch (myfocuser.motorspeed) {
    case SLOW:
      mystepper->SGTHRS(STALL_VALUE);
      break;
    case MED:
      mystepper->SGTHRS(STALL_VALUE / 2);
      break;
    case FAST:
      mystepper->SGTHRS(STALL_VALUE / 6);
      break;
    default:
      // do nothing
      break;
  }
}


//-----------------------------------------------------------------------
// GET STEP MODE
//-----------------------------------------------------------------------
int DriverBoard::get_stepmode(void) {
  DebugPrint("Driverboard::getmicrosteps() from chip: ");
  int sm = mystepper->microsteps();
  DebugPrintln(sm);
  DebugPrint("Driverboard::save setting to myfocuser");
  sm = (sm == 0) ? STEP1 : sm;
  myfocuser.stepmode = sm;
  return sm;
}


//-----------------------------------------------------------------------
// SET STEP MODE
//-----------------------------------------------------------------------
void DriverBoard::set_stepmode() {
  DebugPrint("Driverboard::set this->stepmode");
  // take the myfocuser stepmode and writ it to the TMC2209
  int sm = myfocuser.stepmode;
  sm = (sm == STEP1) ? 0 : sm;
  mystepper->microsteps(sm);
}


//-----------------------------------------------------------------------
// GET STALL GUARD SETTING
// this routine services Management call and comms call :81#
//-----------------------------------------------------------------------
byte DriverBoard::get_stallguard(void) {
  byte sgval = STALL_VALUE;  // we must set it to something in case the next lines are not enabled
  sgval = mystepper->SGTHRS();
  return sgval;
}


//-----------------------------------------------------------------------
// SET STALL GUARD SETTING
// this routine services Management call and comms call :82#
//-----------------------------------------------------------------------
void DriverBoard::set_stallguard(byte newval) {
  mystepper->SGTHRS(newval);
}
