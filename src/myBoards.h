//-----------------------------------------------------------------------
// myFocuserPro2 Driver Board Definitions
// (c) R Brown, 2014-2024, All rights reserved.
// (c) Copyright Holger, 2019-2021. All Rights Reserved.
// (c) Joel Collet, move-timer, 2021-2022, All rights reserved.
// (c) Paul Porters, 2020-2021. All rights reserved.
//-----------------------------------------------------------------------
#ifndef myBoards_h
#define myBoards_h

//-----------------------------------------------------------------------
// includes
//-----------------------------------------------------------------------
#include <Arduino.h>
#include <TMCStepper.h>

//-----------------------------------------------------------------------
// wiring
//-----------------------------------------------------------------------
// Make sure pins RX and TX DO NOT plug into the DRV8825 PCB Socket

//-----------------------------------------------------------------------
// defines
//-----------------------------------------------------------------------
#if !defined(SLOW)
#define SLOW 0  // motorspeeds
#endif
#if !defined(MED)
#define MED 1
#endif
#if !defined(FAST)
#define FAST 2
#endif

#if !defined(STEP1)
#define STEP1 1  // stepmodes
#endif
#if !defined(STEP2)
#define STEP2 2
#endif
#if !defined(STEP4)
#define STEP4 4
#endif
#if !defined(STEP8)
#define STEP8 8
#endif
#if !defined(STEP16)
#define STEP16 16
#endif
#if !defined(STEP32)
#define STEP32 32
#endif
#if !defined(STEP64)
#define STEP64 64
#endif
#if !defined(STEP128)
#define STEP128 128
#endif
#if !defined(STEP256)
#define STEP256 256
#endif
#if !defined(MAXSTEPMODE)
#define MAXSTEPMODE 256
#endif

#define TMC2209DIR 3  // TMC2209 control lines
#define TMC2209STEP 4
#define TMC2209ENABLE 8
#define TMC2209DIAG 12  // for stall guard
#define TMCMS1 7
#define TMCMS2 6
#define MOTORPULSETIME 3  // TMC2209 driver

#define MSDELAY 6000

class DriverBoard {
public:
  DriverBoard(byte);
  void init_TMC2209(void);
  void init_hpsw(void);
  bool get_hpsw(void);
  
  void set_motorpower(byte);

  void init_motortimer(void);
  void start_motortimer(void);
  void stop_motortimer(void);
  bool get_motortimerstate(void);
  void set_motortimerstate(bool);
  // move motor one step, takes parameter direction 0 | 1
  void movemotor(byte);

  int get_motortimerinterval(void);
  void set_motorspeed(void);

  int get_stepmode(void);
  void set_stepmode(void);

  byte get_stallguard(void);
  void set_stallguard(byte);

private:
  TMC2209Stepper* mystepper;
  byte boardtype;  // TMC2209
  bool _motortimerstate;
};

#endif
