#pragma once

#include <pthread.h>
#include <iostream>
#include <mutex>
#include <thread>
#include "freemodbus_tcp.h"
#include "maxon.hpp"

using namespace std;

// motion parameter structure
struct motion_para_type {
  //   upwheel move distance
  __s32 upwheel_up_dis;
  __s32 upwheel_down_dis;

  //   slave move speed
  __s16 move_up_speed;
  __s16 move_down_speed;

  // master move speed
  __s16 master_move_up_speed;
  __s16 master_move_down_speed;

  // correct parameter, down direction - correct
  __s32 upwheel_move_down_dis_correct;

  // correct parameter of pulleys' distance when slave moves down.
  __s32 pulley1_move_down_dis_correct;
  __s32 pulley2_move_down_dis_correct;

  // distance for PPM control, before which change to PPM
  __s32 slave_up_dis_for_ppm;
  __s32 slave_down_dis_for_ppm;

  // master move distance for PPM control, before which change to PPM
  __s32 master_up_dis_for_ppm;
  __s32 master_down_dis_for_ppm;

  // slave motion speed factor, pulley : upwheel
  double up_speed_factor;
  double down_speed_factor;

  // slave motion distance factor, pulley : upwheel
  double pulley1_dis_factor;
  double pulley2_dis_factor;

  // master motion distance factor, pulley : upwheel
  double pulley1_master_down_dis_factor;
  double pulley2_master_down_dis_factor;
};

// robot structure
struct robot_type {
  // system state
  __u16 system_state;

  // system operation mode select
  __u16 mode_select;

  // debug mode select
  __u16 debug_mode_select;

  // debug enable?
  __u16 debug_en;

  // normal_mode enable
  __u16 normal_mode_en;

  // direction
  __u16 dir;

  // auto cycle en
  __s16 auto_cycle_en;

  // cycle;
  __s16 cycle;

  // pulleys homing enable
  __s16 homing_en;

  // load_test_up_en
  __s16 load_test_up_en;

  // pulley 1 torque
  __s16 upwheel_pos;
  __s16 pulley2_torque;

  // pulleys homing done
  __u16 homing_done;

  // down claw 1 debug;
  __s16 down_claw_debug_loose;
  // up_claw_debug done
  __u16 up_claw_hold_done;

  /* ------------------move up motion debug-------------------------- */
  __u16 down_claw_loose_en;

  // motion parameter
  motion_para_type motion_para;
};

class robot : public maxon {
 private:
  robot_type *robot_;
  mutex mut;
  /* -------------------------robot modes------------------------------- */
  // idle mode
  static const __u16 kIdleMode = 0;
  // debug mode
  static const __u16 kDebugMode = 1;
  // nalmal motion mode
  static const __u16 kNomalMode = 2;

  /* ------------------------debug ------------------------------------ */
  // claw motor debug
  static const __u16 kUpClawMotorDebug = 1;

  // upwheel motor debug
  static const __u16 kUpWheelMotorDebug = 2;

  // claw hold debug
  static const __u16 kUpClawHoldDebug = 3;

  // pulley motion debug
  static const __u16 kPulleysMotionDebug = 4;

  // pulley homing debug
  static const __u16 kPulleysHomingDebug = 5;

  // down claw hold motion
  static const __u16 kDownClawHoldDebug = 6;

  // homing motion debug
  static const __u16 kHomingDebug = 7;

  // master move up motion debug
  static const __u16 kMasterMoveUp = 8;

  // master move down motion debug
  static const __u16 kMasterMoveDown = 9;

  //   slave move down motion debug
  static const __u16 kSlaveMoveDown = 10;

  //   slave move up motion debug
  static const __u16 kSlaveMoveUp = 11;

  // upclaw hold
  static const __u16 kUpClawHold = 12;

  // upclaw loose
  static const __u16 kUpClawLoose = 13;

  // downclaw hold
  static const __u16 kDownClawHold = 14;

  // downclaw loose
  static const __u16 kDownClawLoose = 15;

  // move up
  static const __u16 kMoveUp = 16;

  // move down
  static const __u16 kMoveDown = 17;

  // motion cycle, init up
  static const __u16 kMotionCycleInitUp = 18;

  // quit debug
  static const __u16 kQuitDebug = 19;

  // master move up debug
  static const __u16 kMasterMoveUpDebug = 25;
  static const __u16 kMasterMoveDownDebug = 26;

  // test
  static const __u16 kTest = 21;

  // slave up and down
  static const __u16 kSlaveUpDown = 22;

  /* debug state machine */

  /* -------------------------robot motions------------------------------ */

  // homing states
  static const __u8 kHomingIdle = 0;
  static const __u8 kHoming = 1;
  static const __u8 kHomingDone = 2;

  // robot parameters
  // tighten torque 12%
  static const __s16 kPulleysTightenTorque = 120;

  // pull torque 50%
  static const __s16 kPulleysPullTorque = 500;

  // loose torque 10%
  static const __s16 kPulleysLooseTorque = 100;

  // pulleys move distance
  static const __s32 kPulleysMoveUpDistance = 500000;
  static const __s32 kPulleysMoveDownDistance = -500000;

  // /* upwheel motion debug */
  // //   upwheel move distance
  // static const __s32 kUpwheelMoveUpDistance = 450000;
  // static const __s32 kUpwheelMoveDownDistance = -450000;

  // //   slave move speed
  // static const __s16 kMoveUpSpeed = 8000;
  // static const __s16 kMoveDownSpeed = -6500;

  // // master move speed
  // static const __s16 kMasterMoveUpSpeed = 8000;
  // static const __s16 kMasterDownUpSpeed = -6500;

  // // correct parameter, down direction - correct
  // static const __s32 kUpwheelMoveDownDistanceCorrect = 65000;

  // // correct parameter of pulleys' distance when slave moves down.
  // static const __s32 kPulley1MoveDownDistanceCorrect = 10000;
  // static const __s32 kPulley2MoveDownDistanceCorrect = 10000;

  // // slave move distance for PPM control, before which change to PPM
  // static const __s32 kSlaveUpDisForPPM = 110000;
  // static const __s32 kSlaveDownDisForPPM = 100000;

  // // master move distance for PPM control, before which change to PPM
  // static const __s32 kMasterUpDisForPPM = 110000;
  // static const __s32 kMasterDownDisForPPM = 100000;

  // // slave motion speed factor, pulley : upwheel
  // const double kSpeedFactor = 1.2;
  // const double kDownSpeedFactor = 1.45;

  // // slave motion distance factor, pulley : upwheel
  // const double kDisFactor1 = 0.545;
  // const double kDisFactor2 = 0.565;

  // // master motion distance factor, pulley : upwheel
  // const double kMasterMoveDownDisFactor1 = 0.5276;
  // const double kMasterMoveDownDisFactor2 = 0.5209;

  /* upwheel move 450000 parameters */
  const motion_para_type kParaShortStep = {
      -450000,  // upwheel move up distance
      450000,  // upwheel move down distance

      -6300,  // slave move up speed
      5000,  // slave move down speed

      
      5000,// master move up speed
      -2000,// master move down speed


      // correct parameter, down direction - correct
      100000,

      // correct parameter of pulleys' distance when slave moves down.
      // pulley1
      80000,
      // pulley2
      120000,

      // slave move distance for PPM control, before which change to PPM
      // up
      110000,
      // down
      100000,

      // master move distance for PPM control, before which change to PPM
      // pulley1
      5000,
      // pulley2
      5000,
      // slave motion speed factor, pulley : upwheel
      // up
      1.2,
      // down
      1.7,
      // slave motion distance factor, pulley : upwheel
      // pulley1:upwheel
      // 0.53,
      0.55,
      // pulley2:upwheel
      // 0.55,
      0.55,
      // master motion distance factor, pulley : upwheel
      // pulley1:upwheel
      0.5276,
      // pulley2:upwheel
      0.5209};

  // upwheel move 900000 parameters
  const motion_para_type kParaLongStep = {
      -1000000,  // upwheel move up distance
      890000,    // upwheel move down distance

      -8400,  // slave move up speed
      3000,   // slave move down speed

      5000,   // master move up speed
      -2000,  // master move down speed

      // correct parameter, down direction - correct
      120000,

      // correct parameter of pulleys' distance when slave moves down.
      50000,  // pulley1
      50000,  // pulley2

      // slave move distance for PPM control, before which change to PPM
      140000,  // up
      20000,   // down

      // master move distance for PPM control, before which change to PPM
      20000,  // pulley1
      20000,  // pulley2

      // slave motion speed factor, pulley : upwheel
      1.3,  // up
      1.2,  // down

      // slave motion distance factor, pulley : upwheel
      0.55,  // pulley1:upwheel
      0.53,   // pulley2:upwheel

      // master motion distance factor, pulley : upwheel
      0.5276,  // pulley1:upwheel
      0.5209   // pulley2:upwheel
  };

  /* upwheel move 450000 parameters */
  const motion_para_type kParaShortStepLoad = {
      -450000,  // upwheel move up distance
      450000,  // upwheel move down distance

      -4100,  // slave move up speed
      2900,  // slave move down speed

      
      5000,// master move up speed
      -2000,// master move down speed


      // correct parameter, down direction - correct
      100000,

      // correct parameter of pulleys' distance when slave moves down.
      // pulley1
      80000,
      // pulley2
      100000,

      // slave move distance for PPM control, before which change to PPM
      // up
      110000,
      // down
      100000,

      // master move distance for PPM control, before which change to PPM
      // pulley1
      5000,
      // pulley2
      5000,
      // slave motion speed factor, pulley : upwheel
      // up
      1.2,
      // down
      1.7,
      // slave motion distance factor, pulley : upwheel
      // pulley1:upwheel
      // 0.53,
      0.55,
      // pulley2:upwheel
      // 0.55,
      0.55,
      // master motion distance factor, pulley : upwheel
      // pulley1:upwheel
      0.5276,
      // pulley2:upwheel
      0.5209};

  /*debug parameters */
  // claw relative pos 100 inc
  static const __u32 kUpClawDebugRelaPos = 100;

  // upwheel relative pos 1000inc
  static const __u32 kUpWheelDebugRelaPos = 1000;
  // pulleys relative pos 1000inc
  static const __u32 kPulleysDebugRelaPos = 100;

 public:
  robot(USHORT reg[]);
  ~robot();

  /* -------------------------system------------------------------ */
  void system(void);

  /* -------------------------robot control------------------------------ */
  // up claw
  void UpClawHold();
  void UpClawLoose();

  // master move up
  void MasterMoveUp();
  void MasterMoveUpDebug();
  void MasterMoveDownDebug();
  void DownClawHold();
  void DownClawLoose();
  void PulleysTorque(__s16 torque);
  void PulleysMoveUp();
  void PulleysMoveUp(__s32 speed);
  void PulleysMoveUpDebug();
  void PulleysMoveDownDebug();
  void Pulley1MasterSpeedUp();
  void Pulley2MasterSpeedUp();

  // master move down
  void MasterMoveDown();
  void PulleysMoveDown();
  void PulleysMoveDown(__s32 speed);
  void Pulley1MasterSpeedDown();
  void Pulley2MasterSpeedDown();

  //   salve move down
  void SlaveMoveDown();
  void UpWheelMoveDown();
  void UpWheelSpeedDown();
  void Pulley1SpeedDown();
  void Pulley2SpeedDown();

  //   salve move up
  void SlaveMoveUp();
  void UpWheelMoveUp();
  void UpWheelMoveUp(__s32 pos);
  void UpWheelSpeedUp();
  void Pulley1SpeedUp();
  void Pulley2SpeedUp();

  // robot move up
  void MoveUp();
  void MoveDown();

  /* -------------------------debug function------------------------------ */
  void UpClawDebug(void);
  void UpClawHoldDebug(void);
  void UpWheelDebug(void);
  void PulleysDebug(void);
  void PulleysHomingDebug(void);
  void SlaveUpDown();

  // up claw hold debug
  void DownClawHoldDebug(void);

  // homing
  void Homing(void);
};
