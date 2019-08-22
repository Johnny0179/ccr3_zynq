#include "robot.hpp"

robot::robot(USHORT reg[]) : maxon() {
  // define the pointer address
  upclaw_ = (maxon_type *)&reg[100];
  upwheel_ = (maxon_type *)&reg[150];
  downclaw1_ = (maxon_type *)&reg[200];
  downclaw2_ = (maxon_type *)&reg[350];
  pulley1_ = (maxon_type *)&reg[250];
  pulley2_ = (maxon_type *)&reg[300];

  robot_ = (robot_type *)&reg[0];

  // defualt debug mode
  robot_->mode_select = 1;

  // defualt debug mode select
  robot_->debug_mode_select = 11;
}

robot::~robot() {}

/* -------------------------------robot
 * control---------------------------------- */
void robot::system(void) {
  switch (robot_->system_state) {
      // robot operate mode select
    case kIdleMode:
      if (robot_->mode_select == kIdleMode || robot_->debug_mode_select == 0) {
        robot_->system_state = kIdleMode;
        // clear variables
        robot_->upclaw_debug_factor = 0;
        robot_->upwheel_debug_factor = 0;
        robot_->pulleys_distance_factor = 0;
      } else if (robot_->mode_select == kDebugMode) {
        printf("Enter debug mode!\n");
        // start all nodes
        NMTstart(0);

        robot_->system_state = kDebugMode;
      } else {
        robot_->system_state = kNomalMode;
      }

      break;

      // debug mode
    case kDebugMode:

      if (robot_->debug_en == 1) {
        // start NMT
        NMTstart(0);
        // wait epos
        delay_us(kDelayEpos);

        // choose debug mode
        switch (robot_->debug_mode_select) {
            // claw motor debug
          case kUpClawMotorDebug:
            printf("Up Claw debug!\n");
            UpClawDebug();
            break;

            // upwheel motor debug
          case kUpWheelMotorDebug:
            printf("Up wheel debug!\n");
            UpWheelDebug();
            break;

          case kUpClawHoldDebug:
            printf("Up Claw hold debug!\n");
            UpClawHoldDebug();
            break;

          case kPulleysMotionDebug:
            printf("Pulley motion debug!\n");
            PulleysDebug();
            break;

            // pulleys homing
          case kPulleysHomingDebug:
            printf("Pulley homing debug!\n");
            PulleysHomingDebug();
            break;

          case kDownClawHoldDebug:
            printf("Down claw debug!\n");
            DownClawHoldDebug();
            break;

          case kHomingDebug:
            printf("Homing debug!\n");
            Homing();
            break;

          case kMasterMoveUp:
            printf("Master Move up debug!\n");
            MasterMoveUp();
            break;

          case kMasterMoveDown:
            printf("Master Move Down debug!\n");
            MasterMoveDown();
            break;

          case kSlaveMoveDown:
            SlaveMoveDown();
            printf("Slave Move Down debug!\n");
            break;

          case kSlaveMoveUp:
            SlaveMoveUp();
            printf("Slave Move Up debug!\n");
            break;

          case kUpClawHold:
            printf("up claw hold!\n");
            MotorEnable(upclaw_);
            UpClawHold();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kUpClawLoose:
            printf("up claw loose!\n");
            MotorEnable(upclaw_);
            UpClawLoose();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kDownClawHold:
            printf("down claw hold!\n");
            MotorEnable(downclaw1_);
            DownClawHold();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kDownClawLoose:
            printf("down claw loose!\n");
            MotorEnable(downclaw1_);
            DownClawLoose();
            // disable the debug
            robot_->debug_en = 0;
            break;

          default:
            break;
        }
      }

      if (robot_->debug_mode_select != 0) {
        robot_->system_state = kDebugMode;
      } else {
        // change to idle
        robot_->system_state = kIdleMode;

        // stop NMT
        NMTstop(0);
      }

      break;

    default:
      break;
  }
}

/* -------------------------debug function------------------------------ */
void robot::UpClawDebug(void) {
  // enable claw motor
  MotorEnable(kUpClaw);

  MoveRelative(kUpClaw, robot_->upclaw_debug_factor * kUpClawDebugRelaPos);

  // disable the debug
  robot_->debug_en = 0;
}

// UpWheel Debug
void robot::UpWheelDebug(void) {
  MoveRelative(kUpWheel, robot_->upwheel_debug_factor * kUpWheelDebugRelaPos);

  // disable the debug
  robot_->debug_en = 0;
}

// Pulleys Debug
void robot::PulleysDebug(void) {
  // move to destination
  MoveRelative(kPulley1, kPulley2,
               robot_->pulleys_distance_factor * kPulleysDebugRelaPos);

  // disable the debug
  robot_->debug_en = 0;
}

void robot::PulleysHomingDebug(void) {
  // enable pulleys
  MotorEnable(kPulley1);
  MotorEnable(kPulley2);

  // set pulleys torque
  PulleysTorque(kPulleysHomingTorque);

  // wait for homing done
  while (robot_->homing_done != 1) {
    SetTargetTorque(kPulley1, kPulleysHomingTorque);
    SetTargetTorque(kPulley2, kPulleysHomingTorque);
    printf("pulleys homing torq: pulley1-> %d pulley2->%d\n", pulley1_->TrqPV,
           pulley2_->TrqPV);
  }

  // disable the debug
  robot_->debug_en = 0;

  // clear homing done flag
  robot_->homing_done = 0;
}

// homing
void robot::Homing(void) {
  // enable motors
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);
  MotorEnable(upclaw_);
  MotorEnable(upwheel_);
  MotorEnable(downclaw1_);

  // hold claws
  UpClawHold();

  DownClawHold();

  // pulleys tighten
  // PulleysTorque(kPulleysTightenTorque);

  // disable pulleys
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);

  // up claw loose
  UpClawLoose();

  // down claw loose
  // DownClawLoose();

  // set the homing torque
  // PulleysTorque(kPulleysHomingTorque);

  // enable homming
  while (robot_->homing_done != 1) {
    printf("here!\n");
    if (robot_->homing_en == 1) {
      SetMotorAbsPos(upwheel_,
                     upwheel_->PosPV + (robot_->upwheel_pos) * (-100));
      //  clear enable flag
      robot_->homing_en = 0;
    }
  }

  // pulleys homing torque
  // while (robot_->pulleys_homing_done != 1) {
  //   delay_us(10);
  // }

  // down claw hold
  // DownClawHold();

  // up claw hold
  UpClawHold();

  // enable pulleys
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // tighten pulleys
  PulleysTorque(kPulleysTightenTorque);

  // save pulleys' home pos
  pulley1_->home_pos = pulley1_->PosPV;
  pulley2_->home_pos = pulley2_->PosPV;

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);
  MotorDisable(upclaw_);
  MotorDisable(upwheel_);

  robot_->homing_done = 0;
  robot_->debug_en = 0;
}

// down claw debug
void robot::DownClawHoldDebug(void) {
  // DownClawHold();
  // // MotorEnable(downclaw2_);
  // // SetTargetTorque(downclaw2_, kDownClawHoldTorque);
  // // wait for loose cmd
  // while (robot_->down_claw_debug_loose == 0) {
  //   delay_us(1000);
  //   // SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
  // }

  // DownClawLoose();

  MotorEnable(downclaw2_);
  SetTargetTorque(downclaw2_, kDownClawHoldTorque);

  // wait hold complete
  while (downclaw2_->actual_average_vel != 0) {
    delay_us(10);
  }

  MotorEnable(downclaw2_);
  SetMotorAbsPos(downclaw2_, downclaw2_->PosPV + kDownClawLooseDistance);

  // clear debug parameters
  robot_->down_claw_debug_loose = 0;
  // disable the debug
  robot_->debug_en = 0;
}

// up claw hold debug
void robot::UpClawHoldDebug(void) {
  UpClawHold();
  while (robot_->up_claw_debug_loose != 1) {
    delay_us(kDelayEpos);
  }
  UpClawLoose();
  robot_->up_claw_debug_loose = 0;
  // disable the debug
  robot_->debug_en = 0;
}

// up claw hold
void robot::UpClawHold() {
  SetTargetTorque(upclaw_, kUpClawHoldTorque);

  // wait hold complete
  while (upclaw_->actual_average_vel != 0) {
    delay_us(10);
  }

  // change upclaw1 state to hold
  upclaw_->motion_state = kHold;
}

// up claw loose
void robot::UpClawLoose() {
  SetMotorAbsPos(upclaw_, upclaw_->PosPV + kUpClawLooseDistance);

  // change upclaw1 state to loose
  upclaw_->motion_state = kLoose;
}

// down claw hold
void robot::DownClawHold() {
  // MotorEnable(downclaw2_);

  // MotorEnable(kDownClaw1);
  // MotorEnable(kDownClaw2);
  // printf("downclaw2 id %d\n", downclaw2_->motor_id);
  SetTargetTorque(downclaw1_, kDownClawHoldTorque);
  // SetTargetTorque(downclaw2_, kDownClawHoldTorque);

  // wait hold complete
  while (downclaw1_->actual_average_vel != 0 /* ||
         downclaw2_->actual_average_vel != 0 */) {
    delay_us(10);
  }

  // change downclaw1 state to hold
  downclaw1_->motion_state = kHold;
  // downclaw2_->motion_state = kHold;
}

// down claw loose
void robot::DownClawLoose() {
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);

  // SetMotorAbsPos(downclaw1_, downclaw1_->PosPV + kDownClawLooseDistance);
  SetMotorAbsPos(downclaw1_, downclaw2_,
                 downclaw1_->PosPV + kDownClawLooseDistance,
                 downclaw2_->PosPV + kDownClawLooseDistance);
  // change downclaw1 state to
  downclaw1_->motion_state = kLoose;
  downclaw2_->motion_state = kLoose;
}

// Pulleys tighten
void robot::PulleysTorque(__s16 torque) {
  SetTargetTorque(pulley1_, torque);
  SetTargetTorque(pulley2_, torque);

  // wait pulleys tighten complete
  while (pulley1_->actual_average_vel != 0 ||
         pulley2_->actual_average_vel != 0) {
    delay_us(10);
  }
}

// Pulleys move up
void robot::PulleysMoveUp() {
  // SetMotorAbsPos(pulley1_, pulley2_, pulley1_->PosPV +
  // kPulleysMoveUpDistance,
  //                pulley2_->PosPV + kPulleysMoveUpDistance);
  SetMotorAbsPos(pulley1_, pulley2_, pulley1_->home_pos, pulley2_->home_pos);
}

// master move up
void robot::MasterMoveUp() {
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // down claw hold
  DownClawHold();

  PulleysTorque(kPulleysTightenTorque);

  // lock the pos
  SetMotorAbsPos(pulley1_, pulley1_->PosPV);
  SetMotorAbsPos(pulley2_, pulley2_->PosPV);

  // save init pos
  pulley1_->init_pos = pulley1_->PosPV;
  pulley2_->init_pos = pulley2_->PosPV;

  // down claw loose
  DownClawLoose();

  // pulleys move up;
  PulleysMoveUp();

  // up delta pos
  pulley1_->up_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->up_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);

  robot_->debug_en = 0;
}

// master move down
void robot::MasterMoveDown() {
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // tighten pulleys
  PulleysTorque(kPulleysTightenTorque);

  // lock the pos
  SetMotorAbsPos(pulley1_, pulley1_->PosPV);
  SetMotorAbsPos(pulley2_, pulley2_->PosPV);

  // down claw loose
  DownClawLoose();

  PulleysMoveDown();

  // down claw hold
  DownClawHold();

  // disable the debug
  robot_->debug_en = 0;
}

// pulleys move down
void robot::PulleysMoveDown() {
  // SetMotorAbsPos(pulley1_, pulley2_, pulley1_->PosPV +
  // kPulleysMoveDownDistance,
  //                pulley2_->PosPV + kPulleysMoveDownDistance);
  SetMotorAbsPos(pulley1_, pulley2_, pulley1_->PosPV - pulley1_->up_delta_pos,
                 pulley2_->PosPV - pulley2_->up_delta_pos);
}

// upwheel speed down thread
void robot::UpWheelSpeedDown() {
  // wait untill distance reached
  while (upwheel_->PosPV > (upwheel_->init_pos + kUpwheelMoveDownDistance)) {
    delay_us(10);
  }

  MotorDisable(upwheel_);
  delay_us(kDelayEpos);

  //   delta pos
  upwheel_->delta_pos = upwheel_->PosPV - upwheel_->init_pos;

  printf("upwheel delta pos:%d\n", upwheel_->delta_pos);
}

// pulley1 speed down thread, direction!
void robot::Pulley1SpeedDown() {
  // wait untill reach the home pos
  while (pulley1_->PosPV < pulley1_->home_pos-10000) {
    delay_us(10);
  }

  MotorDisable(pulley1_);
  pulley1_->delta_pos = pulley1_->PosPV - pulley1_->init_pos;
  printf("pulley1 delta pos:%d\n", pulley1_->delta_pos);
}

// pulley2 speed down thread, direction!
void robot::Pulley2SpeedDown() {
  while (pulley2_->PosPV < pulley2_->home_pos-10000) {
    delay_us(10);
  }

  MotorDisable(pulley2_);
  pulley2_->delta_pos = pulley2_->PosPV - pulley2_->init_pos;
  printf("pulley2 delta pos:%d\n", pulley2_->delta_pos);
}

// up wheel move down
void robot::UpWheelMoveDown() {
  //   save init pos
  upwheel_->init_pos = upwheel_->PosPV;
  pulley1_->init_pos = pulley1_->PosPV;
  pulley2_->init_pos = pulley2_->PosPV;

  // set speed
  SetMotorSpeed(pulley1_, (__s32)(-kMoveDownSpeed * kDownSpeedFactor));
  SetMotorSpeed(pulley2_, (__s32)(-kMoveDownSpeed * kDownSpeedFactor));
  SetMotorSpeed(upwheel_, kMoveDownSpeed);

  // slave moves up thread
  thread upwheel_speeddown_thread(&robot::UpWheelSpeedDown, this);
  thread pulley1_speeddown_thread(&robot::Pulley1SpeedDown, this);
  thread pulley2_speeddown_thread(&robot::Pulley2SpeedDown, this);

  // wait thread complete
  upwheel_speeddown_thread.join();
  pulley1_speeddown_thread.join();
  pulley2_speeddown_thread.join();
}

// slave move down
void robot::SlaveMoveDown() {
  // enable motors
  MotorEnable(upclaw_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);
  MotorEnable(upwheel_);
  MotorEnable(downclaw1_);
  // upclaw hold
  UpClawHold();

  PulleysTorque(kPulleysTightenTorque);

  // disable pulleys
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);

  // upclaw loose
  UpClawLoose();

  // enable pulleys
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  //  move down motion
  UpWheelMoveDown();

  // up claw hold
  // UpClawHold();

  // disable up claw
  MotorDisable(upclaw_);

  robot_->debug_en = 0;
}

void robot::SlaveMoveUp() {
  // enable motors
  MotorEnable(upclaw_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);
  MotorEnable(upwheel_);
  MotorEnable(downclaw1_);

  //   up claw hold
  UpClawHold();

  //   wait 2s
  // sleep(2);

  // tighten pulleys
  PulleysTorque(kPulleysTightenTorque);

  // wait pulleys tighten
  // sleep(2);

  // disable pulleys
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);

  // up claw loose
  UpClawLoose();

  // enable pulleys
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // move up motion
  UpWheelMoveUp();

  //   up claw hold
  UpClawHold();

  //  disable up claw
  MotorDisable(upclaw_);

  // disable debug
  robot_->debug_en = 0;
}

// upwheel speed up thread
void robot::UpWheelSpeedUp() {
  // wait untill distance reached
  while (upwheel_->PosPV < (upwheel_->init_pos + kUpwheelMoveUpDistance)) {
    printf("init pos:%d \n", upwheel_->init_pos);
    printf(
        "upwheel pos not reached, error->%d\n",
        abs(upwheel_->PosPV - (upwheel_->init_pos + kUpwheelMoveUpDistance)));
    printf("current pos:%d, target pos:%d\n", upclaw_->PosPV,
           (upwheel_->init_pos + kUpwheelMoveUpDistance));
    delay_us(10);
  }

  // stop
  // SetMotorSpeed(upwheel_, 0);
  MotorDisable(upwheel_);

  delay_us(kDelayEpos);
  //   delta pos
  upwheel_->delta_pos = upwheel_->PosPV - upwheel_->init_pos;

  printf("upwheel delta pos:%d\n", upwheel_->delta_pos);
}

// pulley1 speed up thread, direction!
void robot::Pulley1SpeedUp() {
  // wait untill distance reached,1000 inc error!
  while (pulley1_->PosPV >
         (pulley1_->init_pos - kUpwheelMoveUpDistance * kDisFactor1)) {
    //  printf("pulley1 !\n");
    delay_us(10);
  }

  // stop
  // SetMotorSpeed(pulley1_, 0);
  MotorDisable(pulley1_);
  delay_us(kDelayEpos);
  pulley1_->delta_pos = pulley1_->PosPV - pulley1_->init_pos;
  printf("pulley1 delta pos:%d\n", pulley1_->delta_pos);
}

// pulley2 speed up thread, direction!
void robot::Pulley2SpeedUp() {
  // // wait untill distance reached,1000 inc error!
  while (pulley2_->PosPV >
         (pulley2_->init_pos - kUpwheelMoveUpDistance * kDisFactor2)) {
    delay_us(10);
  }

  // stop
  // SetMotorSpeed(pulley2_, 0);
  MotorDisable(pulley2_);
  delay_us(kDelayEpos);
  pulley2_->delta_pos = pulley2_->PosPV - pulley2_->init_pos;
  printf("pulley2 delta pos:%d\n", pulley2_->delta_pos);
}

void robot::UpWheelMoveUp() {
  MotorEnable(upwheel_);
  //   save init pos
  upwheel_->init_pos = upwheel_->PosPV;
  pulley1_->init_pos = pulley1_->PosPV;
  pulley2_->init_pos = pulley2_->PosPV;

  // set speed
  SetMotorSpeed(pulley1_, (__s32)(-kMoveUpSpeed * kSpeedFactor));
  SetMotorSpeed(pulley2_, (__s32)(-kMoveUpSpeed * kSpeedFactor));
  SetMotorSpeed(upwheel_, kMoveUpSpeed);

  // slave moves up thread
  thread upwheel_speedup_thread(&robot::UpWheelSpeedUp, this);
  thread pulley1_speedup_thread(&robot::Pulley1SpeedUp, this);
  thread pulley2_speedup_thread(&robot::Pulley2SpeedUp, this);

  // wait thread complete
  upwheel_speedup_thread.join();
  pulley1_speedup_thread.join();
  pulley2_speedup_thread.join();
}