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

  // default motion parameter
  robot_->motion_para = kParaShortStepLoad;
  // defualt debug mode
  robot_->mode_select = 1;

  // defualt debug mode select
  robot_->debug_mode_select = 21;
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
            printf("Slave Move Down debug!\n");
            SlaveMoveDown();
            break;

          case kSlaveMoveUp:
            printf("Slave Move Up debug!\n");
            SlaveMoveUp();
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
            MotorEnable(downclaw2_);
            DownClawHold();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kDownClawLoose:
            printf("down claw loose!\n");
            MotorEnable(downclaw1_);
            MotorEnable(downclaw2_);
            DownClawLoose();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kMoveUp:
            printf("robot move up!\n");
            MoveUp();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kMoveDown:
            printf("robot move down!\n");
            MoveDown();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kMotionCycleInitUp:
            printf("cycle motion init up!\n");
            MoveUp();
            MoveUp();
            MoveDown();
            MoveDown();
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kQuitDebug:
            printf("quit debug!\n");
            // disable all motors
            MotorDisable(upclaw_);
            MotorDisable(upwheel_);
            MotorDisable(pulley1_);
            MotorDisable(pulley2_);
            MotorDisable(downclaw1_);
            MotorDisable(downclaw2_);
            // stop the canopen network!
            NMTstop(0);
            // disable the debug
            robot_->debug_en = 0;
            break;

          case kMasterMoveUpDebug:
            printf("master move up debug!\n");
            MasterMoveUpDebug();
            break;

          case kMasterMoveDownDebug:
            printf("master move down debug!\n");
            MasterMoveDownDebug();
            break;

          case kTest:
            printf("test!\n");
            // enable motors
            MotorEnable(downclaw1_);
            MotorEnable(downclaw2_);
            MotorEnable(pulley1_);
            MotorEnable(pulley2_);
            // MotorEnable(upclaw_);

            // upclaw hold
            // UpClawHold();

            PulleysTorque(kPulleysTightenTorque);

            // lock the pos
            SetMotorAbsPos(pulley1_, pulley1_->PosPV);
            SetMotorAbsPos(pulley2_, pulley2_->PosPV);

            // down claw loose
            DownClawLoose();

            while (robot_->load_test_up_en != 1) {
              // wait cmd
              delay_us(1000);
              printf("wait load test en!\n");
            }

            // pulleys move up;
            PulleysMoveUpDebug();

            
            // down claw hold
            DownClawHold();

            // disable motors
            MotorDisable(pulley1_);
            MotorDisable(pulley2_);
            MotorDisable(downclaw1_);
            MotorDisable(downclaw2_);

            // disable the debug
            robot_->debug_en = 0;
            break;

          case kSlaveUpDown:
            printf("slave move up and down!\n");
            SlaveUpDown();
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

    case kNomalMode:
      break;

    default:
      break;
  }
}

/* -------------------------debug function------------------------------ */
void robot::UpClawDebug(void) {
  // disable the debug
  robot_->debug_en = 0;
}

// UpWheel Debug
void robot::UpWheelDebug(void) {
  // disable the debug
  robot_->debug_en = 0;
}

// Pulleys Debug
void robot::PulleysDebug(void) {
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
  MotorEnable(downclaw2_);

  // hold claws
  DownClawHold();
  UpClawHold();

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
      SetMotorAbsPos(upwheel_, upwheel_->PosPV + (robot_->upwheel_pos) * (100));
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
  MotorDisable(downclaw2_);
  MotorDisable(upclaw_);
  MotorDisable(upwheel_);

  robot_->homing_done = 0;
  robot_->debug_en = 0;
}

// down claw debug
void robot::DownClawHoldDebug(void) {
  // DownClawLoose();
  MotorEnable(downclaw2_);

  MotorEnable(downclaw1_);
  SetTargetTorque(downclaw2_, kDownClawHoldTorque);

  // wait hold complete
  while (downclaw2_->actual_average_vel != 0) {
    delay_us(10);
  }

  while (robot_->down_claw_debug_loose != 1) {
    delay_us(10);
  }
  SetMotorAbsPos(downclaw2_, downclaw2_->PosPV + kDownClawLooseDistance);

  // clear debug parameters
  robot_->down_claw_debug_loose = 0;
  // disable the debug
  robot_->debug_en = 0;
}

// up claw hold debug
void robot::UpClawHoldDebug(void) {
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
  SetTargetTorque(downclaw1_, kDownClawHoldTorque);
  SetTargetTorque(downclaw2_, kDownClawHoldTorque);

  // wait hold complete
  while (downclaw1_->actual_average_vel != 0 ||
         downclaw2_->actual_average_vel != 0) {
    delay_us(10);
  }

  // change downclaws' state to hold
  downclaw1_->motion_state = kHold;
  downclaw2_->motion_state = kHold;
}

// down claw loose
void robot::DownClawLoose() {
  // MotorEnable(downclaw1_);
  // MotorEnable(downclaw2_);
  printf("down claw start loosing\n");

  // SetMotorAbsPos(downclaw1_, downclaw1_->PosPV + kDownClawLooseDistance);
  SetMotorAbsPos(downclaw1_, downclaw2_,
                 downclaw1_->PosPV + kDownClawLooseDistance,
                 downclaw2_->PosPV + kDownClawLooseDistance);

  printf("down claw start loose complete!\n");
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

void robot::Pulley1MasterSpeedUp() {
  // wait untill distance reached within master_up_dis_for_ppm!
  while (pulley1_->PosPV <
         pulley1_->home_pos - robot_->motion_para.master_up_dis_for_ppm) {
    delay_us(10);
  }

  // stop
  SetMotorAbsPos(pulley1_, pulley1_->home_pos);
}
void robot::Pulley2MasterSpeedUp() {
  // wait untill distance reached within master_up_dis_for_ppm!
  while (pulley2_->PosPV <
         pulley2_->home_pos - robot_->motion_para.master_up_dis_for_ppm) {
    delay_us(10);
  }

  // stop
  SetMotorAbsPos(pulley2_, pulley2_->home_pos);
}

// Pulleys move up
void robot::PulleysMoveUp(__s32 speed) {
  // set move speed
  SetMotorSpeed(pulley1_, speed);
  SetMotorSpeed(pulley2_, speed);
  // master move up thread
  thread pulley1_master_speed_up_thread(&robot::Pulley1MasterSpeedUp, this);
  thread pulley2_master_speed_up_thread(&robot::Pulley2MasterSpeedUp, this);

  // wait thread complete
  pulley1_master_speed_up_thread.join();
  pulley2_master_speed_up_thread.join();
}

void robot::PulleysMoveUp() {
  // move to home position
  SetMotorAbsPos(pulley1_, pulley2_, pulley1_->home_pos, pulley2_->home_pos);
}

void robot::PulleysMoveUpDebug() {
  SetMotorAbsPos(pulley1_, pulley2_, pulley1_->PosPV + kPulleysMoveUpDistance,
                 pulley2_->PosPV + kPulleysMoveUpDistance);
}

void robot::PulleysMoveDownDebug() {
  SetMotorAbsPos(pulley1_, pulley2_, pulley1_->PosPV + kPulleysMoveDownDistance,
                 pulley2_->PosPV + kPulleysMoveDownDistance);
}

// master move up
void robot::MasterMoveUp() {
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // down claw hold
  // DownClawHold();

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
  // PulleysMoveUp(robot_->motion_para.master_move_up_speed);
  PulleysMoveUp();

  // pulleys up delta pos
  pulley1_->up_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->up_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);
  MotorDisable(downclaw2_);

  robot_->debug_en = 0;
}

// master move up
void robot::MasterMoveUpDebug() {
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);
  MotorEnable(upclaw_);

  // up claw hold
  // UpClawHold();

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
  PulleysMoveUpDebug();

  // pulleys up delta pos
  pulley1_->up_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->up_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);
  MotorDisable(downclaw2_);

  robot_->debug_en = 0;
}

void robot::MasterMoveDownDebug() {
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);
  MotorEnable(upclaw_);

  // up claw hold
  // UpClawHold();

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

  // pulleys move down;
  PulleysMoveDownDebug();

  // pulleys up delta pos
  pulley1_->up_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->up_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);
  MotorDisable(downclaw2_);

  robot_->debug_en = 0;
}

// master move down
void robot::MasterMoveDown() {
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // tighten pulleys
  PulleysTorque(kPulleysTightenTorque);

  // lock the pos
  SetMotorAbsPos(pulley1_, pulley1_->PosPV);
  SetMotorAbsPos(pulley2_, pulley2_->PosPV);

  // save init pos
  pulley1_->init_pos = pulley1_->PosPV;
  pulley2_->init_pos = pulley2_->PosPV;

  // save master move down init pos
  pulley1_->master_move_down_init_pos = pulley1_->PosPV;
  pulley2_->master_move_down_init_pos = pulley2_->PosPV;

  // down claw loose
  DownClawLoose();

  // PulleysMoveDown(robot_->motion_para.master_move_down_speed);
  PulleysMoveDown();

  // down delta pos
  pulley1_->down_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->down_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable the debug
  robot_->debug_en = 0;
}

// pulleys move down
void robot::PulleysMoveDown(__s32 speed) {
  // set move speed
  SetMotorSpeed(pulley1_, speed);
  SetMotorSpeed(pulley2_, speed);

  // master move down thread
  thread pulley1_master_speed_down_thread(&robot::Pulley1MasterSpeedDown, this);
  thread pulley2_master_speed_down_thread(&robot::Pulley2MasterSpeedDown, this);

  // wait thread complete
  pulley1_master_speed_down_thread.join();
  pulley2_master_speed_down_thread.join();
}

void robot::Pulley1MasterSpeedDown() {
  // haven't move down yet
  if (pulley1_->down_delta_pos == 0 || pulley2_->down_delta_pos == 0) {
    while (pulley1_->PosPV >
           pulley1_->master_move_down_init_pos -
               robot_->motion_para.upwheel_down_dis *
                   robot_->motion_para.pulley1_master_down_dis_factor +
               robot_->motion_para.master_down_dis_for_ppm) {
      delay_us(10);
    }
    // stop
    SetMotorAbsPos(pulley1_,
                   pulley1_->master_move_down_init_pos -
                       robot_->motion_para.upwheel_down_dis *
                           robot_->motion_para.pulley1_master_down_dis_factor);
  } else {
    // wait untill distance reached within master_down_dis_for_ppm!
    while (pulley1_->PosPV > pulley1_->master_move_down_init_pos -
                                 pulley1_->down_delta_pos +
                                 robot_->motion_para.master_down_dis_for_ppm) {
      delay_us(10);
    }
    // stop
    SetMotorAbsPos(pulley1_, pulley1_->master_move_down_init_pos -
                                 pulley1_->down_delta_pos);
  }
}

void robot::Pulley2MasterSpeedDown() {
  // haven't move down yet
  if (pulley2_->down_delta_pos == 0 || pulley2_->down_delta_pos == 0) {
    while (pulley2_->PosPV >
           pulley2_->master_move_down_init_pos -
               robot_->motion_para.upwheel_down_dis *
                   robot_->motion_para.pulley2_master_down_dis_factor +
               robot_->motion_para.master_down_dis_for_ppm) {
      delay_us(10);
    }
    // stop
    SetMotorAbsPos(pulley2_,
                   pulley2_->master_move_down_init_pos -
                       robot_->motion_para.upwheel_down_dis *
                           robot_->motion_para.pulley2_master_down_dis_factor);
  } else {
    // wait untill distance reached within master_down_dis_for_ppm!
    while (pulley2_->PosPV > pulley2_->master_move_down_init_pos -
                                 pulley2_->down_delta_pos +
                                 robot_->motion_para.master_down_dis_for_ppm) {
      delay_us(10);
    }
    // stop
    SetMotorAbsPos(pulley2_, pulley2_->master_move_down_init_pos -
                                 pulley2_->down_delta_pos);
  }
}

void robot::PulleysMoveDown() {
  // SetMotorAbsPos(pulley1_, pulley2_, pulley1_->PosPV +
  // kPulleysMoveDownDistance,
  //                pulley2_->PosPV + kPulleysMoveDownDistance);

  // haven't move down yet
  if (pulley1_->down_delta_pos == 0 || pulley2_->down_delta_pos == 0) {
    // be careful sign!!!!
    SetMotorAbsPos(pulley1_, pulley2_,
                   pulley1_->PosPV -
                       (robot_->motion_para.upwheel_down_dis) *
                           robot_->motion_para.pulley1_master_down_dis_factor,
                   pulley2_->PosPV -
                       (robot_->motion_para.upwheel_down_dis) *
                           robot_->motion_para.pulley1_master_down_dis_factor);
  } else {
    // SetMotorAbsPos(pulley1_, pulley2_,
    //                pulley1_->PosPV - pulley1_->down_delta_pos,
    //                pulley2_->PosPV - pulley2_->down_delta_pos);

    SetMotorAbsPos(pulley1_, pulley2_,
                   pulley1_->PosPV -
                       (robot_->motion_para.upwheel_down_dis) *
                           robot_->motion_para.pulley1_master_down_dis_factor,
                   pulley2_->PosPV -
                       (robot_->motion_para.upwheel_down_dis) *
                           robot_->motion_para.pulley1_master_down_dis_factor);
  }
}

// upwheel speed down thread
void robot::UpWheelSpeedDown() {
  // wait untill distance reached
  while (upwheel_->PosPV <
         (upwheel_->init_pos + robot_->motion_para.upwheel_down_dis -
          robot_->motion_para.upwheel_move_down_dis_correct -
          robot_->motion_para.slave_down_dis_for_ppm)) {
    delay_us(10);
  }

  // stop
  SetMotorAbsPos(upwheel_,
                 upwheel_->init_pos + robot_->motion_para.upwheel_down_dis -
                     robot_->motion_para.upwheel_move_down_dis_correct);
  MotorDisable(upwheel_);

  //   delta pos
  upwheel_->delta_pos = upwheel_->PosPV - upwheel_->init_pos;

  printf("upwheel delta pos:%d\n", upwheel_->delta_pos);

  // up claw hold
  UpClawHold();
}

// pulley1 speed down thread, direction!
void robot::Pulley1SpeedDown() {
  // wait untill reach the home pos,
  while (pulley1_->PosPV <
         pulley1_->home_pos -
             robot_->motion_para.pulley1_move_down_dis_correct -
             robot_->motion_para.slave_down_dis_for_ppm) {
    delay_us(10);
  }

  // stop
  /* SetMotorAbsPos(
      pulley1_,
      pulley1_->home_pos - robot_->motion_para.pulley1_move_down_dis_correct);
   */
  MotorDisable(pulley1_);

  pulley1_->delta_pos = pulley1_->PosPV - pulley1_->init_pos;
  printf("pulley1 delta pos:%d\n", pulley1_->delta_pos);
}

// pulley2 speed down thread, direction!
void robot::Pulley2SpeedDown() {
  while (pulley2_->PosPV <
         pulley2_->home_pos -
             robot_->motion_para.pulley2_move_down_dis_correct -
             robot_->motion_para.slave_down_dis_for_ppm) {
    delay_us(10);
  }

  // stop
  /* SetMotorAbsPos(
      pulley2_,
      pulley2_->home_pos - robot_->motion_para.pulley2_move_down_dis_correct);
   */
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

  // set speed, upwheel move first
  SetMotorSpeed(upwheel_, robot_->motion_para.move_down_speed);
  SetMotorSpeed(pulley1_, (__s32)(robot_->motion_para.move_down_speed *
                                  robot_->motion_para.down_speed_factor));
  SetMotorSpeed(pulley2_, (__s32)(robot_->motion_para.move_down_speed *
                                  robot_->motion_para.down_speed_factor));

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

  // upclaw hold
  UpClawHold();

  // dont't need to tighten the pulleys when slave move up?
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

  // disable up claw
  MotorDisable(upclaw_);

  robot_->debug_en = 0;
}

void robot::SlaveMoveUp() {
  // enable motors
  MotorEnable(upclaw_);
  MotorEnable(upwheel_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  //   up claw hold
  UpClawHold();

  // don' t need to tighten pulleys when slave moves?
  PulleysTorque(kPulleysTightenTorque);

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
  // UpWheelMoveUp(robot_->motion_para.upwheel_up_dis);

  //  disable up claw
  // MotorDisable(upclaw_);

  // disable debug
  robot_->debug_en = 0;
}

// upwheel speed up thread
void robot::UpWheelSpeedUp() {
  // wait untill distance reached
  while (upwheel_->PosPV >
         (upwheel_->init_pos + robot_->motion_para.upwheel_up_dis)) {
    printf("upwheel pos not reached, error->%d\n",
           abs(upwheel_->PosPV -
               (upwheel_->init_pos + robot_->motion_para.upwheel_up_dis)));

    delay_us(10);
  }

  // stop
  /* SetMotorAbsPos(upwheel_,
                 upwheel_->init_pos + robot_->motion_para.upwheel_up_dis); */
  MotorDisable(upwheel_);

  //   delta pos
  upwheel_->delta_pos = upwheel_->PosPV - upwheel_->init_pos;
  printf("upwheel init pos %d\n", upwheel_->init_pos);
  printf("upwheel current pos %d\n", upwheel_->PosPV);
  printf("upwheel delta pos:%d\n", upwheel_->delta_pos);

  //   up claw hold
  UpClawHold();
}

// pulley1 speed up thread, direction!
void robot::Pulley1SpeedUp() {
  // wait untill distance reached,1000 inc error!
  while (pulley1_->PosPV > (pulley1_->init_pos +
                            robot_->motion_para.upwheel_up_dis *
                                robot_->motion_para.pulley1_dis_factor +
                            robot_->motion_para.slave_up_dis_for_ppm)) {
    delay_us(10);
  }

  // stop
  SetMotorAbsPos(pulley1_, pulley1_->init_pos +
                               robot_->motion_para.upwheel_up_dis *
                                   robot_->motion_para.pulley1_dis_factor);
  MotorDisable(pulley1_);
  pulley1_->delta_pos = pulley1_->PosPV - pulley1_->init_pos;
  printf("pulley1 delta pos:%d\n", pulley1_->delta_pos);
}

// pulley2 speed up thread, direction!
void robot::Pulley2SpeedUp() {
  // wait untill distance reached,1000 inc error!
  while (pulley2_->PosPV > (pulley2_->init_pos +
                            robot_->motion_para.upwheel_up_dis *
                                robot_->motion_para.pulley2_dis_factor +
                            robot_->motion_para.slave_up_dis_for_ppm)) {
    delay_us(10);
  }

  // stop
  SetMotorAbsPos(pulley2_, pulley2_->init_pos +
                               robot_->motion_para.upwheel_up_dis *
                                   robot_->motion_para.pulley2_dis_factor);
  MotorDisable(pulley2_);
  pulley2_->delta_pos = pulley2_->PosPV - pulley2_->init_pos;
  printf("pulley2 delta pos:%d\n", pulley2_->delta_pos);
}

void robot::UpWheelMoveUp() {
  //   save init pos
  upwheel_->init_pos = upwheel_->PosPV;
  pulley1_->init_pos = pulley1_->PosPV;
  pulley2_->init_pos = pulley2_->PosPV;

  // set speed, pulleys move first!
  SetMotorSpeed(pulley1_, (__s32)(robot_->motion_para.move_up_speed *
                                  robot_->motion_para.up_speed_factor));
  SetMotorSpeed(pulley2_, (__s32)(robot_->motion_para.move_up_speed *
                                  robot_->motion_para.up_speed_factor));
  SetMotorSpeed(upwheel_, robot_->motion_para.move_up_speed);

  // slave moves up thread
  thread upwheel_speedup_thread(&robot::UpWheelSpeedUp, this);
  thread pulley1_speedup_thread(&robot::Pulley1SpeedUp, this);
  thread pulley2_speedup_thread(&robot::Pulley2SpeedUp, this);

  // wait thread complete
  upwheel_speedup_thread.join();
  pulley1_speedup_thread.join();
  pulley2_speedup_thread.join();
}

void robot::UpWheelMoveUp(__s32 pos) {
  //   save init pos
  upwheel_->init_pos = upwheel_->PosPV;
  SetMotorAbsPos(upwheel_, upwheel_->PosPV + pos);
}

/*
 *robot move up function
 */
void robot::MoveUp() {
  /* slave move up */
  // enable motors
  MotorEnable(upclaw_);
  MotorEnable(upwheel_);
  // MotorEnable(pulley1_);
  // MotorEnable(pulley2_);

  //   up claw hold
  // UpClawHold();

  // don' t need to tighten pulleys when slave moves?
  // PulleysTorque(kPulleysTightenTorque);

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

  //  disable up claw
  // MotorDisable(upclaw_);

  /* master move up */
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // down claw hold
  // DownClawHold();

  // tighten the pulleys to prevent the master falling down
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
  // PulleysMoveUp(robot_->motion_para.master_move_up_speed);
  PulleysMoveUp();

  // pulleys up delta pos
  pulley1_->up_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->up_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);
  MotorDisable(downclaw2_);

  robot_->debug_en = 0;
}

// robot move down
void robot::MoveDown() {
  /* master move down */
  // enable motors
  MotorEnable(downclaw1_);
  MotorEnable(downclaw2_);
  MotorEnable(pulley1_);
  MotorEnable(pulley2_);

  // tighten pulleys
  PulleysTorque(kPulleysTightenTorque);

  // lock the pos
  SetMotorAbsPos(pulley1_, pulley1_->PosPV);
  SetMotorAbsPos(pulley2_, pulley2_->PosPV);

  // save init pos
  pulley1_->init_pos = pulley1_->PosPV;
  pulley2_->init_pos = pulley2_->PosPV;

  // save master move down init pos
  pulley1_->master_move_down_init_pos = pulley1_->PosPV;
  pulley2_->master_move_down_init_pos = pulley2_->PosPV;

  // down claw loose
  DownClawLoose();

  // PulleysMoveDown(robot_->motion_para.master_move_down_speed);
  PulleysMoveDown();

  // down delta pos
  pulley1_->down_delta_pos = abs(pulley1_->PosPV - pulley1_->init_pos);
  pulley2_->down_delta_pos = abs(pulley2_->PosPV - pulley2_->init_pos);

  // down claw hold
  DownClawHold();

  // disable motors
  MotorDisable(pulley1_);
  MotorDisable(pulley2_);
  MotorDisable(downclaw1_);
  MotorDisable(downclaw2_);

  /* slave move down */
  // enable motors
  MotorEnable(upclaw_);
  // MotorEnable(pulley1_);
  // MotorEnable(pulley2_);
  MotorEnable(upwheel_);

  // upclaw hold?
  // UpClawHold();

  // dont't need to tighten the pulleys when slave move up?
  // PulleysTorque(kPulleysTightenTorque);

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

  // disable up claw
  MotorDisable(upclaw_);

  robot_->debug_en = 0;
}

void robot::SlaveUpDown() {
  // enable upclaw and upwheel
  MotorEnable(upwheel_);
  // MotorEnable(upclaw_);

  // upclaw loose
  // UpClawLoose();

  // move up
  SetMotorAbsPos(upwheel_, upwheel_->PosPV - 1000000);

  // move down
  SetMotorAbsPos(upwheel_, upwheel_->PosPV + 800000);

  // disable motor
  MotorDisable(upwheel_);

  // disable the debug
  robot_->debug_en = 0;
}