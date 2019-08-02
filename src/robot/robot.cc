#include "robot.hpp"

robot::robot(USHORT reg[]) : maxon()
{
    // define the pointer address
    upclaw_ = (maxon_type *)&reg[100];
    upwheel_ = (maxon_type *)&reg[150];
    downclaw1_ = (maxon_type *)&reg[200];
    pulley1_ = (maxon_type *)&reg[250];
    pulley2_ = (maxon_type *)&reg[300];

    robot_ = (robot_type *)&reg[0];

    // defualt debug mode
    robot_->mode_select = 1;

    // defualt debug mode select
    robot_->debug_mode_select = 8;

    // defualt RxPDOmapping
}

robot::~robot()
{
}

/* -------------------------------robot control---------------------------------- */
void robot::system(void)
{

    switch (robot_->system_state)
    {
        // robot operate mode select
    case kIdleMode:
        if (robot_->mode_select == kIdleMode || robot_->debug_mode_select == 0)
        {
            robot_->system_state = kIdleMode;
            // clear variables
            robot_->upclaw_debug_factor = 0;
            robot_->upwheel_debug_factor = 0;
            robot_->pulleys_distance_factor = 0;
        }
        else if (robot_->mode_select == kDebugMode)
        {
            printf("Enter debug mode!\n");
            // start all nodes
            NMTstart(0);

            robot_->system_state = kDebugMode;
        }
        else
        {
            robot_->system_state = kNomalMode;
        }

        break;

        // debug mode
    case kDebugMode:

        if (robot_->debug_en == 1)
        {
            // start NMT
            NMTstart(0);
            // wait epos
            delay_us(kDelayEpos);

            // choose debug mode
            switch (robot_->debug_mode_select)
            {
                //claw motor debug
            case kUpClawMotorDebug:
                printf("Up Claw debug!\n");
                UpClawDebug();
                break;

                //upwheel motor debug
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
                break;

            case kMasterMoveUp:
                MasterMoveUp();
                printf("Master Move up debug!\n");
                break;

            case kMasterMoveDown:
                MasterMoveDown();
                printf("Master Move Down debug!\n");
                break;

            default:
                break;
            }
        }

        if (robot_->debug_mode_select != 0)
        {
            robot_->system_state = kDebugMode;
        }
        else
        {
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
void robot::UpClawDebug(void)
{
    // enable claw motor
    MotorEnable(kUpClaw);

    MoveRelative(kUpClaw, robot_->upclaw_debug_factor * kUpClawDebugRelaPos);

    // disable the debug
    robot_->debug_en = 0;
}

// UpWheel Debug
void robot::UpWheelDebug(void)
{

    MoveRelative(kUpWheel, robot_->upwheel_debug_factor * kUpWheelDebugRelaPos);

    // disable the debug
    robot_->debug_en = 0;
}

// Pulleys Debug
void robot::PulleysDebug(void)
{
    //move to destination
    MoveRelative(kPulley1, kPulley2, robot_->pulleys_distance_factor * kPulleysDebugRelaPos);

    // disable the debug
    robot_->debug_en = 0;
}

void robot::PulleysHomingDebug(void)
{
    // change to torque mode
    ChangeToTorqueMode(pulley1_, pulley2_);

    // enable pulleys
    MotorEnable(kPulley1);
    MotorEnable(kPulley2);

    // wait for homing done
    while (robot_->pulleys_homing_done != 1)
    {
        SetTargetTorque(kPulley1, robot_->pulleys_homing_torque * 10);
        SetTargetTorque(kPulley2, robot_->pulleys_homing_torque * 10);
        printf("pulleys homing torq: pulley1-> %d pulley2->%d\n", pulley1_->TrqPV, pulley2_->TrqPV);
    }

    // // change to position mode
    // ChangeToPositionMode(kPulley1, kPulley2);

    // // enable pulleys
    // MotorEnable(kPulley1);
    // MotorEnable(kPulley2);

    // // set move distance
    // MoveRelative(kPulley1, 50000);
    // MoveRelative(kPulley2, 50000);

    // disable the debug
    robot_->debug_en = 0;

    // clear homing done flag
    robot_->pulleys_homing_done = 0;
}

// homing
__u16 robot::Homing(maxon_type *motor)
{
    while (motor->homing_state != kHomingDone)
    {
        switch (motor->homing_state)
        {
        case kHomingIdle:
            if (motor->homing_en == 1)
            {
                motor->homing_state = kHoming;
            }
            else
            {
                motor->homing_state = kHomingIdle;
            }
            break;

        case kHoming:
            if (abs(motor->TrqPV) < motor->homing_threhold)
            {
                MoveRelative(motor->motor_id, motor->homing_delta_pos);
            }
            else
            {
                motor->homing_state = kHomingDone;
            }

            break;

        case kHomingDone:
            // stop motor
            MotorQuickStop(motor->motor_id);
            // save the lock position
            motor->PosLocked = motor->PosPV;
            break;

        default:
            break;
        }
    }

    return motor->homing_state;
}

// down claw debug
void robot::DownClawHoldDebug(void)
{
    // disable down claw 1
    MotorDisable(kDownClaw1);

    // change to CST mode;
    SetMotorMode(kDownClaw1, 0x0A);

    // remap TxPDO4 to target torque
    TxPDO4Remap(kDownClaw1, kOBJTargetTorque);
    NMTstart(kDownClaw1);

    MotorEnable(kDownClaw1);

    // set initial target torque
    SetTargetTorque(kDownClaw1, kDownClawInitialTorque);

    // set to 20% of target torque
    while (downclaw1_->TrqPV > 0.5 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.2 * kDownClawHoldTorque);
    }

    // set to 40% of target torque
    while (downclaw1_->TrqPV < 0.4 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.4 * kDownClawHoldTorque);
    }

    // set to 60% of target torque
    while (downclaw1_->TrqPV < 0.6 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.6 * kDownClawHoldTorque);
    }

    // set to 80% of target torque
    while (downclaw1_->TrqPV < 0.8 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.8 * kDownClawHoldTorque);
    }

    // set to 100% of target torque
    while (downclaw1_->TrqPV < kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
    }

    // wait for loose cmd
    while (robot_->down_claw_debug_loose == 0)
    {
        delay_us(1000);
        SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
    }

    // // change motor motion state to hold;
    // downclaw1_->motion_state = kHold;

    // MotorDisable(kDownClaw1);
    // // remap TxPdo4 to mode of operation
    // TxPDO4Remap(kDownClaw1, kOBJModeOfOperation);
    // NMTstart(kDownClaw1);
    // // change to PPM mode;
    // SetMotorMode(kDownClaw1, 0x01);

    // // loose down claw
    // MotorEnable(kDownClaw1);
    // MoveRelative(kDownClaw1, kDownClawLooseDistance);

    // // change motor motion state to loose;
    // downclaw1_->motion_state = kLoose;

    //clear debug parameters
    robot_->down_claw_debug_loose = 0;
    // disable the debug
    robot_->debug_en = 0;
}

// up claw hold debug
void robot::UpClawHoldDebug(void)
{
    __u8 loose_counter = 0;

    // disable up claw
    MotorDisable(kUpClaw);

    // change to CST mode;
    SetMotorMode(kUpClaw, 0x0A);

    // remap TxPDO4 to target torque
    TxPDO4Remap(kUpClaw, kOBJTargetTorque);

    MotorEnable(kUpClaw);

    // set initial target torque 60%
    SetTargetTorque(kUpClaw, kUpClawInitialTorque);

    // set to 50% of target torque
    while (upclaw_->TrqPV > 0.5 * kUpClawHoldTorque)
    {
        SetTargetTorque(kUpClaw, 0.5 * kUpClawHoldTorque);
    }

    // set to 80% of target torque
    while (upclaw_->TrqPV < 0.8 * kUpClawHoldTorque)
    {
        SetTargetTorque(kUpClaw, 0.8 * kUpClawHoldTorque);
    }

    // set to 100% of target torque
    while (upclaw_->TrqPV < 1 * kUpClawHoldTorque)
    {
        SetTargetTorque(kUpClaw, 1 * kUpClawHoldTorque);
    }

    // change motor motion state to hold;
    upclaw_->motion_state = kHold;

    // wait for done cmd
    while (robot_->up_claw_hold_done != 1)
    {
        if (robot_->up_claw_debug_loose == 0)
        {
            delay_us(1000);
            SetTargetTorque(kUpClaw, kUpClawHoldTorque);
            printf("upclaw hold torq: %d\n", upclaw_->TrqPV);
        }
        else if (robot_->up_claw_debug_loose == 1 && loose_counter == 0)
        {
            // loose up claw
            MotorDisable(kUpClaw);
            // remap TxPdo4 to mode of operation
            TxPDO4Remap(kUpClaw, kOBJModeOfOperation);

            // change to PPM mode;
            SetMotorMode(kUpClaw, 0x01);

            // loose up claw
            MotorEnable(kUpClaw);
            MoveRelative(kUpClaw, kUpClawLooseDistance);

            // change motor motion state to loose;
            upclaw_->motion_state = kLoose;

            loose_counter = 1;
        }

        printf("up_claw_hold_done: %d\n", robot_->up_claw_hold_done);
    }

    //clear debug parameters
    robot_->up_claw_debug_loose = 0;

    // clear done flag
    robot_->up_claw_hold_done = 0;

    loose_counter = 0;
    // disable the debug
    robot_->debug_en = 0;
}

// down claw hold
void robot::DownClawHold()
{
    ChangeToTorqueMode(kDownClaw1);

    MotorEnable(kDownClaw1);

    // set initial target torque
    SetTargetTorque(kDownClaw1, kDownClawInitialTorque);

    // set to 50% of target torque
    while (downclaw1_->TrqPV > 0.5 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.5 * kDownClawHoldTorque);
    }

    // set to 40% of target torque
    // while (downclaw1_->TrqPV < 0.4 * kDownClawHoldTorque)
    // {
    //     SetTargetTorque(kDownClaw1, 0.4 * kDownClawHoldTorque);
    // }

    // set to 60% of target torque
    // while (downclaw1_->TrqPV < 0.6 * kDownClawHoldTorque)
    // {
    //     SetTargetTorque(kDownClaw1, 0.6 * kDownClawHoldTorque);
    // }

    // set to 80% of target torque
    while (downclaw1_->TrqPV < 0.8 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.8 * kDownClawHoldTorque);
    }

    // set to 100% of target torque
    while (downclaw1_->TrqPV < kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
    }

    // wait the toruqe reach the target, 1% error
    while (abs(downclaw1_->TrqPV - kDownClawHoldTorque) > 10)
    {
        // delay 1ms
        delay_us(1000);
        printf("torque error: %d\n", abs(downclaw1_->TrqPV - kDownClawHoldTorque));
    }

    // change downclaw1 state to hold
    downclaw1_->motion_state = kHold;
}

// down claw loose
void robot::DownClawLoose()
{
    __s32 init_pos;

    // change to position mode
    ChangeToPositionMode(kDownClaw1);

    // enable motor
    MotorEnable(kDownClaw1);

    // save inint pos
    init_pos = downclaw1_->PosPV;

    // set move distance
    MoveRelative(kDownClaw1, kDownClawLooseDistance);

    // wait for claw reaching the target pos, 100 inc error
    while (abs(abs(downclaw1_->PosPV - init_pos) - abs(kDownClawLooseDistance)) > 100)
    {
        // delay 1ms
        delay_us(1000);
        printf("cunrrent pos:%d\n", downclaw1_->PosPV);
        printf("init pos:%d\n", init_pos);
        printf("cmd delta pos:%d\n", abs(kDownClawLooseDistance));
        printf("pos error: %d\n", abs(abs(downclaw1_->PosPV - init_pos) - abs(kDownClawLooseDistance)));
    }

    // change downclaw1 state to
    downclaw1_->motion_state = kLoose;
}

// Pulleys tighten
void robot::PulleysTighten()
{
    delay_us(100000);

    // change to torque mode
    if (ChangeToTorqueMode(pulley1_, pulley2_) == kCfgFail)
    {
        printf("torque mode config fail!\n");
    }

    // enable pulleys
    MotorEnable(kPulley1);
    MotorEnable(kPulley2);

    // set tighten torque
    SetTargetTorque(kPulley1, kPulleysTightenTorque);
    SetTargetTorque(kPulley2, kPulleysTightenTorque);

    // wait the toruqe reach the target, 1% error
    while ((abs(pulley1_->TrqPV - kPulleysTightenTorque) > 10) && (abs(pulley2_->TrqPV - kPulleysTightenTorque) > 10))
    {

        // delay 100us
        delay_us(100);
        printf("pulleys tighten!\n");
    }

    // change pulleys state to tighten
    pulley1_->motion_state = kTighten;
    pulley2_->motion_state = kTighten;
}

/* --------------pulleys move up threads----------------- */
void robot::Pulley1MoveUpThread()
{
    __s32 init_pos1;
    // save inint pos
    init_pos1 = pulley1_->PosPV;
    printf("pulley1 move up thread!\n");
    // wait pulleys reach the target pos, 100 inc error
    while (pulley1_->PosPV < (init_pos1 + kPulleysMoveUpDistance))
    {
        // delay 10us
        // delay_us(10);
        printf("cunrrent pos1:%d\n", pulley1_->PosPV);
        printf("target pos1:%d\n", (init_pos1 + kPulleysMoveUpDistance));
    }

    printf("cunrrent pos1:%d\n", pulley1_->PosPV);
    // change to PPM
    ChangeToPositionMode(kPulley1);

    // enable motor
    MotorEnable(kPulley1);
}

//
void robot::Pulley2MoveUpThread()
{
    __s32 init_pos2;
    init_pos2 = pulley2_->PosPV;
    printf("pulley2 move up thread!\n");
    while (pulley2_->PosPV < (init_pos2 + kPulleysMoveUpDistance))
    {
        // // delay 10us
        delay_us(10);
        printf("cunrrent pos2:%d\n", pulley2_->PosPV);
        printf("target pos2:%d\n", (init_pos2 + kPulleysMoveUpDistance));
    }

    printf("cunrrent pos2:%d\n", pulley2_->PosPV);
    // change to PPM
    ChangeToPositionMode(kPulley2);

    // enable motor
    MotorEnable(kPulley2);
}

// Pulleys move up
void robot::PulleysMoveUp()
{

    // set pulling torque
    SetTargetTorque(kPulley1, kPulleysPullTorque);
    SetTargetTorque(kPulley2, kPulleysPullTorque);

    // wait the toruqe reach the target, 1% error
    while (abs(pulley1_->TrqPV - kPulleysPullTorque) > 10 && abs(pulley2_->TrqPV - kPulleysPullTorque) > 10)
    {
        // delay 1ms
        delay_us(1000);
        printf("pulley1 torque:%d\n", pulley1_->TrqPV);
    }

    // change pulleys state to pulling
    pulley1_->motion_state = kPulling;
    pulley2_->motion_state = kPulling;

    // two pulleys move up thread, this pointer!!!
    thread pulley1_moveup_thread(&robot::Pulley1MoveUpThread, this);
    thread pulley2_moveup_thread(&robot::Pulley2MoveUpThread, this);

    // wait distance reached
    pulley1_moveup_thread.join();
    pulley2_moveup_thread.join();

    // wait for mode changed and enabled
    while ((pulley1_->mode_display != 0x01 || pulley1_->StatusWord != 0x0637) || (pulley2_->mode_display != 0x01 || pulley2_->StatusWord != 0x0637))
    {
        delay_us(1000);
    }

    // change pulleys state to stop
    pulley1_->motion_state = kStop;
    pulley2_->motion_state = kStop;
}

// master move up
void robot::MasterMoveUp()
{

    // down claw hold
    DownClawHold();

    NMTstart();
    // delay_us(1000);

    PulleysTighten();

    NMTstart();
    // delay_us(1000);
    // down claw loose
    DownClawLoose();

    NMTstart();
    // delay_us(1000);

    // pulleys move up;
    PulleysMoveUp();

    // NMTstart();

    // // down claw hold
    // DownClawHold();

    // // disable motors
    // MotorDisable(kPulley1);
    // MotorDisable(kPulley2);
    // MotorDisable(kDownClaw1);

    // DownClawLoose();

    // disable the debug
    robot_->debug_en = 0;
}

/* --------------pulleys move down threads----------------- */
void robot::Pulley1MoveDownThread()
{
    __s32 init_pos1;
    // save inint pos
    init_pos1 = pulley1_->PosPV;
    printf("pulley1 move down thread!\n");
    // wait pulleys reach the target pos, 100 inc error
    while (pulley1_->PosPV > (init_pos1 - kPulleysMoveDownDistance))
    {
        // delay 10us
        // delay_us(10);
        printf("cunrrent pos1:%d\n", pulley1_->PosPV);
        printf("target pos1:%d\n", (init_pos1 - kPulleysMoveDownDistance));
    }

    printf("cunrrent pos1:%d\n", pulley1_->PosPV);
    // change to PPM
    ChangeToPositionMode(kPulley1);

    // enable motor
    MotorEnable(kPulley1);
}

void robot::Pulley2MoveDownThread()
{
    __s32 init_pos2;
    // save inint pos
    init_pos2 = pulley2_->PosPV;
    printf("pulley1 move down thread!\n");
    // wait pulleys reach the target pos, 100 inc error
    while (pulley2_->PosPV > (init_pos2 - kPulleysMoveDownDistance))
    {
        // delay 10us
        // delay_us(10);
        printf("cunrrent pos2:%d\n", pulley2_->PosPV);
        printf("target pos2:%d\n", (init_pos2 - kPulleysMoveDownDistance));
    }

    printf("cunrrent pos2:%d\n", pulley2_->PosPV);
    // change to PPM
    ChangeToPositionMode(kPulley2);

    // enable motor
    MotorEnable(kPulley2);
}

// master move down
void robot::MasterMoveDown()
{

    // tighten pulleys
    PulleysTighten();
    NMTstart();

    // two pulleys move down thread, this pointer!!!
    thread pulley1_movedown_thread(&robot::Pulley1MoveDownThread, this);
    thread pulley2_movedown_thread(&robot::Pulley2MoveDownThread, this);

    // loose down claw
    DownClawLoose();

    // wait distance reached
    pulley1_movedown_thread.join();
    pulley2_movedown_thread.join();

    // wait for mode changed and enabled
    while ((pulley1_->mode_display != 0x01 || pulley1_->StatusWord != 0x0637) || (pulley2_->mode_display != 0x01 || pulley2_->StatusWord != 0x0637))
    {
        delay_us(1000);
    }

    // change pulleys state to stop
    pulley1_->motion_state = kStop;
    pulley2_->motion_state = kStop;

    // NMTstart();

    // // down claw hold
    // DownClawHold();

    // // disable motors
    // MotorDisable(kPulley1);
    // MotorDisable(kPulley2);
    // MotorDisable(kDownClaw1);

    // DownClawLoose();

    // disable the debug
    robot_->debug_en = 0;
}

// pulleys move down
void robot::PulleysMoveDown()
{
}