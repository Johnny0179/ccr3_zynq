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
    // enable pulleys
    MotorEnable(kPulley1);
    MotorEnable(kPulley2);

    // set pulleys torque
    PulleysTorque(robot_->pulleys_homing_torque * 10);

    // wait for homing done
    while (robot_->pulleys_homing_done != 1)
    {
        SetTargetTorque(kPulley1, robot_->pulleys_homing_torque * 10);
        SetTargetTorque(kPulley2, robot_->pulleys_homing_torque * 10);
        printf("pulleys homing torq: pulley1-> %d pulley2->%d\n", pulley1_->TrqPV, pulley2_->TrqPV);
    }

    // disable the debug
    robot_->debug_en = 0;

    // clear homing done flag
    robot_->pulleys_homing_done = 0;
}

// homing
void robot::Homing(void)
{
    // enable motors
    MotorEnable(kPulley1);
    MotorEnable(kPulley2);
    MotorEnable(kDownClaw1);

    // up claw close

    // pulleys homing torque
}

// down claw debug
void robot::DownClawHoldDebug(void)
{
    DownClawHold();

    // wait for loose cmd
    while (robot_->down_claw_debug_loose == 0)
    {
        delay_us(1000);
        SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
    }

    DownClawLoose();

    // change motor motion state to loose;
    downclaw1_->motion_state = kLoose;

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
    // enable motor
    MotorEnable(downclaw1_);

    // change to torque mode
    ChangeToTorqueMode(downclaw1_);

    // set initial target torque
    SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
    delay_us(kDelayEpos);

    // wait the toruqe reach the target, 1% error
    while (abs(downclaw1_->TrqPV - kDownClawHoldTorque) > 10)
    {
        // delay 1ms
        delay_us(1000);
        printf("torque error: %d\n", abs(downclaw1_->TrqPV - kDownClawHoldTorque));
    }

    // disable motor
    // MotorDisable(kDownClaw1);

    // change downclaw1 state to hold
    downclaw1_->motion_state = kHold;
}

// down claw loose
void robot::DownClawLoose()
{
    __s32 init_pos;

    // change to position mode
    ChangeToPositionMode(downclaw1_);

    // save inint pos
    init_pos = downclaw1_->PosPV;

    // set move distance
    MoveRelative(kDownClaw1, kDownClawLooseDistance);

    // wait for claw reaching the target pos, 1000 inc error
    while (abs(abs(downclaw1_->PosPV - init_pos) - abs(kDownClawLooseDistance)) > 1000)
    {
        // delay 1ms
        delay_us(1000);
        printf("cunrrent pos:%d\n", downclaw1_->PosPV);
        printf("init pos:%d\n", init_pos);
        printf("cmd delta pos:%d\n", abs(kDownClawLooseDistance));
        printf("pos error: %d\n", abs(abs(downclaw1_->PosPV - init_pos) - abs(kDownClawLooseDistance)));
    }

    // disable motor
    // MotorDisable(kDownClaw1);

    // change downclaw1 state to
    downclaw1_->motion_state = kLoose;
}

// Pulleys tighten
void robot::PulleysTorque(__s16 torque)
{

    // change to torque mode
    ChangeToTorqueMode(pulley1_);
    ChangeToTorqueMode(pulley2_);

    // set torque
    SetTargetTorque(kPulley1, torque);
    delay_us(kDelayEpos);
    SetTargetTorque(kPulley2, torque);
    delay_us(kDelayEpos);

    // wait the toruqe reach the target, 1% error
    while ((abs(pulley1_->TrqPV - torque) > 10) && (abs(pulley2_->TrqPV - torque) > 10))
    {

        delay_us(100);
        // printf("pulleys tighten!\n");
    }

    // change pulleys state to tighten
    // pulley1_->motion_state = kTighten;
    // pulley2_->motion_state = kTighten;
}

/* --------------pulleys move up threads----------------- */
void robot::Pulley1MoveUpThread()
{

    printf("pulley1 move up thread!\n");
    // wait pulleys reach the target pos, 100 inc error
    while (abs(pulley1_->PosPV - (pulley1_->init_pos + kPulleysMoveUpDistance)) > 100)
    {
        delay_us(10000);
        printf("cunrrent pos1:%d\n", pulley1_->PosPV);
        printf("target pos1:%d\n", (pulley1_->init_pos + kPulleysMoveUpDistance));
    }

    // update delta pos
    pulley1_->delta_pos = pulley1_->PosPV - pulley1_->init_pos;

    // change to PPM
    ChangeToPositionMode(pulley1_);
}

//
void robot::Pulley2MoveUpThread()
{

    printf("pulley2 move up thread!\n");
    while (abs(pulley2_->PosPV - (pulley2_->init_pos + kPulleysMoveUpDistance)) > 100)
    {
        delay_us(10000);
        printf("cunrrent pos2:%d\n", pulley2_->PosPV);
        printf("target pos2:%d\n", (pulley2_->init_pos + kPulleysMoveUpDistance));
    }

    pulley2_->delta_pos = pulley2_->PosPV - pulley2_->init_pos;

    // change to PPM
    ChangeToPositionMode(pulley2_);
}

// Pulleys move up
void robot::PulleysMoveUp()
{
    // move relative
    SetMotorRelPos(kPulley1, kPulleysMoveUpDistance);
    delay_us(kDelayEpos);
    SetMotorRelPos(kPulley2, kPulleysMoveUpDistance);

    // // two pulleys move up thread, this pointer!!!
    // thread pulley1_moveup_thread(&robot::Pulley1MoveUpThread, this);
    // thread pulley2_moveup_thread(&robot::Pulley2MoveUpThread, this);

    // // wait distance reached
    // pulley1_moveup_thread.join();
    // pulley2_moveup_thread.join();

    // wait pulleys reach the target pos, 1000 inc error
    while (abs(pulley1_->PosPV - (pulley1_->init_pos + kPulleysMoveUpDistance)) > 1000 || abs(pulley2_->PosPV - (pulley2_->init_pos + kPulleysMoveUpDistance)) > 1000)
    {
        delay_us(100);
    }

    // change pulleys state to stop
    pulley1_->motion_state = kStop;
    pulley2_->motion_state = kStop;
}

// master move up
void robot::MasterMoveUp()
{
    // enable motors
    MotorEnable(downclaw1_);
    MotorEnable(pulley1_);
    MotorEnable(pulley2_);

    // down claw hold
    DownClawHold();

    PulleysTorque(kPulleysTightenTorque);

    // down claw loose
    DownClawLoose();

    // change to PPM
    ChangeToPositionMode(pulley1_);
    ChangeToPositionMode(pulley2_);

    // save init pos
    pulley1_->init_pos = pulley1_->PosPV;
    pulley2_->init_pos = pulley2_->PosPV;

    // pulleys move up;
    PulleysMoveUp();
    // down claw hold
    DownClawHold();

    // must delay!
    // sleep(1);

    // // MotorDisable(kDownClaw1);

    // /* ----test---- */
    // DownClawLoose();
    // disable motors
    // MotorDisable(kPulley1);
    // MotorDisable(kPulley2);
    // sleep(1);
    // MasterMoveDown();
    // disable the debug
    robot_->debug_en = 0;
}

/* --------------pulleys move down threads----------------- */
void robot::Pulley1MoveDownThread()
{

    printf("pulley1 move down thread!\n");
    // wait pulleys reach the target pos, 1000 inc error
    while (abs(pulley1_->PosPV - (pulley1_->init_pos - kPulleysMoveDownDistance)) > 1000)
    {

        printf("init pos1:%d\n", pulley1_->init_pos);
        printf("cunrrent pos1:%d\n", pulley1_->PosPV);
        printf("target pos1:%d\n", (pulley1_->init_pos - kPulleysMoveDownDistance));

        // update the pos of last time
        pulley1_->last_pos = pulley1_->PosPV;
    }

    // update delta pos
    pulley1_->delta_pos = pulley1_->PosPV - pulley1_->init_pos;
}

void robot::Pulley2MoveDownThread()
{

    printf("pulley2 move down thread!\n");
    // wait pulleys reach the target pos, 1000 inc error
    while (abs(pulley2_->PosPV - (pulley2_->init_pos - kPulleysMoveDownDistance)) > 100)
    {

        printf("init pos2:%d\n", pulley2_->init_pos);
        printf("cunrrent pos2:%d\n", pulley2_->PosPV);
        printf("target pos2:%d\n", (pulley2_->init_pos - kPulleysMoveDownDistance));

        // update the pos of last time
        pulley2_->last_pos = pulley2_->PosPV;
    }

    pulley2_->delta_pos = pulley2_->PosPV - pulley2_->init_pos;
}

// master move down
void robot::MasterMoveDown()
{
    // enable motors
    MotorEnable(downclaw1_);
    MotorEnable(pulley1_);
    MotorEnable(pulley2_);

    // tighten pulleys
    PulleysTorque(kPulleysTightenTorque);

    // change to PPM, hold the pulleys
    ChangeToPositionMode(pulley1_);
    ChangeToPositionMode(pulley2_);

    // save init pos
    pulley1_->init_pos = pulley1_->PosPV;
    pulley2_->init_pos = pulley2_->PosPV;

    // down claw loose
    DownClawLoose();

    // move relative
    SetMotorRelPos(kPulley1, -kPulleysMoveDownDistance);
    delay_us(kDelayEpos);
    SetMotorRelPos(kPulley2, -kPulleysMoveDownDistance);

    // // two pulleys move down thread, this pointer!!!
    // thread pulley1_movedown_thread(&robot::Pulley1MoveDownThread, this);
    // thread pulley2_movedown_thread(&robot::Pulley2MoveDownThread, this);

    // // wait distance reached
    // pulley1_movedown_thread.join();
    // pulley2_movedown_thread.join();

    // wait pulleys reach the target pos, 1000 inc error
    while (abs(pulley1_->PosPV - (pulley1_->init_pos - kPulleysMoveDownDistance)) > 1000 || abs(pulley2_->PosPV - (pulley2_->init_pos - kPulleysMoveDownDistance)) > 1000)
    {
        delay_us(100);
    }

    // down claw hold
    DownClawHold();

    // disable motors
    // MotorDisable(kPulley1);
    // MotorDisable(kPulley2);
    // MotorDisable(kDownClaw1);

    // disable the debug
    robot_->debug_en = 0;
}

// pulleys move down
void robot::PulleysMoveDown()
{
}