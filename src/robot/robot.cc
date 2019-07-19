#include "robot.hpp"

robot::robot(USHORT reg[]) : maxon()
{
    // define the pointer address
    upclaw_ = (maxon_type *)&reg[100];
    upwheel_ = (maxon_type *)&reg[150];
    downclaw1_ = (maxon_type *)&reg[200];

    robot_ = (robot_type *)&reg[0];

    // defualt debug mode
    robot_->mode_select = 1;

    // defualt debug down claw 1
    robot_->debug_mode_select = 5;

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
            robot_->pulleys_debug_factor = 0;
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
            usleep(kDelayEpos);

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

            case kUpClawMotionDebug:
                Homing(upclaw_);
                break;

            case kPulleysMotionDebug:
                PulleysDebug();
                break;

            case kDownClawHoldDebug:
                printf("Down claw debug!\n");
                DownClawHoldDebug();
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
    MoveRelative(kPulley1, kPulley2, robot_->pulleys_debug_factor * kPulleysDebugRelaPos);

    // disable the debug
    robot_->debug_en = 0;
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
    // // enable down claw 1
    // MotorEnable(kDownClaw1);

    // // increase 10000 inc
    // MoveRelative(kDownClaw1, 500000);

    // // wiat torque >20%
    // while (abs(downclaw1_->TrqPV) < 200)
    // {
    //     printf("torque: %d%\n",abs(downclaw1_->TrqPV/10));
    // }

    // disable down claw 1
    MotorDisable(kDownClaw1);

    // change to CST mode;
    SetMotorMode(kDownClaw1, 0x0A);

    // enter pre operation state
    NMTPreOperation(kDownClaw1);

    // clear past RxPDO mapping
    SdoWrU8(kDownClaw1, 0x1603, 0x00, 0);

    // first mapped object in RxPDO4 is Target Torque
    SdoWrU32(kDownClaw1, 0x1603, 0x01, 0x60710010);

    // reset mapping object number, 1
    SdoWrU8(kDownClaw1, 0x1603, 0x00, 1);

    // restart node
    NMTstart(kDownClaw1);

    MotorEnable(kDownClaw1);

    // set initial target torque 30%
    SetTargetTorque(kDownClaw1, kDownClawInitialTorque);

    // set to 20%
    while (downclaw1_->TrqPV > 0.5 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.2 * kDownClawHoldTorque);
        // usleep(kDownClawDelayUs);
        printf("torque: %d%\n",downclaw1_->TrqPV/10);
    }

    // set to 40%
    while (downclaw1_->TrqPV < 0.4 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.4 * kDownClawHoldTorque);
        // usleep(kDownClawDelayUs);
        printf("torque: %d%\n",downclaw1_->TrqPV/10);
    }

    // set to 60%
    while (downclaw1_->TrqPV < 0.6 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.6 * kDownClawHoldTorque);
        // usleep(kDownClawDelayUs);
        printf("torque: %d%\n",downclaw1_->TrqPV/10);
    }

    // set to 80%
    while (downclaw1_->TrqPV < 0.8 * kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, 0.8 * kDownClawHoldTorque);
        // usleep(kDownClawDelayUs);
        printf("torque: %d%\n",downclaw1_->TrqPV/10);
    }

    // set to 100%
    while (downclaw1_->TrqPV < kDownClawHoldTorque)
    {
        SetTargetTorque(kDownClaw1, kDownClawHoldTorque);
        // usleep(kDownClawDelayUs);
        printf("torque: %d%\n",downclaw1_->TrqPV/10);
    }

    sleep(2);
    MotorDisable(kDownClaw1);

    // remap
    TxPDO4Remap(kDownClaw1, kOBJModeOfOperation);

    // change to PPM mode;
    SetMotorMode(kDownClaw1, 0x01);

    MotorEnable(kDownClaw1);
    MoveRelative(kDownClaw1, kDownClawLooseDistance);

    // disable the debug
    robot_->debug_en = 0;

    // ccr3.MotorDisable(1);
    // ccr3.NMTstop(1);

    // // state machine parameters
    // static const __u8 kidle = 0;
    // static const __u8 kPPM = 1;
    // static const __u8 kCST = 2;
    // static const __u8 kDebugDone = 3;

    // // state machine variable
    // static __u8 state = 0;

    // while (state != kDebugDone)
    // {
    //     switch (state)
    //     {
    //     case kidle:
    //         state = kPPM;
    //         break;

    //     case kPPM:
    //         // target torque <3%
    //         if (downclaw1_->TrqPV < 30)
    //         {
    //             // enable down claw 1
    //             MotorEnable(kDownClaw1);
    //             // increase 10000 inc
    //             MoveRelative(kDownClaw1, 10000);
    //             // disable down claw 1
    //             MotorDisable(kDownClaw1);
    //         }
    //         else
    //         {
    //             // change to CST state
    //             state = kCST;
    //         }

    //     case kCST:

    //         break;

    //         break;

    //     default:
    //         break;
    //     }
    // }
}