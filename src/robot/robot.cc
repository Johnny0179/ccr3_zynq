#include "robot.hpp"

robot::robot(USHORT reg[]) : maxon()
{
    // define the pointer address
    upclaw_ = (maxon_type *)&reg[100];
    upwheel_ = (maxon_type *)&reg[150];

    robot_ = (robot_type *)&reg[0];

    // defualt debug mode
    robot_->mode_select = 1;
}

robot::~robot()
{
}

/* -------------------------------NMT control------------------------------------ */
ssize_t robot::NMTstart(void)
{
    can_frame nmt_frame;
    // nmt frame init
    nmt_frame.can_id = kNMT;
    nmt_frame.can_dlc = 2;
    nmt_frame.data[0] = kNMT_Start_Node;
    nmt_frame.data[1] = 0;
    return can0.send(&nmt_frame);
}

ssize_t robot::NMTstop(void)
{
    can_frame nmt_frame;
    // nmt frame init
    nmt_frame.can_id = kNMT;
    nmt_frame.can_dlc = 2;
    nmt_frame.data[0] = kNMT_Stop_Node;
    nmt_frame.data[1] = 0;
    return can0.send(&nmt_frame);
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
        }
        else if (robot_->mode_select == kDebugMode)
        {
            // start NMT
            if (NMTstart() != -1)
            {
                printf("NMT started!\n");
            }
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
            if (NMTstart() != -1)
            {
                printf("NMT started!\n");
            }
            // wait epos
            usleep(kDelayEpos);

            // choose debug mode
            switch (robot_->debug_mode_select)
            {
                //claw motor debug
            case kUpClawMotorDebug:
                UpClawDebug();
                break;

                //upwheel motor debug
            case kUpWheelMotorDebug:
                UpWheelDebug();
                break;

            case kUpClawHomingDebug:
                Homing(upclaw_);
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
            NMTstop();
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

void robot::UpWheelDebug(void)
{
    // enable claw motor
    MotorEnable(kUpWheel);

    MoveRelative(kUpWheel, robot_->upwheel_debug_factor * kUpWheelDebugRelaPos);

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
