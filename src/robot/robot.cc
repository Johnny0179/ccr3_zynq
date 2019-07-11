#include "robot.hpp"

robot::robot(USHORT reg[]) : maxon()
{
    // define the pointer address
    claw_ = (maxon_type *)&reg[100];
    up_wheel_ = (maxon_type *)&reg[150];

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
        if (robot_->mode_select == kIdleMode)
        {
            robot_->system_state = kIdleMode;
        }
        else if (robot_->mode_select == kDebugMode)
        {
            robot_->system_state = kDebugMode;
        }
        else
        {
            robot_->system_state = kNomalMode;
        }

        break;

        // debug mode
    case kDebugMode:
        if (robot_->debug_en == 0)
        {
            // return to idle
            robot_->system_state = kIdleMode;
        }
        else
        {
            switch (robot_->debug_mode_select)
            {
                //claw motor debug
            case kClawMotorDebug:
                ClawDebug();
                break;

                //upwheel motor debug
            case kUpWheelMotorDebug:
                UpWheelDebug();
                break;

            default:
                break;
            }
        }

        break;

    default:
        break;
    }
}

/* -------------------------debug function------------------------------ */
void robot::ClawDebug(void)
{
    // enable claw motor
    MotorEnable(kClaw);

    // wait epos
    usleep(kDelayEpos);
    SetMotorRelPos(kClaw, robot_->claw_debug_factor * kClawDebugRelaPos);
    usleep(kDelayEpos);
    SetCtrlWrd(kClaw, 0x000F);
}

void robot::UpWheelDebug(void)
{
    // enable claw motor
    MotorEnable(kUpWheel);

    // wait epos
    usleep(kDelayEpos);
    SetMotorRelPos(kUpWheel, robot_->upwheel_debug_factor * kUpWheelDebugRelaPos);
    usleep(kDelayEpos);
    SetCtrlWrd(kUpWheel, 0x000F);

    // disable the debug
    robot_->debug_en = 0;
}

// claw homing
__u8 robot::ClawHoming(__u16 claw_homing_en)
{
    __u8 homing_state = 0;
    switch (homing_state)
    {
    case kHomingIdle:
        if (claw_homing_en == 1)
        {
            homing_state = kHoming;
        }
        else
        {
            homing_state = kHomingIdle;
        }
        break;

    case kHoming:
        // if (claw_->TrqPV)
        // {
        //     /* code */
        // }
        break;

    case kHomingDone:

        break;

    default:
        break;
    }
}