#pragma once

#include "maxon.hpp"
#include "freemodbus_tcp.h"

struct robot_type
{
    // system state
    __u16 system_state;

    // system operation mode select
    __u16 mode_select;

    // debug mode select
    __u16 debug_mode_select;

    // debug enable?
    __u16 debug_en;

    // claw debug factor
    __s16 upclaw_debug_factor;

    // up claw loose
    __s16 up_claw_debug_loose;

    // upwheel debug factor
    __s16 upwheel_debug_factor;

    // pulleys distance factor;
    __s16 pulleys_distance_factor;

    // pulleys homing torque
    __s16 pulleys_homing_torque;

    // pulley 1 torque
    __s16 pulley1_torque;
    __s16 pulley2_torque;

    // pulleys homing done
    __u16 pulleys_homing_done;

    // down claw 1 debug;
    __s16 down_claw_debug_loose;
    // up_claw_debug done
    __u16 up_claw_hold_done;

    /* ------------------move up motion debug-------------------------- */
    __u16 down_claw_loose_en;
};

class robot : public maxon
{
private:
    robot_type *robot_;

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

    // up claw hold motion
    static const __u16 kDownClawHoldDebug = 6;

    // homing motion debug
    static const __u16 kHomingDebug = 7;

    // master move up motion debug
    static const __u16 kMasterMoveUp = 8;

    /* debug state machine */

    /* -------------------------robot motions------------------------------ */

    //homing states
    static const __u8 kHomingIdle = 0;
    static const __u8 kHoming = 1;
    static const __u8 kHomingDone = 2;

    // robot parameters
    // tighten torque 10%
    static const __s16 kPulleysTightenTorque = 100;

    // pull torque 47%
    static const __s16 kPulleysPullTorque = 470;

    // pulleys move up distance
    static const __s32 kPulleysMoveUpDistance = 20000;

    /* -------------------------debug parameters------------------------------------ */
    // claw relative pos 100 inc
    static const __u32 kUpClawDebugRelaPos = 100;

    //upwheel relative pos 1000inc
    static const __u32 kUpWheelDebugRelaPos = 1000;
    //pulleys relative pos 1000inc
    static const __u32 kPulleysDebugRelaPos = 100;

public:
    robot(USHORT reg[]);
    ~robot();

    /* -------------------------system------------------------------ */
    void system(void);

    /* -------------------------robot control------------------------------ */
    void MasterMoveUp();

    void DownClawHold();
    void DownClawLoose();
    void PulleysTighten();
    void PulleysMoveUp();
    /* -------------------------debug function------------------------------ */
    void UpClawDebug(void);
    void UpClawHoldDebug(void);
    void UpWheelDebug(void);
    void PulleysDebug(void);
    void PulleysHomingDebug(void);

    // up claw hold debug
    void DownClawHoldDebug(void);

    // homing
    __u16 Homing(maxon_type *motor);
};
