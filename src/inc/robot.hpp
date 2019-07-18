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

    // debug_done
    __u16 debug_done;

    // debug enable?
    __u16 debug_en;

    // claw debug factor
    __s16 upclaw_debug_factor;

    // upwheel debug factor
    __s16 upwheel_debug_factor;

    // pulleys debug factor;
    __s16 pulleys_debug_factor;

    // down claw 1 debug factor;
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

    // claw motor debug
    static const __u16 kUpWheelMotorDebug = 2;

    // claw homing debug
    static const __u16 kUpClawMotionDebug = 3;

    // up wheel debug
    static const __u16 kPulleysMotionDebug = 4;

    // up claw hold motion
    static const __u16 kDownClawHoldDebug = 5;

    /* debug state machine */

    /* -------------------------robot motions------------------------------ */

    //homing states
    static const __u8 kHomingIdle = 0;
    static const __u8 kHoming = 1;
    static const __u8 kHomingDone = 2;

    /* -------------------------debug parameters------------------------------------ */
    // claw relative pos 100 inc
    static const __u32 kUpClawDebugRelaPos = 100;

    //upwheel relative pos 1000inc
    static const __u32 kUpWheelDebugRelaPos = 1000;
    //pulleys relative pos 1000inc
    static const __u32 kPulleysDebugRelaPos = 1000;

public:
    robot(USHORT reg[]);
    ~robot();

    /* -------------------------system------------------------------ */
    void system(void);

    /* -------------------------robot control------------------------------ */

    /* -------------------------debug function------------------------------ */
    void UpClawDebug(void);
    void UpWheelDebug(void);
    void PulleysDebug(void);

    // up claw hold debug
    void DownClawHoldDebug(void);

    // homing
    __u16 Homing(maxon_type *motor);
};
