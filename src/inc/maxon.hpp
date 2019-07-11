#pragma once

#include "can.hpp"

struct maxon_type
{
    __u16 motor_id;
    __u16 CtrlMode;
    // __u16 ServSTA;
    __u16 ServErr;
    // __u16 CtrlWord;
    __s16 StatusWord;
    // __s32 PosSV;
    __s32 PosPV;
    // __s32 PosLocked;
    // __s32 PosLimit;
    __s32 SpdSV;
    __s32 SpdPV;

    __s16 TrqPV;
    // __s16 MaxcurrentLocked;
    // __u16 RdUpdate;
    // __u16 init_ok;
    // __s32 PosPV_Last;

    /* ------------------put new variables blow this line---------------- */
    // __s16 MaxCurrenLimit;
};

class maxon : public can
{
private:
    /* ----------------------------motor electrical parameters--------------------------------- */
    // nominal current 5030mA
    static const __u32 kNominalCurrent = 5030;
    // max speed 16100 rpm
    static const __u32 kMaxSpeed = 16100;

    /* motor canopen parameters */
    static const __u32 kServOnPre = 0x0006;
    static const __u32 kServOn = 0x000F;
    static const __u32 kServOff = 0x0000;

    static const __u32 kServAbsPosSet = 0x003F;
    static const __u32 kServRelPosSet = 0x007F;
    static const __u32 kServHaltBit = 0x0100;

    // static const __u32 TIME_INTERVAL_US = 2000;

    /* CANopen Function Codes */
    static const __u16 kPDO1tx = (__u16)0x3 << 7;
    static const __u16 kPDO1rx = (__u16)0x4 << 7;
    static const __u16 kPDO2tx = (__u16)0x5 << 7;
    static const __u16 kPDO2rx = (__u16)0x6 << 7;
    static const __u16 kPDO3tx = (__u16)0x7 << 7;
    static const __u16 kPDO3rx = (__u16)0x8 << 7;
    static const __u16 kPDO4tx = (__u16)0x9 << 7;
    static const __u16 kPDO4rx = (__u16)0xA << 7;
    static const __u16 kSDOtx = (__u16)0xB << 7;
    static const __u16 kSDOrx = (__u16)0xC << 7;
    static const __u16 kNODE_GUARD = (__u16)0xE << 7;
    static const __u16 kLSS = (__u16)0xF << 7;



public:
    // can device
    can can0;

    /* variable */
    // Motor number
    static const __u8 kMotorNum = 2;

    /* Motor node id List */
    static const __u8 kClaw = 1;
    static const __u8 kUpWheel = 2;

    // motors
    maxon_type *claw_, *up_wheel_;

    // canopen
    ssize_t TxPdo1(__u8 slave_id, __u16 ctrl_wrd);
    ssize_t TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv);
    ssize_t TxPdo3(__u8 slave_id, __s32 speed_set);
    ssize_t TxPdo4(__u8 slave_id, __s32 max_current_limit);

    ssize_t SdoWrU32(__u8 slave_id, __u16 index, __u8 subindex, __u32 data);

    /* ----------------------------motor control--------------------------------- */
    ssize_t SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd);

    // set absolute position
    ssize_t SetMotorAbsPos(__u8 slave_id, __s32 abs_pos);

    // set relative position
    ssize_t SetMotorRelPos(__u8 slave_id, __s32 relative_pos);

    // set motor speed
    ssize_t SetMotorSpeed(__u8 slave_id, __s32 speed_set);

    // set current limit
    ssize_t SetMotorCurrentLimit(__u8 slave_id, __s32 max_current_limit);

    void MotorEnable(__u8 slave_id);
    void MotorDisable(__u8 slave_id);

    maxon(void);
    ~maxon();

    void MotorParaRead(__u16 cob_id, maxon_type *motor, can_frame *recv_frame);
    void CanDisPatch(void);
};
