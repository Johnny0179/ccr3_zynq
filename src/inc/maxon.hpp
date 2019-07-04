#pragma once

#include "can.hpp"
class maxon
{
private:
    can can0;
    /* CANopen Function Codes */
    static const __u16 kNMT = (__u16)0x0 << 7;
    static const __u16 kSYNC = (__u16)0x1 << 7;
    static const __u16 kTIME_STAMP = (__u16)0x2 << 7;
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

    /* NMT Command Specifier, sent by master to change a slave state */
    /* ------------------------------------------------------------- */
    /* Should not be modified */
    static const __u16 kNMT_Start_Node = 0x01;
    static const __u16 kNMT_Stop_Node = 0x02;
    static const __u16 kNMT_Enter_PreOperational = 0x80;
    static const __u16 kNMT_Reset_Node = 0x81;
    static const __u16 kNMT_Reset_Comunication = 0x82;

    /* Motor List */

public:
    maxon(/* args */);
    ~maxon();

    ssize_t NMTstart(void);
    ssize_t NMTstop(void);

    ssize_t TxPdo1(__u8 slave_id, __u16 ctrl_wrd);

    ssize_t SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd);

    // start motor
    void MotorEnable(__u8 slave_id);
    void StopMotor(__u8 slave_id);
};
