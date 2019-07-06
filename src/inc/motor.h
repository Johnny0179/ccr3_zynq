#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "can.hpp"

struct maxon
{
    __u16 SlaveID;
    __u16 CtrlMode;
    __u16 ServSTA;
    __u16 ServErr;
    __u16 CtrlWord;
    __s16 StatusWord;
    __s32 PosSV;
    __s32 PosPV;
    __s32 PosLocked;
    __s32 PosLimit;
    __s32 SpdSV;
    __s32 SpdPV;
    __s16 MaxCurrenLimit;
    __s16 TrqPV;
    __s16 MaxcurrentLocked;
    __u16 RdUpdate;
    __u16 init_ok;
    __s32 PosPV_Last;
};

#endif