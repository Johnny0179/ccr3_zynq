#pragma once

#include "maxon.hpp"
#include "freemodbus_tcp.h"

class robot : public maxon
{
private:

    /* CANopen Function Codes */
    static const __u16 kNMT = (__u16)0x0 << 7;
    static const __u16 kSYNC = (__u16)0x1 << 7;
    static const __u16 kTIME_STAMP = (__u16)0x2 << 7;

    /* NMT Command Specifier, sent by master to change a slave state */
    /* ------------------------------------------------------------- */
    /* Should not be modified */
    static const __u16 kNMT_Start_Node = 0x01;
    static const __u16 kNMT_Stop_Node = 0x02;
    static const __u16 kNMT_Enter_PreOperational = 0x80;
    static const __u16 kNMT_Reset_Node = 0x81;
    static const __u16 kNMT_Reset_Comunication = 0x82;



    /* variables */

public:

    robot(USHORT reg[]);
    ~robot();

    //

    // NMT functions
    ssize_t NMTstart(void);
    ssize_t NMTstop(void);

    
};
