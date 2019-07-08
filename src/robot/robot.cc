#include "robot.hpp"

robot::robot(USHORT reg[]) : maxon()
{

    claw=(maxon_type *)&reg[100];
}

robot::~robot()
{
}

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
