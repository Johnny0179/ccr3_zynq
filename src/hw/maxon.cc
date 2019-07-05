#include "maxon.hpp"

maxon::maxon(/* args */)
{
    can can0;
}

maxon::~maxon()
{
}

ssize_t maxon::NMTstart(void)
{
    can_frame nmt_frame;
    // nmt frame init
    nmt_frame.can_id = kNMT;
    nmt_frame.can_dlc = 2;
    nmt_frame.data[0] = kNMT_Start_Node;
    nmt_frame.data[1] = 0;
    return can0.send(&nmt_frame);
}

ssize_t maxon::NMTstop(void)
{
    can_frame nmt_frame;
    // nmt frame init
    nmt_frame.can_id = kNMT;
    nmt_frame.can_dlc = 2;
    nmt_frame.data[0] = kNMT_Stop_Node;
    nmt_frame.data[1] = 0;
    return can0.send(&nmt_frame);
}

ssize_t maxon::TxPdo1(__u8 slave_id, __u16 ctrl_wrd)
{
    can_frame tx_pdo1_frame;

    // tx_pdo1 frame init
    tx_pdo1_frame.can_id = kPDO1rx + slave_id;
    tx_pdo1_frame.can_dlc = 2;
    tx_pdo1_frame.data[0] = ctrl_wrd & 0xff;
    tx_pdo1_frame.data[1] = (ctrl_wrd >> 8) & 0xff;

    return can0.send(&tx_pdo1_frame);
}

ssize_t maxon::SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd)
{
    return TxPdo1(slave_id, ctrl_wrd);
}


void maxon::MotorEnable(__u8 slave_id)
{
    SetCtrlWrd(slave_id, 0x0006);
    // delay 2ms
    usleep(2000);
    SetCtrlWrd(slave_id, 0x000F);
}

void maxon::StopMotor(__u8 slave_id)
{
    SetCtrlWrd(slave_id, 0x0000);
}

int maxon::CanRecv(can_frame *recv_frame){
    return can0.receive(recv_frame);
}