#include "robot.hpp"

robot::robot(USHORT *reg)
{
    // can can0;

    // initialize the pointer
    motor[0]=(maxon *)reg;
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

ssize_t robot::TxPdo1(__u8 slave_id, __u16 ctrl_wrd)
{
    can_frame tx_pdo1_frame;

    // tx_pdo1 frame init
    tx_pdo1_frame.can_id = kPDO1rx + slave_id;
    tx_pdo1_frame.can_dlc = 2;
    tx_pdo1_frame.data[0] = ctrl_wrd & 0xff;
    tx_pdo1_frame.data[1] = (ctrl_wrd >> 8) & 0xff;

    return can0.send(&tx_pdo1_frame);
}

ssize_t robot::SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd)
{
    return TxPdo1(slave_id, ctrl_wrd);
}

// enable motor
void robot::MotorEnable(__u8 slave_id)
{
    SetCtrlWrd(slave_id, 0x0006);
    // delay 2ms
    usleep(2000);
    SetCtrlWrd(slave_id, 0x000F);
}

void robot::StopMotor(__u8 slave_id)
{
    SetCtrlWrd(slave_id, 0x0000);
}

int robot::CanRecv(can_frame *recv_frame)
{
    return can0.receive(recv_frame);
}

void robot::CanDisPatch()
{
    can_frame *recv_frame;
    
    CanRecv(recv_frame);

    __u16 cob_id = recv_frame->can_id & (~0x007F);
    __u16 SlaveId = (recv_frame->can_id & 0x7F);

    switch (cob_id)
    {
        // 0x180
    case kPDO1tx:
        motor[SlaveId]->StatusWord = (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
        motor[SlaveId]->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor[SlaveId]->PosPV = (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                                        (recv_frame->data[5] << 8) | recv_frame->data[4]);
        break;

        // 0x280
    case kPDO2tx:
        motor[SlaveId]->StatusWord = (recv_frame->data[1] << 8) | recv_frame->data[0];
        motor[SlaveId]->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor[SlaveId]->SpdPV = (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                                        (recv_frame->data[5] << 8) | recv_frame->data[4]);
        break;

        // 0x380
    case kPDO3tx:
        motor[SlaveId]->SpdPV = (__s32)((recv_frame->data[3] << 24) | recv_frame->data[2] << 16 |
                                        (recv_frame->data[1] << 8) | recv_frame->data[0]);
        motor[SlaveId]->PosPV = (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                                        (recv_frame->data[5] << 8) | recv_frame->data[4]);
        break;

        // 0x480
    case kPDO4tx:
        motor[SlaveId]->StatusWord = (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
        motor[SlaveId]->ServErr = (__u16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor[SlaveId]->TrqPV = (__s16)((recv_frame->data[5] << 8) | recv_frame->data[4]);
        motor[SlaveId]->CtrlMode = recv_frame->data[6];
        break;

        // case PDO1rx: // 0x200
        // case PDO2rx: // 0x300
        // case PDO3rx: // 0x400
        // case PDO4rx: // 0x500
        //              // ��վ���͸���վ��PDO,û�������ݲ���Ҫ����
        //              // proceedPDO(d,m);
        //     break;
        // case SDOtx: // 0x581,��վ���գ���վ���ͣ�
        //     ProcessSDOrx(m);
        //     OSSemPost(sem_SrvCAN_rx);

        // case SDOrx:          // 0x600,��վ���ͣ���վ����
        //     ProcessSDOtx(m); //�鿴Ӧ���Ƿ���ȷ����ʼ��Ҫ�����״̬.
        //     OSSemPost(sem_SrvCAN_rx);
        //     break;
        // case NODE_GUARD: // 0x700
        //     ptrServ[SlaveId]->ServSTA =
        //         (m->data[0]) & 0x7F; // 0,or 0x04(0x84),or 0x05(0x85),or 0x7F(0xFF)
        //     OSSemPost(sem_SrvCAN_rx);
        //     // proceedNODE_GUARD(d,m);
        //     break;

    default:
        break;
    }
}