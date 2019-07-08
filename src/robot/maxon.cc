#include "maxon.hpp"

maxon::maxon()
{
}

maxon::~maxon()
{
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

ssize_t maxon::TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv)
{
    can_frame tx_pdo2_frame;

    // tx_pdo1 frame init
    tx_pdo2_frame.can_id = kPDO2rx + slave_id;
    tx_pdo2_frame.can_dlc = 6;

    tx_pdo2_frame.data[0] = ctrl_wrd & 0xff;
    tx_pdo2_frame.data[1] = (ctrl_wrd >> 8) & 0xff;

    tx_pdo2_frame.data[2] = pos_sv & 0xff;
    tx_pdo2_frame.data[3] = (pos_sv >> 8) & 0xff;
    tx_pdo2_frame.data[4] = (pos_sv >> 16) & 0xff;
    tx_pdo2_frame.data[5] = (pos_sv >> 24) & 0xff;

    return can0.send(&tx_pdo2_frame);
}

ssize_t maxon::SdoWrU32(__u8 slave_id, __u16 index, __u8 subindex, __u32 data)
{
    can_frame sdo_rx_frame;
    sdo_rx_frame.can_id = kSDOrx + slave_id;
    sdo_rx_frame.can_dlc = 8;
    sdo_rx_frame.data[0] = 0x23;
    sdo_rx_frame.data[1] = index & 0xff;
    sdo_rx_frame.data[2] = (index >> 8) & 0xff;
    sdo_rx_frame.data[3] = subindex;
    sdo_rx_frame.data[4] = data & 0xff;
    sdo_rx_frame.data[5] = (data >> 8) & 0xff;
    sdo_rx_frame.data[6] = (data >> 16) & 0xff;
    sdo_rx_frame.data[7] = (data >> 24) & 0xff;

    return can0.send(&sdo_rx_frame);
}

/* ---------------------------Motor control----------------------------------- */

ssize_t maxon::SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd)
{
    return TxPdo1(slave_id, ctrl_wrd);
}

// enable motor
void maxon::MotorEnable(__u8 slave_id)
{
    SetCtrlWrd(slave_id, 0x0006);
    // delay 2ms
    usleep(2000);
    SetCtrlWrd(slave_id, 0x000F);
}

void maxon::MotorDisable(__u8 slave_id)
{
    SetCtrlWrd(slave_id, 0x0000);
}

ssize_t maxon::SetMotorAbsPos(__u8 slave_id, __s32 abs_pos)
{
    return TxPdo2(slave_id, kServAbsPosSet, abs_pos);
}

void maxon::CanDisPatch(void)
{

    can_frame frame;
    can_frame *recv_frame = &frame;

    can0.receive(recv_frame);

    __u16 cob_id = recv_frame->can_id & (~0x007F);
    __u16 SlaveId = (recv_frame->can_id & 0x7F);

    switch (SlaveId)
    {
    case kClaw:
        MotorParaRead(cob_id, claw, recv_frame);
        break;

    default:
        break;
    }
}

void maxon::MotorParaRead(__u16 cob_id, maxon_type *motor, can_frame *recv_frame)
{
    switch (cob_id)
    {
        // 0x180
    case kPDO1tx:
        motor->StatusWord = (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
        motor->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor->PosPV = (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                               (recv_frame->data[5] << 8) | recv_frame->data[4]);
        break;

        // 0x280
    case kPDO2tx:
        motor->StatusWord = (recv_frame->data[1] << 8) | recv_frame->data[0];
        motor->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor->SpdPV = (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                               (recv_frame->data[5] << 8) | recv_frame->data[4]);
        break;

        // 0x380
    case kPDO3tx:
        motor->SpdPV = (__s32)((recv_frame->data[3] << 24) | recv_frame->data[2] << 16 |
                               (recv_frame->data[1] << 8) | recv_frame->data[0]);
        motor->PosPV = (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                               (recv_frame->data[5] << 8) | recv_frame->data[4]);
        break;

        // 0x480
    case kPDO4tx:
        motor->StatusWord = (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
        motor->ServErr = (__u16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor->TrqPV = (__s16)((recv_frame->data[5] << 8) | recv_frame->data[4]);
        motor->CtrlMode = recv_frame->data[6];
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
        //     ptrServ->ServSTA =
        //         (m->data[0]) & 0x7F; // 0,or 0x04(0x84),or 0x05(0x85),or 0x7F(0xFF)
        //     OSSemPost(sem_SrvCAN_rx);
        //     // proceedNODE_GUARD(d,m);
        //     break;

    default:
        break;
    }
}
