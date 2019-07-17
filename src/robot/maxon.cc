#include "maxon.hpp"

maxon::maxon()
{
}

maxon::~maxon()
{
}

// TxPDO1
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

// TxPDO2
ssize_t maxon::TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv)
{
    can_frame tx_pdo2_frame;

    // tx_pdo2 frame init
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

// TxPDO3
ssize_t maxon::TxPdo3(__u8 slave_id, __s32 speed_set)
{
    can_frame tx_pdo3_frame;

    // tx_pdo3 frame init
    tx_pdo3_frame.can_id = kPDO3rx + slave_id;
    tx_pdo3_frame.can_dlc = 4;
    tx_pdo3_frame.data[0] = speed_set & 0xff;
    tx_pdo3_frame.data[1] = (speed_set >> 8) & 0xff;
    tx_pdo3_frame.data[2] = (speed_set >> 16) & 0xff;
    tx_pdo3_frame.data[3] = (speed_set >> 24) & 0xff;
    return can0.send(&tx_pdo3_frame);
}

// TxPDO4 operation mode set
ssize_t maxon::TxPdo4(__u8 slave_id, __u16 mode_of_operation)
{
    can_frame tx_pdo4_frame;

    // tx_pdo4 frame init
    tx_pdo4_frame.can_id = kPDO4rx + slave_id;
    tx_pdo4_frame.can_dlc = 1;
    tx_pdo4_frame.data[0] = mode_of_operation;

    return can0.send(&tx_pdo4_frame);
}

// TxPDO4 CST mode
ssize_t maxon::TxPdo4(__u8 slave_id, __u16 mode_of_operation, __u16 torque_offset, __u16 target_torque)
{
    can_frame tx_pdo4_frame;

    // tx_pdo4 frame init
    tx_pdo4_frame.can_id = kPDO4rx + slave_id;

    tx_pdo4_frame.can_dlc = 5;
    tx_pdo4_frame.data[0] = mode_of_operation;
    tx_pdo4_frame.data[1] = torque_offset & 0xff;
    tx_pdo4_frame.data[2] = (torque_offset >> 8) & 0xff;
    tx_pdo4_frame.data[3] = target_torque & 0xff;
    tx_pdo4_frame.data[4] = (target_torque >> 8) & 0xff;

    return can0.send(&tx_pdo4_frame);
}

// sdo write 8bit
ssize_t maxon::SdoWrU8(__u8 slave_id, __u16 index, __u8 subindex, __u32 data)
{
    can_frame sdo_rx_frame;
    sdo_rx_frame.can_id = kSDOrx + slave_id;
    sdo_rx_frame.can_dlc = 8;
    sdo_rx_frame.data[0] = 0x2F;
    sdo_rx_frame.data[1] = index & 0xff;
    sdo_rx_frame.data[2] = (index >> 8) & 0xff;
    sdo_rx_frame.data[3] = subindex;
    sdo_rx_frame.data[4] = data & 0xff;
    sdo_rx_frame.data[5] = 0;
    sdo_rx_frame.data[6] = 0;
    sdo_rx_frame.data[7] = 0;

    return can0.send(&sdo_rx_frame);
}

// sdo write 32bit
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
    // delay 50ms
    usleep(50000);
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

ssize_t maxon::SetMotorRelPos(__u8 slave_id, __s32 relative_pos)
{
    return TxPdo2(slave_id, kServRelPosSet, relative_pos);
}

ssize_t maxon::SetMotorSpeed(__u8 slave_id, __s32 speed_set)
{
    return TxPdo3(slave_id, speed_set);
}

ssize_t maxon::SetMotorOperationMode(__u8 slave_id, __u16 mode_of_operation)
{
    return TxPdo4(slave_id, mode_of_operation);
}

// read the can frame
void maxon::CanDisPatch(void)
{

    can_frame frame;
    can_frame *recv_frame = &frame;

    can0.receive(recv_frame);

    __u16 cob_id = recv_frame->can_id & (~0x007F);
    __u16 SlaveId = (recv_frame->can_id & 0x7F);

    switch (SlaveId)
    {
    case kUpClaw:
        // get the node id
        upclaw_->motor_id = SlaveId;
        // read the parameters
        MotorParaRead(cob_id, upclaw_, recv_frame);
        break;

    case kUpWheel:
        upwheel_->motor_id = SlaveId;
        MotorParaRead(cob_id, upwheel_, recv_frame);
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
        motor->mode_display = recv_frame->data[6];
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

// move to relative position
void maxon::MoveRelative(__u8 slave_id, __s32 relative_pos)
{
    // enable claw motor
    MotorEnable(kUpWheel);

    // wait epos
    usleep(kDelayEpos);
    SetMotorRelPos(slave_id, relative_pos);
    usleep(kDelayEpos);
    SetCtrlWrd(slave_id, 0x000F);
}

// move to relative position, 2 motors
void maxon::MoveRelative(__u8 slave_id1, __u8 slave_id2, __s32 relative_pos)
{
    // enable motor1
    MotorEnable(slave_id1);

    // enable motor2
    MotorEnable(slave_id2);

    // wait epos
    usleep(kDelayEpos);
    SetMotorRelPos(slave_id1, relative_pos);
    usleep(kDelayEpos);
    SetMotorRelPos(slave_id2, relative_pos);

    usleep(kDelayEpos);
    SetCtrlWrd(slave_id1, 0x000F);
    usleep(kDelayEpos);
    SetCtrlWrd(slave_id2, 0x000F);
}

// move to absolute position
void maxon::MoveAbsolute(__u8 slave_id, __s32 absolute_pos)
{
    // wait epos
    usleep(kDelayEpos);
    SetMotorRelPos(slave_id, absolute_pos);
    usleep(kDelayEpos);
    SetCtrlWrd(slave_id, 0x000F);
}

// quick stop motor
void maxon::MotorQuickStop(__u8 slave_id)
{
    // wait epos
    usleep(kDelayEpos);
    SetCtrlWrd(slave_id, 0x000B);
}