#include "maxon.hpp"

maxon::maxon() {}

maxon::~maxon() {}
/* -------------------------------NMT
 * control------------------------------------ */
void maxon::NMTstart(void) {
  delay_us(kDelayEpos);
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Start_Node;
  nmt_frame.data[1] = 0;
  if (can0.send(&nmt_frame) != -1) {
    printf("Start all nodes!");
  } else {
    printf("CAN communication error!\n");
  }
}

void maxon::NMTstart(__u8 slave_id) {
  delay_us(kDelayEpos);
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Start_Node;
  nmt_frame.data[1] = slave_id;
  if (can0.send(&nmt_frame) != -1) {
    if (slave_id != 0) {
      printf("Start node%d!\n", slave_id);
    } else {
      printf("Start all nodes!\n");
    }
  } else {
    printf("CAN communication error!\n");
  }
}

void maxon::NMTPreOperation(__u8 slave_id) {
  delay_us(kDelayEpos);
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Enter_PreOperational;
  nmt_frame.data[1] = 0;
  if (can0.send(&nmt_frame) != -1) {
    printf("Node%d enter pre operation state!\n", slave_id);
  } else {
    printf("CAN communication error!\n");
  }
}

void maxon::NMTstop(__u8 slave_id) {
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Stop_Node;
  nmt_frame.data[1] = slave_id;
  if (can0.send(&nmt_frame) != -1) {
    if (slave_id != 0) {
      printf("Stop node%d!\n", slave_id);
    } else {
      printf("Stop all nodes!\n");
    }
  } else {
    printf("CAN communication error!\n");
  }
}

// cmd sync
void maxon::CmdSync() {
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kSYNC;
  nmt_frame.can_dlc = 0;

  can0.send(&nmt_frame);
}

/* -------------------TxPDO mapping------------------ */
// TxPDO4 mapping
void maxon::TxPDO4Remap(__u8 slave_id, __u32 object_value) {
  // enter preoperation state
  NMTPreOperation(slave_id);
  // clear past PDO mapping
  SdoWrU8(slave_id, 0x1603, 0x00, 0);
  // first new mapped object in RxPDO4, mode of operation
  SdoWrU32(slave_id, 0x1603, 0x01, object_value);

  // reset mapping object number, 1
  SdoWrU8(slave_id, 0x1603, 0x00, 1);

  // sleep(1);
}

// TxPDO1
ssize_t maxon::TxPdo1(__u8 slave_id, __u16 ctrl_wrd) {
  can_frame tx_pdo1_frame;

  // tx_pdo1 frame init
  tx_pdo1_frame.can_id = kPDO1rx + slave_id;
  tx_pdo1_frame.can_dlc = 2;
  tx_pdo1_frame.data[0] = ctrl_wrd & 0xff;
  tx_pdo1_frame.data[1] = (ctrl_wrd >> 8) & 0xff;

  return can0.send(&tx_pdo1_frame);
}

// TxPDO2
ssize_t maxon::TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv) {
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

ssize_t maxon::TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv,
                      __u16 mode_of_operation) {
  can_frame tx_pdo2_frame;

  // tx_pdo2 frame init
  tx_pdo2_frame.can_id = kPDO2rx + slave_id;
  tx_pdo2_frame.can_dlc = 7;

  tx_pdo2_frame.data[0] = ctrl_wrd & 0xff;
  tx_pdo2_frame.data[1] = (ctrl_wrd >> 8) & 0xff;

  tx_pdo2_frame.data[2] = pos_sv & 0xff;
  tx_pdo2_frame.data[3] = (pos_sv >> 8) & 0xff;
  tx_pdo2_frame.data[4] = (pos_sv >> 16) & 0xff;
  tx_pdo2_frame.data[5] = (pos_sv >> 24) & 0xff;
  tx_pdo2_frame.data[6] = mode_of_operation;

  return can0.send(&tx_pdo2_frame);
}

// TxPDO3
ssize_t maxon::TxPdo3(__u8 slave_id, __s32 speed_set) {
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

// TxPDO3 CST mode
ssize_t maxon::TxPdo3(__u8 slave_id, __s16 target_torque,
                      __u16 mode_of_operation) {
  can_frame tx_pdo3_frame;

  // tx_pdo4 frame init
  tx_pdo3_frame.can_id = kPDO3rx + slave_id;

  tx_pdo3_frame.can_dlc = 3;
  tx_pdo3_frame.data[0] = target_torque & 0xff;
  tx_pdo3_frame.data[1] = (target_torque >> 8) & 0xff;
  tx_pdo3_frame.data[2] = mode_of_operation;

  return can0.send(&tx_pdo3_frame);
}

// TxPDO4 operation mode set
ssize_t maxon::TxPdo4(__u8 slave_id, __s32 speed_set, __u16 mode_of_operation) {
  can_frame tx_pdo4_frame;

  // tx_pdo4 frame init
  tx_pdo4_frame.can_id = kPDO4rx + slave_id;
  tx_pdo4_frame.can_dlc = 5;
  tx_pdo4_frame.data[0] = speed_set & 0xff;
  tx_pdo4_frame.data[1] = (speed_set >> 8) & 0xff;
  tx_pdo4_frame.data[2] = (speed_set >> 16) & 0xff;
  tx_pdo4_frame.data[3] = (speed_set >> 24) & 0xff;
  tx_pdo4_frame.data[4] = mode_of_operation;

  return can0.send(&tx_pdo4_frame);
}

ssize_t maxon::TxPdo4(__u8 slave_id, __u16 mode_of_operation) {
  can_frame tx_pdo4_frame;

  // tx_pdo4 frame init
  tx_pdo4_frame.can_id = kPDO4rx + slave_id;
  tx_pdo4_frame.can_dlc = 1;

  tx_pdo4_frame.data[0] = mode_of_operation;

  return can0.send(&tx_pdo4_frame);
}

// TxPDO4 CST mode
ssize_t maxon::TxPdo4CST(__u8 slave_id, __u16 target_torque) {
  can_frame tx_pdo4_frame;

  // tx_pdo4 frame init
  tx_pdo4_frame.can_id = kPDO4rx + slave_id;

  tx_pdo4_frame.can_dlc = 2;
  tx_pdo4_frame.data[0] = target_torque & 0xff;
  tx_pdo4_frame.data[1] = (target_torque >> 8) & 0xff;

  return can0.send(&tx_pdo4_frame);
}

// sdo write 8bit
ssize_t maxon::SdoWrU8(__u8 slave_id, __u16 index, __u8 subindex, __u32 data) {
  delay_us(kDelayEpos);
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

// sdo write 16bit
ssize_t maxon::SdoWrU16(__u8 slave_id, __u16 index, __u8 subindex, __u32 data) {
  delay_us(kDelayEpos);
  can_frame sdo_rx_frame;
  sdo_rx_frame.can_id = kSDOrx + slave_id;
  sdo_rx_frame.can_dlc = 8;
  sdo_rx_frame.data[0] = 0x2B;
  sdo_rx_frame.data[1] = index & 0xff;
  sdo_rx_frame.data[2] = (index >> 8) & 0xff;
  sdo_rx_frame.data[3] = subindex;
  sdo_rx_frame.data[4] = data & 0xff;
  sdo_rx_frame.data[5] = (data >> 8) & 0xff;
  sdo_rx_frame.data[6] = 0;
  sdo_rx_frame.data[7] = 0;

  return can0.send(&sdo_rx_frame);
}

// sdo write 32bit
ssize_t maxon::SdoWrU32(__u8 slave_id, __u16 index, __u8 subindex, __u32 data) {
  delay_us(kDelayEpos);
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

/*
 *Motor control
 */

ssize_t maxon::SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd) {
  return TxPdo1(slave_id, ctrl_wrd);
}

// enable motor
void maxon::MotorEnable(__u8 slave_id) {
  SetCtrlWrd(slave_id, 0x0006);
  delay_us(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000F);
  delay_us(kDelayEpos);
}

// enable motor
__s8 maxon::MotorEnable(const maxon_type *motor) {
  printf("motor id: %d\n", motor->motor_id);
  SetCtrlWrd(motor->motor_id, 0x0006);
  delay_us(kDelayEpos);
  // bit5: quick stop
  while ((motor->StatusWord >> 5 & 1) != 1) {
    // must delay!
    delay_us(kDelayEpos);
  }

  SetCtrlWrd(motor->motor_id, 0x000F);
  delay_us(kDelayEpos);
  // wait enable cmd success
  while ((motor->StatusWord & 0x00ff) != 0x37) {
    // must delay!
    delay_us(kDelayEpos);
  }
  return kCfgSuccess;
}

void maxon::MotorDisable(__u8 slave_id) { SetCtrlWrd(slave_id, 0x0000); }

__s8 maxon::MotorDisable(const maxon_type *motor) {
  SetCtrlWrd(motor->motor_id, 0x0000);
  delay_us(kDelayEpos);
  // wait disable cmd success
  while (motor->StatusWord != 0x0240) {
    delay_us(kDelayEpos);
  }

  return kCfgSuccess;
}

ssize_t maxon::SetMotorAbsPos(__u8 slave_id, __s32 abs_pos) {
  delay_us(kDelayEpos);
  return TxPdo2(slave_id, kServAbsPosSet, abs_pos);
}

__s8 maxon::SetMotorAbsPos(const maxon_type *motor, __s32 abs_pos) {
  __s32 init_cfg_pos;
  // save the init pos before configuration
  init_cfg_pos = motor->PosPV;

  if (init_cfg_pos != abs_pos) {
    // not lock the pos
    TxPdo2(motor->motor_id, kServAbsPosSet, abs_pos, 0x01);
    delay_us(kDelayEpos);
    // wait EPOS configure start
    while (motor->mode_display != 0x01 ||
           abs(motor->PosPV - init_cfg_pos) < 1000) {
      delay_us(kDelayEpos);
      TxPdo2(motor->motor_id, kServAbsPosSet, abs_pos, 0x01);
      SetCtrlWrd(motor->motor_id, 0x000F);
    }
    printf("motor%d start configuring!\n", motor->motor_id);
    // wait the abs pos reach the target, 500inc error
    while (abs(motor->PosPV - abs_pos) > 500) {
      delay_us(kDelayEpos);
      printf("init configure pos:%d\n", init_cfg_pos);
      printf("current pos:%d,target pos:%d\n", motor->PosPV, abs_pos);
    }
  } else {
    // lock the pos
    TxPdo2(motor->motor_id, kServAbsPosSet, abs_pos, 0x01);
    delay_us(kDelayEpos);
    // wait the abs pos reach the target, 1000inc error
    while (motor->mode_display != 0x01 || abs(motor->PosPV - abs_pos) > 1000) {
      delay_us(kDelayEpos);
      TxPdo2(motor->motor_id, kServAbsPosSet, abs_pos, 0x01);
      SetCtrlWrd(motor->motor_id, 0x000F);
    }
  }

  printf("motor%d reach target!\n", motor->motor_id);
  return kCfgSuccess;
}

// __s8 maxon::SetMotorAbsPos(const maxon_type *motor, __s32 abs_pos) {
//   TxPdo2(motor->motor_id, kServAbsPosSet, abs_pos, 0x01);
//   delay_us(kDelayEpos);
//   // wait the abs pos reach the target, 1000inc error
//   while (motor->mode_display != 0x01 || abs(motor->PosPV - abs_pos) > 1000) {
//     delay_us(kDelayEpos);
//     TxPdo2(motor->motor_id, kServAbsPosSet, abs_pos, 0x01);
//     SetCtrlWrd(motor->motor_id, 0x000F);
//     printf("motor%d configuring!\n", motor->motor_id);
//     printf("current pos:%d,target pos:%d\n", motor->PosPV, abs_pos);
//   }
//   printf("motor%d configure done!\n", motor->motor_id);
//   return kCfgSuccess;
// }

__s8 maxon::SetMotorAbsPos(const maxon_type *motor1, const maxon_type *motor2,
                           __s32 abs_pos1, __s32 abs_pos2) {
  TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
  TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
  delay_us(kDelayEpos);

  // wait the abs pos reach the target, 1000inc error
  while (motor1->mode_display != 0x01 || abs(motor1->PosPV - abs_pos1) > 1000 ||
         abs(motor2->PosPV - abs_pos2) > 1000 || motor2->mode_display != 0x01) {
    printf("configuring!\n");

    delay_us(kDelayEpos);
    TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
    SetCtrlWrd(motor1->motor_id, 0x000F);
    TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
    SetCtrlWrd(motor2->motor_id, 0x000F);
  }

  return kCfgSuccess;
}

__s8 maxon::SetMotorAbsPos(const maxon_type *motor1, const maxon_type *motor2,
                           const maxon_type *motor3, __s32 abs_pos1,
                           __s32 abs_pos2, __s32 abs_pos3) {
  TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
  TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
  TxPdo2(motor3->motor_id, kServAbsPosSet, abs_pos2, 0x01);
  delay_us(kDelayEpos);

  // wait the abs pos reach the target, 1000inc error
  while (motor1->mode_display != 0x01 || abs(motor1->PosPV - abs_pos1) > 1000 ||
         abs(motor2->PosPV - abs_pos2) > 1000 || motor2->mode_display != 0x01 ||
         motor3->mode_display != 0x01 || abs(motor3->PosPV - abs_pos3) > 1000) {
    printf("configuring!\n");

    delay_us(kDelayEpos);
    TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
    SetCtrlWrd(motor1->motor_id, 0x000F);
    TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
    SetCtrlWrd(motor2->motor_id, 0x000F);
    TxPdo2(motor3->motor_id, kServAbsPosSet, abs_pos3, 0x01);
    SetCtrlWrd(motor3->motor_id, 0x000F);
  }
  return kCfgSuccess;
}

ssize_t maxon::SetMotorRelPos(__u8 slave_id, __s32 relative_pos) {
  return TxPdo2(slave_id, kServRelPosSet, relative_pos);
}

__s8 maxon::SetMotorRelPos(maxon_type *motor, __s32 relative_pos) {
  // save init pos
  motor->init_pos = motor->PosPV;

  TxPdo2(motor->motor_id, kServRelPosSet, relative_pos, 0x01);
  delay_us(kDelayEpos);
  // wait the rel pos reach the target, 1000inc error
  while (motor->mode_display != 0x01 ||
         abs(motor->PosPV - (motor->init_pos + relative_pos)) > 1000) {
    motor->delta_pos = motor->PosPV - motor->init_pos;
    printf("error:%d inc, motor %d relative pos not reached!\n",
           abs(motor->PosPV - (motor->init_pos + relative_pos)),
           motor->motor_id);
    delay_us(kDelayEpos);
  }

  return kCfgSuccess;
}

ssize_t maxon::SetMotorSpeed(__u8 slave_id, __s32 speed_set) {
  return TxPdo3(slave_id, speed_set);
}

// set speed
__s8 maxon::SetMotorSpeed(const maxon_type *motor, __s32 speed) {
  // set
  TxPdo4(motor->motor_id, speed, 0x03);
  delay_us(kDelayEpos);
  // wait the speed reach 5% of the target speed
  while (motor->mode_display != 0x03 ||
         abs(motor->actual_average_vel) < abs(speed * 0.05)) {
    // printf("motor %d configuring!\n", motor->motor_id);
    // printf("current speed:%d\n", motor->SpdPV);
    // printf("target speed:%d\n", speed);
    TxPdo4(motor->motor_id, speed, 0x03);
    SetCtrlWrd(motor->motor_id, 0x000F);
    delay_us(kDelayEpos);
  }

  return kCfgSuccess;
}

ssize_t maxon::SetTargetTorque(__u8 slave_id, __s16 target_torque) {
  return TxPdo3(slave_id, target_torque, 0x0A);
}

__s8 maxon::SetTargetTorque(const maxon_type *motor, __s16 target_torque) {
  // set
  TxPdo3(motor->motor_id, target_torque, 0x0A);
  delay_us(kDelayEpos);

  // wait the toruqe reach the target, 1% error
  while (motor->mode_display != 0x0A ||
         abs(motor->actual_average_torque - target_torque) > 10) {
    TxPdo3(motor->motor_id, target_torque, 0x0A);
    delay_us(kDelayEpos);
    printf("motor%d torque:%d \n", motor->motor_id,
           motor->actual_average_torque / 10);
  }
  printf("motor%d torque configure done!\n", motor->motor_id);
  return kCfgSuccess;
}

// set motor operation mode
ssize_t maxon::SetMotorMode(__u8 slave_id, __u16 operation_mode) {
  return TxPdo4(slave_id, operation_mode);
}

// read the can frame
void maxon::CanDisPatch(void) {
  can_frame frame;
  can_frame *recv_frame = &frame;

  can0.receive(recv_frame);

  __u16 cob_id = recv_frame->can_id & (~0x007F);
  __u16 SlaveId = (recv_frame->can_id & 0x7F);

  switch (SlaveId) {
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

    case kPulley1:
      pulley1_->motor_id = SlaveId;
      MotorParaRead(cob_id, pulley1_, recv_frame);
      break;

    case kPulley2:
      pulley2_->motor_id = SlaveId;
      MotorParaRead(cob_id, pulley2_, recv_frame);
      break;

    case kDownClaw1:
      downclaw1_->motor_id = SlaveId;
      MotorParaRead(cob_id, downclaw1_, recv_frame);
      break;

    case kDownClaw2:
      downclaw2_->motor_id = SlaveId;
      MotorParaRead(cob_id, downclaw2_, recv_frame);
      break;

    default:
      break;
  }
}

void maxon::MotorParaRead(__u16 cob_id, maxon_type *motor,
                          can_frame *recv_frame) {
  switch (cob_id) {
      // 0x180
    case kPDO1tx:
      motor->StatusWord =
          (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
      motor->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
      motor->PosPV =
          (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                  (recv_frame->data[5] << 8) | recv_frame->data[4]);
      break;

      // 0x280
    case kPDO2tx:
      motor->StatusWord = (recv_frame->data[1] << 8) | recv_frame->data[0];
      motor->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
      motor->SpdPV =
          (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                  (recv_frame->data[5] << 8) | recv_frame->data[4]);
      break;

      // 0x380
    case kPDO3tx:
      motor->actual_average_vel =
          (__s32)((recv_frame->data[3] << 24) | recv_frame->data[2] << 16 |
                  (recv_frame->data[1] << 8) | recv_frame->data[0]);
      motor->actual_average_torque =
          (__s16)((recv_frame->data[5] << 8) | recv_frame->data[4]);
      break;

      // 0x480
    case kPDO4tx:
      motor->StatusWord =
          (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
      motor->ServErr =
          (__u16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
      motor->TrqPV = (__s16)((recv_frame->data[5] << 8) | recv_frame->data[4]);
      motor->mode_display = recv_frame->data[6];
      break;

    default:
      break;
  }
}

// move to relative position
void maxon::MoveRelative(__u8 slave_id, __s32 relative_pos) {
  SetMotorRelPos(slave_id, relative_pos);
  delay_us(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000F);
  delay_us(kDelayEpos);
}

// move to relative position, 2 motors
void maxon::MoveRelative(__u8 slave_id1, __u8 slave_id2, __s32 relative_pos) {
  // enable motor1
  MotorEnable(slave_id1);

  // enable motor2
  MotorEnable(slave_id2);

  SetMotorRelPos(slave_id1, relative_pos);

  SetMotorRelPos(slave_id2, relative_pos);

  SetCtrlWrd(slave_id1, 0x000F);

  SetCtrlWrd(slave_id2, 0x000F);
}

// move to absolute position
void maxon::MoveAbsolute(__u8 slave_id, __s32 absolute_pos) {
  // wait epos
  delay_us(kDelayEpos);
  SetMotorRelPos(slave_id, absolute_pos);
  delay_us(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000F);
}

// quick stop motor
void maxon::MotorQuickStop(__u8 slave_id) {
  // wait epos
  delay_us(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000B);
}

// delay_us
void maxon::delay_us(__u32 us) { usleep(us); }

// 2 motor
__s8 maxon::ChangeToTorqueMode(const maxon_type *motor1,
                               const maxon_type *motor2) {
  // disable pulleys
  MotorDisable(motor1->motor_id);
  MotorDisable(motor2->motor_id);

  // sleep(1);

  // change to CST mode
  SetMotorMode(motor1->motor_id, 0x0A);
  SetMotorMode(motor2->motor_id, 0x0A);

  // sleep(1);

  // remap pulley1 TxPDO4 to target torque
  TxPDO4Remap(motor2->motor_id, kOBJTargetTorque);
  TxPDO4Remap(motor1->motor_id, kOBJTargetTorque);

  // node enter normal mode
  NMTstart(motor1->motor_id);
  NMTstart(motor2->motor_id);

  if (motor1->mode_display == 0x0A && motor2->mode_display == 0x0A) {
    return kCfgSuccess;
  } else {
    return kCfgFail;
  }
}

// one motor
void maxon::ChangeToTorqueMode(__u8 slave_id) {
  // disable pulleys
  MotorDisable(slave_id);

  // change to CST mode
  SetMotorMode(slave_id, 0x0A);

  // remap TxPDO4 to target torque
  TxPDO4Remap(slave_id, kOBJTargetTorque);

  // node enter normal mode
  NMTstart(slave_id);
}

// change to torque mode
__s8 maxon::ChangeToTorqueMode(const maxon_type *motor) {
  // change to CST mode
  SetMotorMode(motor->motor_id, 0x0A);
  // wait for epos
  // delay_us(kDelayEpos);
  // if (motor->mode_display == 0x0A)
  // {
  //     return kCfgSuccess;
  // }
  // else
  // {
  //     return kCfgFail;
  // }
  while (motor->mode_display != 0x0A) {
    delay_us(1000);
  }
  return kCfgSuccess;
}

// one motor
void maxon::ChangeToPositionMode(__u8 slave_id) {
  // disable motor
  MotorDisable(slave_id);

  // remap TxPdo4 to mode of operation
  TxPDO4Remap(slave_id, kOBJModeOfOperation);

  // restart node
  NMTstart(slave_id);

  // change to PPM mode;
  SetMotorMode(slave_id, 0x01);
}

// change to positionmode
__s8 maxon::ChangeToPositionMode(const maxon_type *motor) {
  // change to PPM mode;
  SetMotorMode(motor->motor_id, 0x01);
  // wait for EPOS response
  // delay_us(kDelayEpos);
  // if (motor->mode_display == 0x01)
  // {
  //     return kCfgSuccess;
  // }
  // else
  // {
  //     return kCfgFail;
  // }
  while (motor->mode_display != 0x01) {
    delay_us(1000);
  }

  return kCfgSuccess;
}

// two motor
void maxon::ChangeToPositionMode(__u8 slave_id1, __u8 slave_id2) {
  // disable motor
  MotorDisable(slave_id1);
  MotorDisable(slave_id2);

  // remap TxPdo4 to mode of operation
  TxPDO4Remap(slave_id1, kOBJModeOfOperation);
  TxPDO4Remap(slave_id2, kOBJModeOfOperation);

  // restart node
  NMTstart(slave_id1);
  NMTstart(slave_id2);

  // change to PPM mode;
  SetMotorMode(slave_id1, 0x01);
  SetMotorMode(slave_id2, 0x01);
}
