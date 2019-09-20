#pragma once

#include "can.hpp"

struct maxon_type {
  __u16 motor_id;
  __u16 mode_display;
  // __u16 ServSTA;
  __u16 ServErr;
  // __u16 CtrlWord;
  __s16 StatusWord;
  // __s32 PosSV;
  __s32 PosPV;

  // __s32 PosLimit;
  __s32 SpdSV;
  __s32 SpdPV;

  __s16 TrqPV;
  // __s16 MaxcurrentLocked;
  // __u16 RdUpdate;
  // __u16 init_ok;
  // __s32 PosPV_Last;

  /* ------------------put new variables blow this line---------------- */

  // mode of operation select
  __u16 motion_state;

  // init pos
  __s32 init_pos;

  // last time pos
  __s32 last_pos;

  // delta pos
  __s32 delta_pos;

  // home pos
  __s32 home_pos;

  // up delta pos
  __s32 up_delta_pos;

  // down delta pos
  __s32 down_delta_pos;

  // master move down init pos
  __s32 master_move_down_init_pos;

  // loose torque
  __s16 loose_torque;

  // actual average speed
  __s32 actual_average_vel;

  //   actual average troque
  __s16 actual_average_torque;
};

class maxon : public can {
 private:
  /* motor electrical
   * parameters
   */
  // nominal current 5030mA
  static const __u32 kNominalCurrent = 5030;
  // max speed 16100 rpm
  static const __u32 kMaxSpeed = 16100;

  /* mode of operation */
  // PPM
  static const __u16 kPPM = 0x01;
  // CST
  static const __u16 kCST = 0x0A;

  /* motor canopen parameters */
  static const __u32 kServOnPre = 0x0006;
  static const __u32 kServOn = 0x000F;
  static const __u32 kServOff = 0x0000;

  static const __u32 kServAbsPosSet = 0x003F;
  static const __u32 kServRelPosSet = 0x007F;
  static const __u32 kServHaltBit = 0x0100;

  /* NMT Command Specifier, sent by master to change a slave state */
  /* ------------------------------------------------------------- */
  /* Should not be modified */
  static const __u16 kNMT_Start_Node = 0x01;
  static const __u16 kNMT_Stop_Node = 0x02;
  static const __u16 kNMT_Enter_PreOperational = 0x80;
  static const __u16 kNMT_Reset_Node = 0x81;
  static const __u16 kNMT_Reset_Comunication = 0x82;

  /* CANopen Function Codes */
  static const __u16 kNMT = (__u16)0x0 << 7;
  static const __u16 kSYNC = (__u16)0x1 << 7;
  static const __u16 kTIME_STAMP = (__u16)0x2 << 7;

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

  // motors
  maxon_type *upclaw_, *upwheel_, *downclaw1_, *downclaw2_, *pulley1_,
      *pulley2_;

  // delay_time wait for epos 1000us
  static const __u32 kDelayEpos = 1000;
  /* variable */
  // Motor number
  static const __u8 kMotorNum = 6;

  // motor motion state
  static const __u8 kIdle = 0;
  static const __u8 kHold = 1;
  static const __u8 kLoose = 2;
  static const __u8 kTighten = 3;
  static const __u8 kPulling = 4;
  static const __u8 kStop = 5;

  /* Motor node id List */
  static const __u8 kUpClaw = 1;
  static const __u8 kUpWheel = 2;
  static const __u8 kPulley1 = 3;
  static const __u8 kPulley2 = 4;
  static const __u8 kDownClaw1 = 5;
  static const __u8 kDownClaw2 = 6;

  /* --------------------PDO mapping object value---------------------------- */
  // mode of operation object vlaue
  static const __u32 kOBJModeOfOperation = 0x60600008;
  // target torque object value
  static const __u32 kOBJTargetTorque = 0x60710010;

  /* -------------------debug parameters------------------------------------ */
  /* upclaw debug parameters */
  // initial torque, per thousand of “Motor rated torque
  // static const __s16 kUpClawInitialTorque = -600;

  // upclaw hold torque, 30%
  static const __s16 kUpClawHoldTorque = 300;

  // upclaw loose distance
  static const __s32 kUpClawLooseDistance = -344108;

  /* downclaw debug parameters */
  // initial torque, per thousand of “Motor rated torque
  static const __u16 kDownClawInitialTorque = 600;
  // down hold torque, 40%
  static const __u16 kDownClawHoldTorque = 400;

  static const __useconds_t kDownClawDelayUs = 500000;
  // loose distance
  static const __s32 kDownClawLooseDistance = -200000;

  // homing torque
  static const __u32 kPulleysHomingTorque = 400;

  /* -------------------------Config parameters-------------------------- */
  static const __s8 kCfgSuccess = 0;
  static const __s8 kCfgFail = -1;
  /* -------------------------NMT functions------------------------------ */
  void NMTstart(void);
  void NMTstart(__u8 slave_id);
  void NMTPreOperation(__u8 slave_id);
  void NMTstop(__u8 slave_id);
  void CmdSync();

  /* TxPDO mapping */
  void TxPDO4Remap(__u8 slave_id, __u32 object_value);

  // canopen
  ssize_t TxPdo1(__u8 slave_id, __u16 ctrl_wrd);
  ssize_t TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv,
                 __u16 mode_of_operation);
  ssize_t TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv);
  ssize_t TxPdo3(__u8 slave_id, __s32 speed_set);
  // for torque set
  ssize_t TxPdo3(__u8 slave_id, __s16 target_torque, __u16 mode_of_operation);

  // for mode set
  ssize_t TxPdo4(__u8 slave_id, __s32 speed_set, __u16 mode_of_operation);
  ssize_t TxPdo4(__u8 slave_id, __u16 mode_of_operation);
  // for CST mode
  ssize_t TxPdo4CST(__u8 slave_id, __u16 target_torque);

  // sdo 8bit write
  ssize_t SdoWrU8(__u8 slave_id, __u16 index, __u8 subindex, __u32 data);
  // sdo 16bit write
  ssize_t SdoWrU16(__u8 slave_id, __u16 index, __u8 subindex, __u32 data);
  // sdo 32bit write
  ssize_t SdoWrU32(__u8 slave_id, __u16 index, __u8 subindex, __u32 data);

  /* ----------------------------motor control---------------------------------
   */
  ssize_t SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd);

  // set absolute position
  ssize_t SetMotorAbsPos(__u8 slave_id, __s32 abs_pos);
  __s8 SetMotorAbsPos(const maxon_type *motor, __s32 abs_pos);
  __s8 SetMotorAbsPos(const maxon_type *motor1, const maxon_type *motor2,
                      __s32 abs_pos1, __s32 abs_pos2);
  __s8 SetMotorAbsPos(const maxon_type *motor1, const maxon_type *motor2,
                      const maxon_type *motor3, __s32 abs_pos1, __s32 abs_pos2,
                      __s32 abs_pos3);

  // set relative position
  ssize_t SetMotorRelPos(__u8 slave_id, __s32 relative_pos);
  __s8 SetMotorRelPos(maxon_type *motor, __s32 relative_pos);
  // set motor speed
  ssize_t SetMotorSpeed(__u8 slave_id, __s32 speed_set);
  __s8 SetMotorSpeed(const maxon_type *motor, __s32 speed);

  // set motor operation mode
  ssize_t SetMotorMode(__u8 slave_id, __u16 operation_mode);

  // enable motor
  void MotorEnable(__u8 slave_id);
  __s8 MotorEnable(const maxon_type *motor);
  // disable motor
  void MotorDisable(__u8 slave_id);
  __s8 MotorDisable(const maxon_type *motor);
  // quick stop motor
  void MotorQuickStop(__u8 slave_id);

  // change to torque mode
  __s8 ChangeToTorqueMode(const maxon_type *motor1, const maxon_type *motor2);
  void ChangeToTorqueMode(__u8 slave_id);
  __s8 ChangeToTorqueMode(const maxon_type *motor);

  // change to PPM
  void ChangeToPositionMode(__u8 slave_id);
  __s8 ChangeToPositionMode(const maxon_type *motor);
  void ChangeToPositionMode(__u8 slave_id1, __u8 slave_id2);

  // move to relative position
  void MoveRelative(__u8 slave_id, __s32 relative_pos);
  // move to relative positon 2 motors
  void MoveRelative(__u8 slave_id1, __u8 slave_id2, __s32 relative_pos);
  // move to absolute position
  void MoveAbsolute(__u8 slave_id, __s32 absolute_pos);

  // set target torque
  ssize_t SetTargetTorque(__u8 slave_id, __s16 target_torque);
  __s8 SetTargetTorque(const maxon_type *motor, __s16 target_torque);

  maxon(void);
  ~maxon();

  void MotorParaRead(__u16 cob_id, maxon_type *motor, can_frame *recv_frame);
  void CanDisPatch(void);

  // time delay
  void delay_us(__u32 us);
};
