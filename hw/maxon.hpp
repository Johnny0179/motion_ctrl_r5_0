#pragma once

#include "nmt.hpp"

class maxon {
 private:
  u16 motor_id_;
  u16 mode_display;
  // u16 ServSTA;
  u16 serv_err_;
  // u16 CtrlWord;
  s16 StatusWord;
  // s32 PosSV;
  s32 PosPV;

  // s32 PosLimit;
  s32 SpdSV;
  s32 SpdPV;

  s16 TrqPV;
  // s16 MaxcurrentLocked;
  // u16 RdUpdate;
  // u16 init_ok;
  // s32 PosPV_Last;

  /* ------------------put new variables blow this line---------------- */

  // mode of operation select
  u16 motion_state;

  // init pos
  s32 init_pos;

  // last time pos
  s32 last_pos;

  // delta pos
  s32 delta_pos;

  // home pos
  s32 home_pos;

  // up delta pos
  s32 up_delta_pos;

  // down delta pos
  s32 down_delta_pos;

  // master move down init pos
  s32 master_move_down_init_pos;

  // loose torque
  s16 loose_torque;

  // actual average speed
  s32 actual_average_vel;

  //   actual average troque
  s16 actual_average_torque;

 public:
  /* ----------------------------motor control---------------------------------
   */
  int SetCtrlWrd(nmt &nmt,u16 ctrl_wrd);

  // set absolute position
  int SetMotorAbsPos(u8 slave_id, s32 abs_pos);
  s8 SetMotorAbsPos(const maxon_type *motor, s32 abs_pos);
  s8 SetMotorAbsPos(const maxon_type *motor1, const maxon_type *motor2,
                      s32 abs_pos1, s32 abs_pos2);
  s8 SetMotorAbsPos(const maxon_type *motor1, const maxon_type *motor2,
                      const maxon_type *motor3, s32 abs_pos1, s32 abs_pos2,
                      s32 abs_pos3);

  // set relative position
  int SetMotorRelPos(u8 slave_id, s32 relative_pos);
  s8 SetMotorRelPos(maxon_type *motor, s32 relative_pos);
  // set motor speed
  int SetMotorSpeed(u8 slave_id, s32 speed_set);
  s8 SetMotorSpeed(const maxon_type *motor, s32 speed);

  // set motor operation mode
  int SetMotorMode(u8 slave_id, u16 operation_mode);

  // enable motor
  void MotorEnable(u8 slave_id);
  s8 MotorEnable(const maxon_type *motor);
  // disable motor
  void MotorDisable(u8 slave_id);
  s8 MotorDisable(const maxon_type *motor);
  // quick stop motor
  void MotorQuickStop(u8 slave_id);

  // change to torque mode
  s8 ChangeToTorqueMode(const maxon_type *motor1, const maxon_type *motor2);
  void ChangeToTorqueMode(u8 slave_id);
  s8 ChangeToTorqueMode(const maxon_type *motor);

  // change to PPM
  void ChangeToPositionMode(u8 slave_id);
  s8 ChangeToPositionMode(const maxon_type *motor);
  void ChangeToPositionMode(u8 slave_id1, u8 slave_id2);

  // move to relative position
  void MoveRelative(u8 slave_id, s32 relative_pos);
  // move to relative positon 2 motors
  void MoveRelative(u8 slave_id1, u8 slave_id2, s32 relative_pos);
  // move to absolute position
  void MoveAbsolute(u8 slave_id, s32 absolute_pos);

  // set target torque
  int SetTargetTorque(u8 slave_id, s16 target_torque);
  s8 SetTargetTorque(const maxon_type *motor, s16 target_torque);

  maxon(void);
  ~maxon();

  void MotorParaRead(u16 cob_id, maxon_type *motor, u32 *recv_frame);
  void CanDisPatch(void);

  // time delay
  void delay_us(u32 us);
};
