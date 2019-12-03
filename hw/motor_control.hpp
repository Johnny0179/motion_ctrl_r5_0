#pragma once

/* ----------------------------motor control---------------------------------
   */
  int SetCtrlWrd(u16 ctrl_wrd);

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