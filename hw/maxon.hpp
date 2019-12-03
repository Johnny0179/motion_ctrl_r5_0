#pragma once

#include "nmt.hpp"
#include <vector>

using namespace std;

class maxon
{
private:
  u16 id_;
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

  // TXPDO1
  vector<u8> txpdo1;

  // TXPDO2
  vector<u8> txpdo2;

  // TXPDO3
  vector<u8> txpdo3;

  // TXPDO4
  vector<u8> txpdo4;

public:
  // be the friend class of nmt
  friend class nmt;
  maxon(void);
  ~maxon();

  /* ------------------------ TXPDO ------------------------- */
  //   txpdo1
  int TxPdo1(nmt &nmt, u16 ctrl_wrd);

  // txpdo2
  int TxPdo2(nmt &nmt, u16 ctrl_wrd, s32 pos_sv,
             u16 mode_of_operation);
  // txpdo3
  int TxPdo3(nmt &nmt, s16 target_torque,
             u16 mode_of_operation);
  // txpdo4
  int TxPdo4(nmt &nmt, s32 speed_set, u16 mode_of_operation);

  void MotorParaRead(u16 cob_id, maxon_type *motor, u32 *recv_frame);

  // time delay
  void delay_us(u32 us);
};
