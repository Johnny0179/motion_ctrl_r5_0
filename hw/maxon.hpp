#pragma once

#include "nmt.hpp"
#include <vector>

using namespace std;

class maxon
{
private:
  // motor id
  u16 id_;
  // u16 CtrlWord;
  s16 status_word_;
  // mode_display
  u16 mode_display_;
  // u16 ServSTA;
  u16 serv_err_;

  s16 TrqPV;

  s32 PosPV;

  s32 SpdPV;

  s32 SpdSV;

  // actual average speed
  s32 actual_average_vel;

  //   actual average troque
  s16 actual_average_torque;

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

  // be the friend class of can
  // friend class can;

  maxon(void);
  ~maxon();

  // motor control
  int SetCtrlWrd(nmt &nmt,u16 ctrl_wrd);

  // time delay
  void delay_us(u32 us);
};
