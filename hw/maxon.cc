#include "maxon.hpp"

maxon::maxon()
{
  // init the maxon motor
}
maxon::~maxon() {}

// set contrl word
int maxon::SetCtrlWrd(u16 ctrl_wrd)
{
}

// TxPDO1
int maxon::TxPdo1(nmt &nmt, u16 ctrl_wrd)
{
  // u32 tx_pdo1_frame[kFramDataLength+2];

  // // tx_pdo1 frame
  // tx_pdo1_frame[0] = kPDO1rx + id_;
  // tx_pdo1_frame[1] = 2;
  // tx_pdo1_frame[2] = ctrl_wrd & 0xff;
  // tx_pdo1_frame[3] = (ctrl_wrd >> 8) & 0xff;

  // return nmt.send(tx_pdo1_frame);
}

// TxPDO2
int maxon::TxPdo2(nmt &nmt, u16 ctrl_wrd, s32 pos_sv,
                  u16 mode_of_operation)
{
  // u32 tx_pdo2_frame[kFramDataLength+2];
  //   // tx_pdo2 frame
  // tx_pdo2_frame.can_id = kPDO2rx + id_;
  // tx_pdo2_frame.can_dlc = 7;

  // tx_pdo2_frame.data[0] = ctrl_wrd & 0xff;
  // tx_pdo2_frame.data[1] = (ctrl_wrd >> 8) & 0xff;

  // tx_pdo2_frame.data[2] = pos_sv & 0xff;
  // tx_pdo2_frame.data[3] = (pos_sv >> 8) & 0xff;
  // tx_pdo2_frame.data[4] = (pos_sv >> 16) & 0xff;
  // tx_pdo2_frame.data[5] = (pos_sv >> 24) & 0xff;
  // tx_pdo2_frame.data[6] = mode_of_operation;
  // return nmt.send(tx_pdo2_frame);
}