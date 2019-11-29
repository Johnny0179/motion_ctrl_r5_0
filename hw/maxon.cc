#include "maxon.hpp"

maxon::maxon() {
    // init the maxon motor
}
maxon::~maxon() {}

// set contrl word
int maxon::SetCtrlWrd(nmt &nmt, u16 ctrl_wrd) {
  return nmt.TxPdo1(motor_id_, ctrl_wrd);
}