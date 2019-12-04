#include "maxon.hpp"

maxon::maxon()
{
  // set the motor id
}
maxon::~maxon() {}

// set contrl word
int maxon::SetCtrlWrd(nmt &nmt, u16 ctrl_wrd)
{
  return nmt.TxPdo1(id_, ctrl_wrd);
}