#pragma once
#include "can.hpp"

class nmt : public can
{
private:
  /* NMT Command Specifier, sent by master to change a slave state */
  /* ------------------------------------------------------------- */
  /* Should not be modified */
  static const u16 kNMT_Start_Node = 0x01;
  static const u16 kNMT_Stop_Node = 0x02;
  static const u16 kNMT_Enter_PreOperational = 0x80;
  static const u16 kNMT_Reset_Node = 0x81;
  static const u16 kNMT_Reset_Comunication = 0x82;

  /* CANopen Function Codes */
  static const u16 kNMT = (u16)0x0 << 7;
  static const u16 kSYNC = (u16)0x1 << 7;
  static const u16 kTIME_STAMP = (u16)0x2 << 7;

  /* CANopen Function Codes */
  static const u16 kPDO1tx = (u16)0x3 << 7;
  static const u16 kPDO1rx = (u16)0x4 << 7;
  static const u16 kPDO2tx = (u16)0x5 << 7;
  static const u16 kPDO2rx = (u16)0x6 << 7;
  static const u16 kPDO3tx = (u16)0x7 << 7;
  static const u16 kPDO3rx = (u16)0x8 << 7;
  static const u16 kPDO4tx = (u16)0x9 << 7;
  static const u16 kPDO4rx = (u16)0xA << 7;
  static const u16 kSDOtx = (u16)0xB << 7;
  static const u16 kSDOrx = (u16)0xC << 7;
  static const u16 kNODE_GUARD = (u16)0xE << 7;
  static const u16 kLSS = (u16)0xF << 7;

public:
  //  frend class
  friend class maxon;
  nmt(/* args */);
  ~nmt();
  /* -------------------------NMT functions------------------ */
  void NMTstart(void);
  void NMTstart(u8 slave_id);
  void NMTPreOperation(u8 slave_id);
  void NMTstop(u8 slave_id);
  void CmdSync(void);

  // can dispatch
  void CanDisPatch(void);
};
