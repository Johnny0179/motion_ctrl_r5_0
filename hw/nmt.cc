#include "nmt.hpp"

nmt::nmt(/* args */) {}

nmt::~nmt() {}

/* NMT control */
void nmt::NMTstart(void) {
  delay_us(kDelayEpos);
  u32 nmt_frame[kFramDataLength];
  // nmt frame init
  nmt_frame[0] = kNMT;            /* id */
  nmt_frame[1] = 2;               /* dlc */
  nmt_frame[2] = kNMT_Start_Node; /* data 0 */
  nmt_frame[3] = 0;
  if (send(nmt_frame) == 0) {
    printf("Start all nodes!");
  } else {
    printf("CAN communication error!\n");
  }
}

void nmt::NMTstart(u8 slave_id) {
  delay_us(kDelayEpos);
  u32 nmt_frame[kFramDataLength];
  // nmt frame init
  nmt_frame[0] = kNMT;
  nmt_frame[1] = 2;
  nmt_frame[2] = kNMT_Start_Node;
  nmt_frame[3] = slave_id;
  if (send(nmt_frame) == 0) {
    if (slave_id != 0) {
      printf("Start node%d!\n", slave_id);
    } else {
      printf("Start all nodes!\n");
    }
  } else {
    printf("CAN communication error!\n");
  }
}

void nmt::NMTPreOperation(u8 slave_id) {
  delay_us(kDelayEpos);
  u32 nmt_frame[kFramDataLength];
  // nmt frame init
  nmt_frame[0] = kNMT;
  nmt_frame[1] = 2;
  nmt_frame[2] = kNMT_Enter_PreOperational;
  nmt_frame[3] = 0;
  if (send(nmt_frame) == 0) {
    printf("Node%d enter pre operation state!\n", slave_id);
  } else {
    printf("CAN communication error!\n");
  }
}

void nmt::NMTstop(u8 slave_id) {
  u32 nmt_frame[kFramDataLength];
  // nmt frame init
  nmt_frame[0] = kNMT;
  nmt_frame[1] = 2;
  nmt_frame[2] = kNMT_Stop_Node;
  nmt_frame[3] = slave_id;
  if (send(nmt_frame) == 0) {
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
void nmt::CmdSync(void) {
  u32 nmt_frame[kFramDataLength];
  // nmt frame init
  nmt_frame[0] = kSYNC;
  nmt_frame[1] = 0;

  send(nmt_frame);
}

// TxPDO1
int nmt::TxPdo1(u8 slave_id, u16 ctrl_wrd) {
  u32 tx_pdo1_frame[kFramDataLength];

  // tx_pdo1 frame init
  tx_pdo1_frame[0] = kPDO1rx + slave_id;
  tx_pdo1_frame[1] = 2;
  tx_pdo1_frame[2] = ctrl_wrd & 0xff;
  tx_pdo1_frame[3] = (ctrl_wrd >> 8) & 0xff;

  return send(tx_pdo1_frame);
}