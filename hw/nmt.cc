#include "nmt.hpp"

nmt::nmt(/* args */) {}

nmt::~nmt() {}

/* NMT control */
void nmt::NMTstart(void) {
  delay_us(kDelayEpos);
  can_frame_type nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Start_Node;
  nmt_frame.data[1] = 0;
  if (send(nmt_frame) == 0) {
    printf("Start all nodes!");
  } else {
    printf("CAN communication error!\n");
  }
}

void nmt::NMTstart(u8 slave_id) {
  delay_us(kDelayEpos);
  can_frame_type nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Start_Node;
  nmt_frame.data[1] = slave_id;
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
  can_frame_type nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Enter_PreOperational;
  nmt_frame.data[1] = 0;
  if (send(nmt_frame) == 0) {
    printf("Node%d enter pre operation state!\n", slave_id);
  } else {
    printf("CAN communication error!\n");
  }
}

void nmt::NMTstop(u8 slave_id) {
  can_frame_type nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Stop_Node;
  nmt_frame.data[1] = slave_id;
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
  can_frame_type nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kSYNC;
  nmt_frame.can_dlc = 0;

  send(nmt_frame);
}

// TxPDO1
int nmt::TxPdo1(maxon &motor, u16 ctrl_wrd) {
  // u32 tx_pdo1_frame[kFramDataLength+2];

  // // tx_pdo1 frame
  // tx_pdo1_frame[0] = kPDO1rx + id_;
  // tx_pdo1_frame[1] = 2;
  // tx_pdo1_frame[2] = ctrl_wrd & 0xff;
  // tx_pdo1_frame[3] = (ctrl_wrd >> 8) & 0xff;

  // return nmt.send(tx_pdo1_frame);
}

// TxPDO2
int nmt::TxPdo2(maxon &motor, u16 ctrl_wrd, s32 pos_sv, u16 mode_of_operation) {
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

// can frame decode
void nmt::CanDecode(vector<maxon> & vec) {
  // interrupt occur
  if (can_recv_int_flag_ == 1) {
    u16 cob_id = recv_frame_.can_id & (~0x007F);
    u16 motor_id = (recv_frame_.can_id & 0x7F);

    switch (cob_id) {
        // 0x180
      case kPDO1tx:
        vec[motor_id].status_word_ =
            (u16)(recv_frame_.data[1] << 8) | recv_frame_.data[0];
        vec[motor_id].TrqPV = (s16)((recv_frame_.data[3] << 8) | recv_frame_.data[2]);
        vec[motor_id].PosPV =
            (s32)((recv_frame_.data[7] << 24) | (recv_frame_.data[6] << 16) |
                  (recv_frame_.data[5] << 8) | recv_frame_.data[4]);
        break;

        // 0x280
      case kPDO2tx:
        vec[motor_id].status_word_ = (recv_frame_.data[1] << 8) | recv_frame_.data[0];
        vec[motor_id].TrqPV = (s16)((recv_frame_.data[3] << 8) | recv_frame_.data[2]);
        vec[motor_id].SpdPV =
            (s32)((recv_frame_.data[7] << 24) | (recv_frame_.data[6] << 16) |
                  (recv_frame_.data[5] << 8) | recv_frame_.data[4]);
        break;

        // 0x380
      case kPDO3tx:
        vec[motor_id].actual_average_vel =
            (s32)((recv_frame_.data[3] << 24) | recv_frame_.data[2] << 16 |
                  (recv_frame_.data[1] << 8) | recv_frame_.data[0]);
        vec[motor_id].actual_average_torque =
            (s16)((recv_frame_.data[5] << 8) | recv_frame_.data[4]);
        break;

        // 0x480
      case kPDO4tx:
        vec[motor_id].status_word_ =
            (u16)(recv_frame_.data[1] << 8) | recv_frame_.data[0];
        vec[motor_id].serv_err_ =
            (u16)((recv_frame_.data[3] << 8) | recv_frame_.data[2]);
        vec[motor_id].TrqPV = (s16)((recv_frame_.data[5] << 8) | recv_frame_.data[4]);
        vec[motor_id].mode_display_ = recv_frame_.data[6];
        break;

      default:
        break;
    }
  }
  
}
