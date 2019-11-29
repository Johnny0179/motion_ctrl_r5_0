#pragma once

#include "xcanps.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xstatus.h"

class can {
 private:
  // Interrupt controller
  // static const u32 kIntCtrlr=XScuGic;

  /* CAN Device parapmeters */
  // CAN device id
  static const u32 kCANDeviceId = XPAR_XCANPS_0_DEVICE_ID;

  // INT Controller device id
  static const u32 kIntcDeviceId = XPAR_SCUGIC_SINGLE_DEVICE_ID;

  // CAN INT vector id
  static const u32 kCANIntVecId = XPAR_XCANPS_0_INTR;

  // Maximum CAN frame length in word
  static const u32 kMaxFrameSizeInWords = (XCANPS_MAX_FRAME_SIZE / sizeof(u32));

  // Frame Data field length
  static const u32 kFramDataLength = 8;

  // message id constant
  static const u32 kTestMessageId = 2000;

  /*
   * The Baud Rate Prescaler Register (BRPR) and Bit Timing Register (BTR)
   * are setup such that CAN baud rate equals 40Kbps, assuming that the
   * the CAN clock is 24MHz. The user needs to modify these values based on
   * the desired baud rate and the CAN clock frequency. For more information
   * see the CAN 2.0A, CAN 2.0B, ISO 11898-1 specifications.
   */

  /*
   * Timing parameters to be set in the Bit Timing Register (BTR).
   * These values are for a 40 Kbps baudrate assuming the CAN input clock
   * frequency is 24 MHz.
   */
  static const u32 kTestBtrSyncjumpwith = 3;
  static const u32 kTestBtrSecondTimeSegment = 2;
  static const u32 kTestBtrFirstTimeSegment = 15;

  /*
   * The Baud rate Prescalar value in the Baud Rate Prescaler Register
   * needs to be set based on the input clock  frequency to the CAN core and
   * the desired CAN baud rate.
   * This value is for a 40 Kbps baudrate assuming the CAN input clock frequency
   * is 24 MHz.
   */
  static const u32 kTestBrprBaudPrescalar = 29;

  /* Variable Definitions */
  //   Instance of the Can driver
  XCanPs can_instance_;
  // Instance of the Interrupt Controller driver
  XScuGic intc_instance_;

  // Frame was sent successfully
  volatile int send_done_;
  // Asynchronous error occurred
  volatile int loopback_error_;
  // Received a frame
  volatile int recv_done_;

  // send frame
  u32 send_frame_[kFramDataLength];
  // recv_frame;
  u32 recev_frame_[kFramDataLength];

  // CAN filter number, 8 canopen devices, each has 4 TxPDO.
  static const u8 kNodeNum = 6;

  static const int kCanFilterNum = 24;

 public:
  can(/* args */);
  ~can();
  int CanInit(XScuGic *IntcInstPtr, XCanPs *CanInstPtr, u16 CanDeviceId,
              u16 CanIntrId);

  void Config(XCanPs *InstancePtr);
  int SendFrame(XCanPs *InstancePtr, u32 *TxFrame);
  int send(u32 *TxFrame);

  void SendHandler(void *CallBackRef);
  void RecvHandler(void *CallBackRef);
  void ErrorHandler(void *CallBackRef, u32 ErrorMask);
  void EventHandler(void *CallBackRef, u32 Mask);

  int SetupInterruptSystem(XScuGic *IntcInstancePtr, XCanPs *CanInstancePtr,
                           u16 CanIntrId);
};