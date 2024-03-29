#include "can.hpp"

can::can(/* args */) {}

can::~can() {}

int can::CanInit(XScuGic *IntcInstPtr, XCanPs *CanInstPtr, u16 CanDeviceId,
                 u16 CanIntrId) {
  int Status;
  XCanPs_Config *ConfigPtr;
  /*
   * Initialize the Can device.
   */
  ConfigPtr = XCanPs_LookupConfig(CanDeviceId);
  if (ConfigPtr == NULL) {
    return XST_FAILURE;
  }
  XCanPs_CfgInitialize(CanInstPtr, ConfigPtr, ConfigPtr->BaseAddr);

  /*
   * Run self-test on the device, which verifies basic sanity of the
   * device and the driver.
   */
  Status = XCanPs_SelfTest(CanInstPtr);
  if (Status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  /*
   * Configure CAN device.
   */
  Config(CanInstPtr);

  /*
   * Set interrupt handlers.
   */
  XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_SEND, (void *)SendHandler,
                    (void *)CanInstPtr);
  XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_RECV, (void *)RecvHandler,
                    (void *)CanInstPtr);
  XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_ERROR, (void *)ErrorHandler,
                    (void *)CanInstPtr);
  XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_EVENT, (void *)EventHandler,
                    (void *)CanInstPtr);

  /*
   * Initialize the flags.
   */
  send_done_ = FALSE;
  recv_done_ = FALSE;
  loopback_error_ = FALSE;

  /*
   * Connect to the interrupt controller.
   */
  Status = SetupInterruptSystem(IntcInstPtr, CanInstPtr, CanIntrId);
  if (Status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  /*
   * Enable all interrupts in CAN device.
   */
  XCanPs_IntrEnable(CanInstPtr, XCANPS_IXR_ALL);

  /*
   * Enter Normal Mode.
   */
  XCanPs_EnterMode(CanInstPtr, XCANPS_MODE_NORMAL);
  while (XCanPs_GetMode(CanInstPtr) != XCANPS_MODE_NORMAL)
    ;

  //   /*
  //    * Loop back a frame. The RecvHandler is expected to handle
  //    * the frame reception.
  //    */
  //   SendFrame(CanInstPtr); /* Send a frame */

  //   /*
  //    * Wait here until both sending and reception have been completed.
  //    */
  //   while ((send_done_ != TRUE) || (recv_done_ != TRUE))
  //     ;
}

/*****************************************************************************/
/**
 *
 * This function configures CAN device. Baud Rate Prescaler Register (BRPR) and
 * Bit Timing Register (BTR) are set in this function.
 *
 * @param	InstancePtr is a pointer to the driver instance.
 *
 * @return	None.
 *
 * @note		If the CAN device is not working correctly, this
 *function may enter an infinite loop and will never return to the caller.
 *
 ******************************************************************************/
void can::Config(XCanPs *InstancePtr) {
  /*
   * Enter Configuration Mode if the device is not currently in
   * Configuration Mode.
   */
  XCanPs_EnterMode(InstancePtr, XCANPS_MODE_CONFIG);
  while (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG)
    ;

  /*
   * Setup Baud Rate Prescaler Register (BRPR) and
   * Bit Timing Register (BTR).
   */
  XCanPs_SetBaudRatePrescaler(InstancePtr, kTestBrprBaudPrescalar);
  XCanPs_SetBitTiming(InstancePtr, kTestBtrSyncjumpwith,
                      kTestBtrSecondTimeSegment, kTestBtrFirstTimeSegment);
}

int can::SendFrame(XCanPs *InstancePtr, u32 *TxFrame) {
  // u8 *FramePtr;
  // int Index;
  // int Status;

  /*
   * Create correct values for Identifier and Data Length Code Register.
   */
  // TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)TEST_MESSAGE_ID, 0, 0, 0, 0);
  // TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)FRAME_DATA_LENGTH);

  /*
   * Now fill in the data field with known values so we can verify them
   * on receive.
   */
  // FramePtr = (u8 *)(&TxFrame[2]);
  // for (Index = 0; Index < FRAME_DATA_LENGTH; Index++) {
  // 	*FramePtr++ = (u8)Index;
  // }

  /*
   * Now wait until the TX FIFO is not full and send the frame.
   */
  while (XCanPs_IsTxFifoFull(InstancePtr) == TRUE)
    ;

  return XCanPs_Send(InstancePtr, TxFrame);
}

int can::send(can_frame_type &send_frame) {
  u32 tx_frame[4];
  u8 *frame_ptr;
  int i;

  tx_frame[0] = send_frame.can_id;
  tx_frame[1] = send_frame.can_dlc;
  frame_ptr = (u8 *)(&tx_frame[2]);

  for (i = 0; i < send_frame.can_dlc; i++) {
    *frame_ptr = send_frame.data[i];
    frame_ptr++;
  }

  return SendFrame(&can_instance_, tx_frame);
}

/*****************************************************************************/
/**
 *
 * Callback function (called from interrupt handler) to handle confirmation of
 * transmit events when in interrupt mode.
 *
 * @param	CallBackRef is the callback reference passed from the interrupt
 *		handler, which in our case is a pointer to the driver instance.
 *
 * @return	None.
 *
 * @note		This function is called by the driver within interrupt
 *context.
 *
 ******************************************************************************/
void can::SendHandler(void *CallBackRef) {
  /*
   * The frame was sent successfully. Notify the task context.
   */
  send_done_ = TRUE;
}

/*****************************************************************************/
/**
 *
 * Callback function (called from interrupt handler) to handle frames received
 *in interrupt mode.  This function is called once per frame received. The
 *driver's receive function is called to read the frame from RX FIFO.
 *
 * @param	CallBackRef is the callback reference passed from the interrupt
 *		handler, which in our case is a pointer to the device instance.
 *
 * @return	None.
 *
 * @note		This function is called by the driver within interrupt
 *context.
 *
 ******************************************************************************/
void can::RecvHandler(void *CallBackRef) {
  XCanPs *CanPtr = (XCanPs *)CallBackRef;
  int Status;
  u32 rx_frame[4];
  u8 *frame_ptr;
  int i;

  // CAN receive interrupt
  can_recv_int_flag_ = 1;

  Status = XCanPs_Recv(CanPtr, rx_frame);

  recv_frame_.can_id = rx_frame[0];
  recv_frame_.can_dlc = rx_frame[1];
  frame_ptr = (u8 *)(&rx_frame[2]);

  for (i = 0; i < recv_frame_.can_dlc; i++) {
    recv_frame_.data[i] = *frame_ptr;
    frame_ptr++;
  }

  // CAN decode
  

  // if (Status != XST_SUCCESS) {
  //   loopback_error_ = TRUE;
  //   recv_done_ = TRUE;
  //   return;
  // }

  /*
   * Verify Identifier and Data Length Code.
   */
  // if (RxFrame[0] !=
  //     (u32)XCanPs_CreateIdValue((u32)TEST_MESSAGE_ID, 0, 0, 0, 0)) {
  //   loopback_error_ = TRUE;
  //   recv_done_ = TRUE;
  //   return;
  // }

  // if ((RxFrame[1] & ~XCANPS_DLCR_TIMESTAMP_MASK) != TxFrame[1]) {
  //   loopback_error_ = TRUE;
  //   recv_done_ = TRUE;
  //   return;
  // }

  /*
   * Verify the Data field contents.
   */
  // FramePtr = (u8 *)(&RxFrame[2]);
  // for (Index = 0; Index < FRAME_DATA_LENGTH; Index++) {
  //   if (*FramePtr++ != (u8)Index) {
  //     loopback_error_ = TRUE;
  //     break;
  //   }
  // }
  if (Status != XST_SUCCESS) {
    recv_done_ = TRUE;
  }
}

/*****************************************************************************/
/**
 *
 * Callback function (called from interrupt handler) to handle error interrupt.
 * Error code read from Error Status register is passed into this function.
 *
 * @param	CallBackRef is the callback reference passed from the interrupt
 *		handler, which in our case is a pointer to the driver instance.
 * @param	ErrorMask is a bit mask indicating the cause of the error.
 *		Its value equals 'OR'ing one or more XCANPS_ESR_* defined in
 *		xcanps_hw.h.
 *
 * @return	None.
 *
 * @note		This function is called by the driver within interrupt
 *context.
 *
 ******************************************************************************/
void can::ErrorHandler(void *CallBackRef, u32 ErrorMask) {
  if (ErrorMask & XCANPS_ESR_ACKER_MASK) {
    /*
     * ACK Error handling code should be put here.
     */
  }

  if (ErrorMask & XCANPS_ESR_BERR_MASK) {
    /*
     * Bit Error handling code should be put here.
     */
  }

  if (ErrorMask & XCANPS_ESR_STER_MASK) {
    /*
     * Stuff Error handling code should be put here.
     */
  }

  if (ErrorMask & XCANPS_ESR_FMER_MASK) {
    /*
     * Form Error handling code should be put here.
     */
  }

  if (ErrorMask & XCANPS_ESR_CRCER_MASK) {
    /*
     * CRC Error handling code should be put here.
     */
  }

  /*
   * Set the shared variables.
   */
  loopback_error_ = TRUE;
  recv_done_ = TRUE;
  send_done_ = TRUE;
}

/*****************************************************************************/
/**
 *
 * Callback function (called from interrupt handler) to handle the following
 * interrupts:
 *   - XCANPS_IXR_BSOFF_MASK:	Bus Off Interrupt
 *   - XCANPS_IXR_RXOFLW_MASK:	RX FIFO Overflow Interrupt
 *   - XCANPS_IXR_RXUFLW_MASK:	RX FIFO Underflow Interrupt
 *   - XCANPS_IXR_TXBFLL_MASK:	TX High Priority Buffer Full Interrupt
 *   - XCANPS_IXR_TXFLL_MASK:	TX FIFO Full Interrupt
 *   - XCANPS_IXR_WKUP_MASK:	Wake up Interrupt
 *   - XCANPS_IXR_SLP_MASK:	Sleep Interrupt
 *   - XCANPS_IXR_ARBLST_MASK:	Arbitration Lost Interrupt
 *
 *
 * @param	CallBackRef is the callback reference passed from the
 *		interrupt Handler, which in our case is a pointer to the
 *		driver instance.
 * @param	IntrMask is a bit mask indicating pending interrupts.
 *		Its value equals 'OR'ing one or more of the XCANPS_IXR_*_MASK
 *		value(s) mentioned above.
 *
 * @return	None.
 *
 * @note		This function is called by the driver within interrupt
 *context. This function should be changed to meet specific application needs.
 *
 ******************************************************************************/
void can::EventHandler(void *CallBackRef, u32 IntrMask) {
  XCanPs *CanPtr = (XCanPs *)CallBackRef;

  if (IntrMask & XCANPS_IXR_BSOFF_MASK) {
    /*
     * Entering Bus off status interrupt requires
     * the CAN device be reset and reconfigured.
     */
    XCanPs_Reset(CanPtr);
    Config(CanPtr);
    return;
  }

  if (IntrMask & XCANPS_IXR_RXOFLW_MASK) {
    /*
     * Code to handle RX FIFO Overflow Interrupt should be put here.
     */
  }

  if (IntrMask & XCANPS_IXR_RXUFLW_MASK) {
    /*
     * Code to handle RX FIFO Underflow Interrupt
     * should be put here.
     */
  }

  if (IntrMask & XCANPS_IXR_TXBFLL_MASK) {
    /*
     * Code to handle TX High Priority Buffer Full
     * Interrupt should be put here.
     */
  }

  if (IntrMask & XCANPS_IXR_TXFLL_MASK) {
    /*
     * Code to handle TX FIFO Full Interrupt should be put here.
     */
  }

  if (IntrMask & XCANPS_IXR_WKUP_MASK) {
    /*
     * Code to handle Wake up from sleep mode Interrupt
     * should be put here.
     */
  }

  if (IntrMask & XCANPS_IXR_SLP_MASK) {
    /*
     * Code to handle Enter sleep mode Interrupt should be put here.
     */
  }

  if (IntrMask & XCANPS_IXR_ARBLST_MASK) {
    /*
     * Code to handle Lost bus arbitration Interrupt
     * should be put here.
     */
  }
}

/*****************************************************************************/
/**
 *
 * This function sets up the interrupt system so interrupts can occur for the
 * CAN. This function is application-specific since the actual system may or
 * may not have an interrupt controller. The CAN could be directly connected
 * to a processor without an interrupt controller. The user should modify this
 * function to fit the application.
 *
 * @param	IntcInstancePtr is a pointer to the instance of the ScuGic.
 * @param	CanInstancePtr contains a pointer to the instance of the CAN
 *		which is going to be connected to the interrupt
 *		controller.
 * @param	CanIntrId is the interrupt Id and is typically
 *		XPAR_<CANPS_instance>_INTR value from xparameters.h.
 *
 * @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
 *
 * @note		None.
 *
 ****************************************************************************/
int can::SetupInterruptSystem(XScuGic *IntcInstancePtr, XCanPs *CanInstancePtr,
                              u16 CanIntrId) {
  int Status;
#ifdef XPAR_INTC_0_DEVICE_ID
#ifndef TESTAPP_GEN
  /* Initialize the interrupt controller and connect the ISRs */
  Status = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
  if (Status != XST_SUCCESS) {
    xil_printf("Failed init intc\r\n");
    return XST_FAILURE;
  }
#endif
  /*
   * Connect the driver interrupt handler
   */
  Status = XIntc_Connect(IntcInstancePtr, CanIntrId,
                         (XInterruptHandler)XCanPs_IntrHandler, CanInstancePtr);
  if (Status != XST_SUCCESS) {
    xil_printf("Failed connect intc\r\n");
    return XST_FAILURE;
  }
#ifndef TESTAPP_GEN
  /*
   * Start the interrupt controller such that interrupts are enabled for
   * all devices that cause interrupts.
   */
  Status = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
  if (Status != XST_SUCCESS) {
    return XST_FAILURE;
  }
#endif

  /*
   * Enable the interrupt for the CAN device.
   */
  XIntc_Enable(IntcInstancePtr, CanIntrId);
#ifndef TESTAPP_GEN
  Xil_ExceptionInit();
  Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                               (Xil_ExceptionHandler)XIntc_InterruptHandler,
                               (void *)IntcInstancePtr);
#endif
#else
#ifndef TESTAPP_GEN
  XScuGic_Config *IntcConfig; /* Instance of the interrupt controller */

  Xil_ExceptionInit();

  /*
   * Initialize the interrupt controller driver so that it is ready to
   * use.
   */
  IntcConfig = XScuGic_LookupConfig(kIntcDeviceId);
  if (NULL == IntcConfig) {
    return XST_FAILURE;
  }

  Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
                                 IntcConfig->CpuBaseAddress);
  if (Status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  /*
   * Connect the interrupt controller interrupt handler to the hardware
   * interrupt handling logic in the processor.
   */
  Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                               (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                               IntcInstancePtr);
#endif

  /*
   * Connect the device driver handler that will be called when an
   * interrupt for the device occurs, the handler defined above performs
   * the specific interrupt processing for the device.
   */
  Status = XScuGic_Connect(IntcInstancePtr, CanIntrId,
                           (Xil_InterruptHandler)XCanPs_IntrHandler,
                           (void *)CanInstancePtr);
  if (Status != XST_SUCCESS) {
    return Status;
  }

  /*
   * Enable the interrupt for the CAN device.
   */
  XScuGic_Enable(IntcInstancePtr, CanIntrId);
#endif
#ifndef TESTAPP_GEN
  /*
   * Enable interrupts in the Processor.
   */
  Xil_ExceptionEnable();
#endif
  return XST_SUCCESS;
}
