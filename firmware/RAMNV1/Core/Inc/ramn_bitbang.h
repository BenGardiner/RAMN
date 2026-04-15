/*
 * ramn_bitbang.h
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 TOYOTA MOTOR CORPORATION.
  * ALL RIGHTS RESERVED.</center></h2>
  *
  * This software component is licensed by TOYOTA MOTOR CORPORATION under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 */

// Module to experiment with bitbanging on the CAN bus

#ifndef INC_RAMN_BITBANG_H_
#define INC_RAMN_BITBANG_H_

#include "main.h"

#ifdef ENABLE_BITBANG

#include "stm32l5xx.h"

// Used to set module parameters
RAMN_Result_t RAMN_BITBANG_Set(char* param);

// Display current settings
RAMN_Result_t RAMN_BITBANG_Show(void);

// Toggle TX pins to force CAN errors (for duration of timeout)
RAMN_Result_t RAMN_BITBANG_Jam();

// Measures the current bus load (returns ratio * 100000)
uint32_t RAMN_BITBANG_BusLoad(void);

// Reads and interpret the first CAN message it reads
RAMN_Result_t RAMN_BITBANG_Read();

// Dump with simple '0' and '1'
RAMN_Result_t RAMN_BITBANG_Dump(void);

// Send provided bits (and read results)
RAMN_Result_t RAMN_BITBANG_Send(char *param);

// Repeats generation of Overload Frames
RAMN_Result_t RAMN_BITBANG_LoopOF(void);

// Sends a dominant bit after a certain trigger (to force either error or overload), while recording the bus
RAMN_Result_t RAMN_BITBANG_DenyOnce(uint8_t n);

// Same as above, but will loop and repeat (instead of recording the bus)
RAMN_Result_t RAMN_BITBANG_Deny(uint8_t n);

#if defined(ENABLE_GSUSB)
// Enter a GS_USB + bitbang bridge mode.
// CAN frames received via bitbang are delivered to the host through GS_USB.
// CAN frames sent by the host through GS_USB are transmitted via bitbang.
// When ENABLE_SUMP_OLS is also defined, PulseView's SUMP probe (0x02) is
// auto-detected: the loop pauses, enters SUMP mode to serve captured samples,
// and resumes when SUMP exits (ESC). Press ESC (outside SUMP) to exit entirely.
RAMN_Result_t RAMN_BITBANG_GsUsbLoop(void);
#endif

#if defined(ENABLE_SUMP_OLS)
// Print the SUMP compressed circular buffer as a compact VCD file on the
// serial console.  Decompresses RLE entries on the fly, emitting only
// signal changes.  The output can be saved to a .vcd file and loaded into
// PulseView.
RAMN_Result_t RAMN_BITBANG_Vcd(void);
#endif

#endif
#endif /* INC_RAMN_BITBANG_H_ */
