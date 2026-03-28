/*
 * ramn_sump.h
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

// Module implementing SUMP / OLS (Open Bench Logic Sniffer) protocol
// for monitoring CAN TX and RX pin states via PulseView / sigrok.
// Active concurrently with bitbang mode.

#ifndef INC_RAMN_SUMP_H_
#define INC_RAMN_SUMP_H_

#include "main.h"

#if defined(ENABLE_SUMP_OLS) && defined(ENABLE_BITBANG)

#include "stm32l5xx.h"

// ------- SUMP Protocol Command Bytes -------

// Short commands (1 byte, no parameters)
#define SUMP_RESET  0x00
#define SUMP_RUN    0x01
#define SUMP_ID     0x02
#define SUMP_DESC   0x04
#define SUMP_XON    0x11
#define SUMP_XOFF   0x13

// Long commands (1 byte command + 4 bytes parameters)
#define SUMP_DIV          0x80
#define SUMP_CNT          0x81
#define SUMP_FLAGS        0x82
#define SUMP_TRIG_1       0xC0
#define SUMP_TRIG_VALS_1  0xC1
#define SUMP_TRIG_2       0xC4
#define SUMP_TRIG_VALS_2  0xC5
#define SUMP_TRIG_3       0xC8
#define SUMP_TRIG_VALS_3  0xC9
#define SUMP_TRIG_4       0xCC
#define SUMP_TRIG_VALS_4  0xCD

// ------- SUMP Metadata Tags -------
#define SUMP_META_NAME         0x01
#define SUMP_META_FPGA_VER     0x02
#define SUMP_META_CPU_VER      0x03
#define SUMP_META_SAMPLE_MEM   0x21
#define SUMP_META_DYNAMIC_MEM  0x22
#define SUMP_META_SAMPLE_RATE  0x23
#define SUMP_META_PROTO_VER    0x24
#define SUMP_META_NUM_PROBES   0x40
#define SUMP_META_PROTO_VERS   0x41
#define SUMP_META_END          0x00

// ------- SUMP Capture Settings -------

// Number of samples in the circular buffer.
// Each sample is 1 byte (only 2 bits used: CAN_TX and CAN_RX).
#define SUMP_SAMPLE_BUFFER_SIZE  4096

// Maximum sample rate in Hz (80 MHz / minimum prescaler).
// Default to 1 MHz for a sensible PulseView experience.
#define SUMP_MAX_SAMPLE_RATE     1000000

// Number of probes (channels): CAN_TX = channel 0, CAN_RX = channel 1
#define SUMP_NUM_PROBES  2

// ASCII ESC character used to exit SUMP mode from a terminal
#define SUMP_EXIT_CHAR  0x1B

// Device name reported to PulseView
#define SUMP_DEVICE_NAME  "RAMN"

// ------- SUMP State Machine -------
typedef enum {
    SUMP_STATE_IDLE   = 0,
    SUMP_STATE_ARMED  = 1,
    SUMP_STATE_TRIGGED = 2
} SUMP_State_t;

// ------- SUMP Configuration -------
typedef struct {
    uint32_t divider;           // Sample rate divider
    uint32_t read_count;        // Number of samples to read
    uint32_t delay_count;       // Number of post-trigger samples
    uint32_t trigger_mask[4];   // Trigger masks (4 stages)
    uint32_t trigger_values[4]; // Trigger values (4 stages)
    uint32_t flags;             // Channel flags
    SUMP_State_t state;         // Current state machine state
} SUMP_Config_t;

// ------- Public API -------

// Enter SUMP mode. Processes SUMP binary commands until exit.
// Returns when user sends ESC (0x1B) or a reset-to-exit sequence.
void RAMN_SUMP_Enter(void);

// Process a single SUMP command byte (called from the receive loop).
// Returns True if SUMP mode should exit.
RAMN_Bool_t RAMN_SUMP_ProcessByte(uint8_t byte);

#endif /* ENABLE_SUMP_OLS && ENABLE_BITBANG */
#endif /* INC_RAMN_SUMP_H_ */
