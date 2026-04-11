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
//
// Capture happens passively during bitbang commands. When PulseView
// connects (auto-detected via the SUMP ID query byte 0x02 arriving
// as a single-byte command in slcan mode), the device enters SUMP
// mode and serves the pre-recorded samples on SUMP_RUN.

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
// Each sample is 1 byte (3 bits used: CAN_TX, CAN_RX, TRIG).
// Must be a power of 2 for efficient circular buffer masking.
#define SUMP_SAMPLE_BUFFER_SIZE  4096

// Mask for circular buffer index wrap-around (buffer_size - 1).
#define SUMP_SAMPLE_BUFFER_MASK  (SUMP_SAMPLE_BUFFER_SIZE - 1)

// Reported sample memory size. With RLE compression, we can represent
// far more samples than the physical buffer holds, so we advertise
// a larger window to PulseView (1M samples).
#define SUMP_REPORTED_SAMPLE_MEM  1048576

// Sample rate reported to PulseView. This is a nominal value;
// actual sample rate depends on the bitbang bit_quanta setting.
#define SUMP_MAX_SAMPLE_RATE     1000000

// Number of probes (channels):
//   Channel 0: CAN_TX (PB9)
//   Channel 1: CAN_RX (PB8)
//   Channel 2: TRIG   (high for one sample at the bitbang trigger point)
#define SUMP_NUM_PROBES  3

// Sample bit positions
#define SUMP_BIT_TX    0x01   // Bit 0: CAN TX
#define SUMP_BIT_RX    0x02   // Bit 1: CAN RX
#define SUMP_BIT_TRIG  0x04   // Bit 2: Trigger marker

// SUMP FLAGS register bits
#define SUMP_FLAG_RLE  0x0100   // Bit 8: Enable RLE compression

// SUMP RLE marker: MSB of the 4-byte sample group set means RLE count
#define SUMP_RLE_MARKER  0x80   // Set on byte[3] of a 4-byte RLE count word

// Special value indicating no trigger has been recorded yet
#define SUMP_NO_TRIGGER  0xFFFFFFFF

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

// ------- Shared Sample Buffer (written by bitbang, read by SUMP) -------

// Circular sample buffer populated by bitbang operations.
// Each byte: bit 0 = CAN_TX (PB9), bit 1 = CAN_RX (PB8), bit 2 = TRIG.
extern uint8_t  RAMN_SUMP_Samples[SUMP_SAMPLE_BUFFER_SIZE];

// Total number of samples written (may exceed buffer size; used for wrap detection).
extern volatile uint32_t RAMN_SUMP_SampleCount;

// Circular buffer write index (always < SUMP_SAMPLE_BUFFER_SIZE).
extern volatile uint32_t RAMN_SUMP_WriteIndex;

// Sample index (within SampleCount space) where the bitbang trigger fired.
// Set to SUMP_NO_TRIGGER before a capture; set by BB_SUMP_MARK_TRIGGER.
extern volatile uint32_t RAMN_SUMP_TriggerIndex;

// Record a TX+RX sample into the SUMP circular buffer.
// Called from bitbang ISR-disabled code.
// tx_high: 1 if TX pin is recessive, 0 if dominant
// rx_high: 1 if RX pin is recessive, 0 if dominant
static inline void RAMN_SUMP_RecordSample(uint8_t tx_high, uint8_t rx_high)
{
    uint32_t wi = RAMN_SUMP_WriteIndex;
    uint8_t sample = 0;
    if (tx_high) sample |= SUMP_BIT_TX;
    if (rx_high) sample |= SUMP_BIT_RX;
    RAMN_SUMP_Samples[wi] = sample;
    RAMN_SUMP_WriteIndex = (wi + 1) & SUMP_SAMPLE_BUFFER_MASK;
    RAMN_SUMP_SampleCount++;
}

// Mark the current position as the trigger point.
// The trigger sample gets the TRIG bit set (bit 2 = high for one sample).
static inline void RAMN_SUMP_MarkTrigger(void)
{
    // Record trigger index in the linear sample count space
    RAMN_SUMP_TriggerIndex = RAMN_SUMP_SampleCount;

    // Set the TRIG bit on the most recently written sample
    uint32_t prev = (RAMN_SUMP_WriteIndex - 1) & SUMP_SAMPLE_BUFFER_MASK;
    RAMN_SUMP_Samples[prev] |= SUMP_BIT_TRIG;
}

// Reset the sample buffer before a new bitbang operation.
static inline void RAMN_SUMP_ResetCapture(void)
{
    RAMN_SUMP_SampleCount = 0;
    RAMN_SUMP_WriteIndex = 0;
    RAMN_SUMP_TriggerIndex = SUMP_NO_TRIGGER;
}

// ------- SUMP Mode Flag -------

// True while the firmware is inside RAMN_SUMP_Enter().
// The CDC ISR checks this flag: when True, ALL received bytes are forwarded
// raw to the stream buffer (bypassing line-buffering and \r detection).
// Without this, PulseView's binary SUMP long commands (>= 0x80) get stuck
// in the CDC line buffer, and SUMP_Enter never receives them.
extern volatile RAMN_Bool_t RAMN_SUMP_Active;

// ------- Public API -------

// Enter SUMP mode. Processes SUMP binary commands until exit.
// Returns when user sends ESC (0x1B) or a reset-to-exit sequence.
void RAMN_SUMP_Enter(void);

// Process a single SUMP command byte (called from the receive loop).
// Returns True if SUMP mode should exit.
RAMN_Bool_t RAMN_SUMP_ProcessByte(uint8_t byte);

// Check if a received slcan/CLI byte indicates PulseView is connecting.
// Returns True if the byte is the SUMP_ID query (0x02), meaning
// the device should auto-enter SUMP mode.
RAMN_Bool_t RAMN_SUMP_IsSUMPProbe(const uint8_t* buffer, uint32_t length);

#endif /* ENABLE_SUMP_OLS && ENABLE_BITBANG */
#endif /* INC_RAMN_SUMP_H_ */
