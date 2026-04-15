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
#define SUMP_CNT          0x81   // Combined read+delay count (original SUMP spec)
#define SUMP_FLAGS        0x82
#define SUMP_DELAYCOUNT   0x83   // Separate delay count (sigrok OLS driver)
#define SUMP_READCOUNT    0x84   // Separate read count (sigrok OLS driver)
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

// Number of bytes in the compressed circular buffer.
// Entries are either sample bytes (bit 7 = 0, bits 2:0 = TX|RX|TRIG)
// or RLE count bytes (bit 7 = 1, bits 6:0 = additional repeats 1-127).
// Must be a power of 2 for efficient circular buffer masking.
#define SUMP_SAMPLE_BUFFER_SIZE  4096

// Mask for circular buffer index wrap-around (buffer_size - 1).
#define SUMP_SAMPLE_BUFFER_MASK  (SUMP_SAMPLE_BUFFER_SIZE - 1)

// Default sample memory advertised to PulseView when no capture has
// been done yet.  With RLE the actual capacity depends on data content;
// this value is used as default read_count / delay_count.
#define SUMP_DEFAULT_SAMPLE_MEM  1048576

// Maximum additional repeats a single RLE count byte can encode (7 bits).
#define SUMP_STORAGE_RLE_MAX  127

// Bit mask to distinguish storage RLE count bytes from sample bytes.
#define SUMP_STORAGE_RLE_FLAG  0x80

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
#define SUMP_FLAG_DEMUX        0x0001   // Bit 0: Demux mode
#define SUMP_FLAG_CHANGRP_1    0x0004   // Bit 2: Disable channel group 1 (channels 0-7)
#define SUMP_FLAG_CHANGRP_2    0x0008   // Bit 3: Disable channel group 2 (channels 8-15)
#define SUMP_FLAG_CHANGRP_3    0x0010   // Bit 4: Disable channel group 3 (channels 16-23)
#define SUMP_FLAG_CHANGRP_4    0x0020   // Bit 5: Disable channel group 4 (channels 24-31)
#define SUMP_FLAG_RLE          0x0100   // Bit 8: Enable RLE compression

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
    uint8_t  sample_width;      // Bytes per sample (1-4), derived from FLAGS channel group mask
    SUMP_State_t state;         // Current state machine state
} SUMP_Config_t;

// ------- Shared Compressed Sample Buffer (written by bitbang, read by SUMP) -------
//
// The circular buffer stores RLE-compressed samples.  Each byte is either:
//   - Sample byte  (bit 7 = 0): bits 2:0 = TX | RX | TRIG
//   - RLE count    (bit 7 = 1): bits 6:0 = additional repeats (1-127)
//
// Layout invariant:  an RLE count byte is always immediately preceded by a
// sample byte for the same value, forming a "pair".  When the RLE count
// reaches 127, the next identical sample starts a new sample byte rather
// than chaining another RLE byte.  This means at most one consecutive
// RLE byte, which greatly simplifies wrap-around handling.

// Circular compressed buffer.
extern uint8_t  RAMN_SUMP_Samples[SUMP_SAMPLE_BUFFER_SIZE];

// Total logical samples recorded (monotonically increasing; may exceed buffer capacity).
extern volatile uint32_t RAMN_SUMP_SampleCount;

// Total compressed bytes written (monotonically increasing; used for wrap detection).
extern volatile uint32_t RAMN_SUMP_CompressedBytes;

// Circular buffer write index (always < SUMP_SAMPLE_BUFFER_SIZE).
extern volatile uint32_t RAMN_SUMP_WriteIndex;

// Logical sample index where the bitbang trigger fired.
// Set to SUMP_NO_TRIGGER before a capture; set by RAMN_SUMP_MarkTrigger.
extern volatile uint32_t RAMN_SUMP_TriggerIndex;

// Last sample value written (for RLE comparison).  0xFF = no previous sample.
extern volatile uint8_t  RAMN_SUMP_PrevSample;

// Actual sample rate in Hz at which the most recent capture was taken.
// Set by bitbang code before resetting the capture.
extern volatile uint32_t RAMN_SUMP_SampleRate;

// Record a TX+RX sample into the SUMP compressed circular buffer.
// Called from bitbang ISR-disabled code.
// tx_high: 1 if TX pin is recessive, 0 if dominant
// rx_high: 1 if RX pin is recessive, 0 if dominant
static inline void RAMN_SUMP_RecordSample(uint8_t tx_high, uint8_t rx_high)
{
    uint8_t sample = 0;
    if (tx_high) sample |= SUMP_BIT_TX;
    if (rx_high) sample |= SUMP_BIT_RX;

    RAMN_SUMP_SampleCount++;

    // Try to extend an existing RLE run for the same sample value.
    if (RAMN_SUMP_CompressedBytes > 0U && sample == RAMN_SUMP_PrevSample)
    {
        uint32_t prev_idx = (RAMN_SUMP_WriteIndex - 1U) & SUMP_SAMPLE_BUFFER_MASK;
        uint8_t  prev_byte = RAMN_SUMP_Samples[prev_idx];

        if (prev_byte & SUMP_STORAGE_RLE_FLAG)
        {
            // Previous byte is an RLE count — try to increment it.
            if ((prev_byte & 0x7FU) < SUMP_STORAGE_RLE_MAX)
            {
                RAMN_SUMP_Samples[prev_idx] = prev_byte + 1U;
                return;
            }
            // RLE count is full (127).  Fall through to emit a new sample byte,
            // maintaining the pair-constraint (no chained RLE bytes).
        }
        else
        {
            // Previous byte is a sample byte — start a new RLE count of 1.
            uint32_t wi = RAMN_SUMP_WriteIndex;
            RAMN_SUMP_Samples[wi] = SUMP_STORAGE_RLE_FLAG | 1U;
            RAMN_SUMP_WriteIndex = (wi + 1U) & SUMP_SAMPLE_BUFFER_MASK;
            RAMN_SUMP_CompressedBytes++;
            return;
        }
    }

    // Write a new sample byte (first sample, different value, or RLE was full).
    {
        uint32_t wi = RAMN_SUMP_WriteIndex;
        RAMN_SUMP_Samples[wi] = sample;
        RAMN_SUMP_WriteIndex = (wi + 1U) & SUMP_SAMPLE_BUFFER_MASK;
        RAMN_SUMP_PrevSample = sample;
        RAMN_SUMP_CompressedBytes++;
    }
}

// Mark the current position as the trigger point.
// The trigger sample gets the TRIG bit set (bit 2 = high for one sample).
static inline void RAMN_SUMP_MarkTrigger(void)
{
    // Nothing to mark if no samples recorded yet.
    if (RAMN_SUMP_CompressedBytes == 0U) return;

    RAMN_SUMP_TriggerIndex = RAMN_SUMP_SampleCount;

    uint32_t prev_idx = (RAMN_SUMP_WriteIndex - 1U) & SUMP_SAMPLE_BUFFER_MASK;
    uint8_t  prev_byte = RAMN_SUMP_Samples[prev_idx];

    if (prev_byte & SUMP_STORAGE_RLE_FLAG)
    {
        // Previous entry is an RLE count.  We need to "split" the last
        // repeat out and write it as a separate sample byte with TRIG.
        uint8_t count = prev_byte & 0x7FU;
        if (count > 1U)
        {
            // Decrement the RLE count (keep the rest of the run).
            RAMN_SUMP_Samples[prev_idx] = SUMP_STORAGE_RLE_FLAG | (count - 1U);
        }
        else
        {
            // Count was 1 — remove the RLE byte entirely.
            RAMN_SUMP_WriteIndex = prev_idx;
            RAMN_SUMP_CompressedBytes--;
        }

        // Write a new sample byte with the TRIG bit set.
        uint8_t trig_sample = RAMN_SUMP_PrevSample | SUMP_BIT_TRIG;
        uint32_t wi = RAMN_SUMP_WriteIndex;
        RAMN_SUMP_Samples[wi] = trig_sample;
        RAMN_SUMP_WriteIndex = (wi + 1U) & SUMP_SAMPLE_BUFFER_MASK;
        RAMN_SUMP_PrevSample = trig_sample;
        RAMN_SUMP_CompressedBytes++;
    }
    else
    {
        // Previous entry is a sample byte — just OR in the TRIG bit.
        RAMN_SUMP_Samples[prev_idx] |= SUMP_BIT_TRIG;
        RAMN_SUMP_PrevSample |= SUMP_BIT_TRIG;
    }
}

// Reset the sample buffer before a new bitbang operation.
static inline void RAMN_SUMP_ResetCapture(void)
{
    RAMN_SUMP_SampleCount    = 0;
    RAMN_SUMP_CompressedBytes = 0;
    RAMN_SUMP_WriteIndex     = 0;
    RAMN_SUMP_TriggerIndex   = SUMP_NO_TRIGGER;
    RAMN_SUMP_PrevSample     = 0xFFU;
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
