/*
 * ramn_sump.c
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

#include "ramn_sump.h"

#if defined(ENABLE_SUMP_OLS) && defined(ENABLE_BITBANG)

#include "ramn_usb.h"
#include "cmsis_os.h"
#include "stream_buffer.h"

// Shared sample buffer: circular, written by bitbang operations, read by SUMP protocol.
uint8_t  RAMN_SUMP_Samples[SUMP_SAMPLE_BUFFER_SIZE];
volatile uint32_t RAMN_SUMP_SampleCount = 0;
volatile uint32_t RAMN_SUMP_WriteIndex = 0;
volatile uint32_t RAMN_SUMP_TriggerIndex = SUMP_NO_TRIGGER;

// SUMP mode flag: True while inside RAMN_SUMP_Enter().
// Checked by the CDC ISR to forward all bytes raw.
volatile RAMN_Bool_t RAMN_SUMP_Active = False;

// SUMP configuration
static SUMP_Config_t sump_cfg;

// Extern: USB RX stream buffer from main.c
extern StreamBufferHandle_t USBD_RxStreamBufferHandle;

// Forward declarations
static void SUMP_SendID(void);
static void SUMP_SendMetadata(void);
static void SUMP_SendCapturedSamples(void);
static void SUMP_SendByte(uint8_t byte);
static void SUMP_SendBytes(const uint8_t* data, uint32_t len);

// ---- USB Send Helpers ----

// Set True when a USB send fails; checked by the send loop to abort early
// instead of blocking for seconds on each subsequent failed send.
static volatile RAMN_Bool_t sump_send_error = False;

static void SUMP_SendByte(uint8_t byte)
{
    if (sump_send_error == True) return;
    if (RAMN_USB_SendFromTask(&byte, 1U) != RAMN_OK) sump_send_error = True;
}

static void SUMP_SendBytes(const uint8_t* data, uint32_t len)
{
    if (sump_send_error == True) return;
    if (RAMN_USB_SendFromTask(data, len) != RAMN_OK) sump_send_error = True;
}

// ---- SUMP ID Response ----

static void SUMP_SendID(void)
{
    // SUMP protocol requires the response "1ALS" for the ID query
    SUMP_SendBytes((const uint8_t*)"1ALS", 4U);
}

// ---- SUMP Metadata Response ----

static void SUMP_SendMetadata(void)
{
    // Device name
    SUMP_SendByte(SUMP_META_NAME);
    SUMP_SendBytes((const uint8_t*)SUMP_DEVICE_NAME, sizeof(SUMP_DEVICE_NAME)); // includes null terminator

    // Sample memory (4 bytes, big-endian) — report the RLE-inflated window
    SUMP_SendByte(SUMP_META_SAMPLE_MEM);
    SUMP_SendByte((SUMP_REPORTED_SAMPLE_MEM >> 24) & 0xFF);
    SUMP_SendByte((SUMP_REPORTED_SAMPLE_MEM >> 16) & 0xFF);
    SUMP_SendByte((SUMP_REPORTED_SAMPLE_MEM >> 8) & 0xFF);
    SUMP_SendByte(SUMP_REPORTED_SAMPLE_MEM & 0xFF);

    // Max sample rate (4 bytes, big-endian)
    SUMP_SendByte(SUMP_META_SAMPLE_RATE);
    SUMP_SendByte((SUMP_MAX_SAMPLE_RATE >> 24) & 0xFF);
    SUMP_SendByte((SUMP_MAX_SAMPLE_RATE >> 16) & 0xFF);
    SUMP_SendByte((SUMP_MAX_SAMPLE_RATE >> 8) & 0xFF);
    SUMP_SendByte(SUMP_MAX_SAMPLE_RATE & 0xFF);

    // Number of probes (1 byte)
    SUMP_SendByte(SUMP_META_NUM_PROBES);
    SUMP_SendByte(SUMP_NUM_PROBES);

    // Protocol version (1 byte)
    SUMP_SendByte(SUMP_META_PROTO_VERS);
    SUMP_SendByte(2);

    // End of metadata
    SUMP_SendByte(SUMP_META_END);
}

// ---- Send Pre-Recorded Samples (windowed around trigger) ----

// Helper: emit a single sample word using the current sample width.
// With 3 probes PulseView typically enables only channel group 1 (byte 0),
// so sample_width = 1 and only one byte is sent per sample.
static inline void SUMP_EmitSample(uint8_t sample)
{
    uint8_t sw = sump_cfg.sample_width;
    uint8_t buf[4] = {sample, 0, 0, 0};
    SUMP_SendBytes(buf, sw);
}

// Helper: emit an RLE repeat-count word using the current sample width.
// The MSB of the last byte is set to distinguish counts from samples.
// Maximum representable count depends on sample width:
//   1 byte  → 7 bits  → 0-127
//   2 bytes → 15 bits → 0-32767
//   3 bytes → 23 bits → 0-8388607
//   4 bytes → 31 bits → 0-2147483647
static inline void SUMP_EmitRLECount(uint32_t count)
{
    uint8_t sw = sump_cfg.sample_width;
    uint8_t buf[4];
    buf[0] = (uint8_t)(count & 0xFF);
    if (sw >= 2) buf[1] = (uint8_t)((count >> 8) & 0xFF);
    if (sw >= 3) buf[2] = (uint8_t)((count >> 16) & 0xFF);
    if (sw >= 4) buf[3] = (uint8_t)((count >> 24) & 0x7F);
    // Set RLE marker (MSB) on the last byte
    buf[sw - 1] |= SUMP_RLE_MARKER;
    SUMP_SendBytes(buf, sw);
}

// Maximum RLE count for the current sample width.
static inline uint32_t SUMP_MaxRLECount(void)
{
    switch (sump_cfg.sample_width)
    {
    case 1:  return 0x7FU;
    case 2:  return 0x7FFFU;
    case 3:  return 0x7FFFFFU;
    default: return 0x7FFFFFFFU;
    }
}

// Compute sample width (bytes per sample) from the FLAGS register.
// FLAGS bits 2-5 are channel group disable flags (1 = disabled).
// Each enabled group adds 1 byte to the sample width.
static uint8_t SUMP_ComputeSampleWidth(uint32_t flags)
{
    uint8_t width = 0;
    if (!(flags & SUMP_FLAG_CHANGRP_1)) width++;
    if (!(flags & SUMP_FLAG_CHANGRP_2)) width++;
    if (!(flags & SUMP_FLAG_CHANGRP_3)) width++;
    if (!(flags & SUMP_FLAG_CHANGRP_4)) width++;
    // Safety: at least 1 byte (if all groups somehow disabled, default to 1)
    if (width == 0) width = 1;
    return width;
}

static void SUMP_SendCapturedSamples(void)
{
    uint32_t total_written = RAMN_SUMP_SampleCount;
    uint32_t trig_idx = RAMN_SUMP_TriggerIndex;

    // Determine how many samples are actually in the circular buffer
    uint32_t available = total_written;
    if (available > SUMP_SAMPLE_BUFFER_SIZE) available = SUMP_SAMPLE_BUFFER_SIZE;

    // Determine how many samples PulseView wants
    uint32_t read_count = sump_cfg.read_count;
    RAMN_Bool_t rle_enabled = (sump_cfg.flags & SUMP_FLAG_RLE) ? True : False;

    // Clamp read_count based on whether RLE is active
    if (rle_enabled == True)
    {
        // With RLE, we can represent up to SUMP_REPORTED_SAMPLE_MEM samples
        if (read_count > SUMP_REPORTED_SAMPLE_MEM) read_count = SUMP_REPORTED_SAMPLE_MEM;
        if (read_count == 0) read_count = SUMP_REPORTED_SAMPLE_MEM;
    }
    else
    {
        // Without RLE, clamp to physical buffer
        if (read_count > SUMP_SAMPLE_BUFFER_SIZE) read_count = SUMP_SAMPLE_BUFFER_SIZE;
        if (read_count == 0) read_count = SUMP_SAMPLE_BUFFER_SIZE;
    }

    // If no samples captured yet, send all-recessive (both pins high, no trigger)
    if (available == 0)
    {
        uint8_t idle = SUMP_BIT_TX | SUMP_BIT_RX;
        if (rle_enabled == True && read_count > 1)
        {
            // Emit one idle sample + RLE counts (split at max boundary)
            uint32_t max_rle = SUMP_MaxRLECount();
            uint32_t pad = read_count - 1;
            SUMP_EmitSample(idle);
            while (pad > 0)
            {
                uint32_t batch = (pad > max_rle) ? max_rle : pad;
                SUMP_EmitRLECount(batch);
                pad -= batch;
                if (pad > 0)
                {
                    SUMP_EmitSample(idle);
                    pad--;
                }
            }
        }
        else
        {
            for (uint32_t i = 0; i < read_count; i++)
            {
                SUMP_EmitSample(idle);
            }
        }
        return;
    }

    // Window around the trigger point.
    // delay_count = number of post-trigger samples PulseView expects.
    // The "end" of data we send is delay_count samples after the trigger.
    // The "start" is (read_count - delay_count) samples before the trigger.
    //
    // If no trigger was recorded, treat the last sample as the trigger point
    // (send the most recent read_count samples).

    uint32_t delay_count = sump_cfg.delay_count;
    if (delay_count > read_count) delay_count = read_count;

    uint32_t end_linear;
    uint32_t start_linear;

    if (trig_idx != SUMP_NO_TRIGGER && trig_idx <= total_written)
    {
        end_linear = trig_idx + delay_count;
        if (end_linear > total_written) end_linear = total_written;
        start_linear = (end_linear > read_count) ? (end_linear - read_count) : 0;
    }
    else
    {
        end_linear = total_written;
        start_linear = (total_written > read_count) ? (total_written - read_count) : 0;
    }

    // Ensure we don't read samples that have been overwritten by the circular buffer
    if (total_written > SUMP_SAMPLE_BUFFER_SIZE)
    {
        uint32_t oldest_available = total_written - SUMP_SAMPLE_BUFFER_SIZE;
        if (start_linear < oldest_available) start_linear = oldest_available;
    }

    uint32_t send_count = end_linear - start_linear;
    if (send_count > read_count) send_count = read_count;

    // SUMP protocol: send samples newest-to-oldest (reverse order)
    if (rle_enabled == True)
    {
        // RLE encoding: compress consecutive identical samples.
        // The max count per RLE word depends on sample_width (e.g., 127 for 1 byte).
        uint32_t max_rle = SUMP_MaxRLECount();
        uint32_t idx = end_linear;
        uint32_t sent = 0;
        uint8_t prev_sample = 0xFF; // Invalid initial value
        uint32_t run_count = 0;

        while (sent < send_count && idx > start_linear)
        {
            idx--;
            uint32_t circ_idx = idx & SUMP_SAMPLE_BUFFER_MASK;
            uint8_t sample = RAMN_SUMP_Samples[circ_idx];

            if (sample == prev_sample && run_count < max_rle)
            {
                run_count++;
            }
            else
            {
                // Flush previous run
                if (run_count > 0)
                {
                    SUMP_EmitRLECount(run_count);
                }
                if (sample != prev_sample)
                {
                    // New distinct sample
                    SUMP_EmitSample(sample);
                    prev_sample = sample;
                    run_count = 0;
                }
                else
                {
                    // Same sample but hit max count — re-emit sample to start new run
                    SUMP_EmitSample(sample);
                    run_count = 0;
                }
            }
            sent++;
        }

        // Flush final run
        if (run_count > 0)
        {
            SUMP_EmitRLECount(run_count);
        }

        // Pad to reach read_count with idle samples (RLE-compressed)
        if (sent < read_count)
        {
            uint32_t pad_count = read_count - sent;
            uint8_t idle = SUMP_BIT_TX | SUMP_BIT_RX;

            if (prev_sample != idle)
            {
                // Start a new idle run
                SUMP_EmitSample(idle);
                pad_count--;
            }

            // Emit RLE counts, splitting at max_rle boundary
            while (pad_count > 0)
            {
                uint32_t batch = (pad_count > max_rle) ? max_rle : pad_count;
                SUMP_EmitRLECount(batch);
                pad_count -= batch;
                if (pad_count > 0)
                {
                    // Re-emit sample for next batch
                    SUMP_EmitSample(idle);
                    pad_count--;
                }
            }
        }
    }
    else
    {
        // Non-RLE path: send samples one by one
        uint32_t idx = end_linear;
        uint32_t sent = 0;

        while (sent < send_count && idx > start_linear)
        {
            idx--;
            uint32_t circ_idx = idx & SUMP_SAMPLE_BUFFER_MASK;
            SUMP_EmitSample(RAMN_SUMP_Samples[circ_idx]);
            sent++;
        }

        // Pad with idle samples
        while (sent < read_count)
        {
            SUMP_EmitSample(SUMP_BIT_TX | SUMP_BIT_RX);
            sent++;
        }
    }
}

// ---- SUMP Command Processing ----

// State for multi-byte command reception
static uint8_t sump_cmd_pending = 0;
static uint8_t sump_cmd_buf[4];
static uint8_t sump_cmd_buf_idx = 0;

// Returns True if SUMP mode should exit
static RAMN_Bool_t SUMP_ProcessCommand(uint8_t cmd, const uint8_t* params)
{
    switch (cmd)
    {
    case SUMP_RESET:
        // Flush any stale TX data (e.g. from a previous SUMP_RUN whose
        // samples were not fully consumed by the host before reconnecting).
        RAMN_USB_FlushTxPipeline();
        // Reset internal state (but preserve the captured sample buffer)
        sump_cfg.state = SUMP_STATE_IDLE;
        sump_cfg.divider = 0;
        sump_cfg.read_count = SUMP_REPORTED_SAMPLE_MEM;
        sump_cfg.delay_count = SUMP_REPORTED_SAMPLE_MEM;
        sump_cfg.flags = 0;
        sump_cfg.sample_width = 4;  // Default: all channel groups enabled
        for (int i = 0; i < 4; i++)
        {
            sump_cfg.trigger_mask[i] = 0;
            sump_cfg.trigger_values[i] = 0;
        }
        break;

    case SUMP_RUN:
        // Return the pre-recorded bitbang samples.
        // If no samples have been captured yet (no prior bb command),
        // this sends idle (all-recessive) data, which is correct:
        // the user must first run a bitbang capture command to populate
        // the SUMP buffer.
        sump_send_error = False;
        SUMP_SendCapturedSamples();
        break;

    case SUMP_ID:
        sump_send_error = False;
        // Flush stale TX data so "1ALS" is the first thing the host reads
        RAMN_USB_FlushTxPipeline();
        SUMP_SendID();
        break;

    case SUMP_DESC:
        sump_send_error = False;
        SUMP_SendMetadata();
        break;

    case SUMP_XON:
    case SUMP_XOFF:
        // Flow control: not implemented
        break;

    case SUMP_DIV:
        sump_cfg.divider = (uint32_t)params[0]
                         | ((uint32_t)params[1] << 8)
                         | ((uint32_t)params[2] << 16);
        break;

    case SUMP_CNT:
        // Combined read+delay count (original SUMP spec, 0x81).
        // bytes[0:1] = read count, bytes[2:3] = delay count.
        sump_cfg.read_count  = (((uint32_t)params[1] << 8) | (uint32_t)params[0]) + 1;
        sump_cfg.read_count <<= 2;  // multiply by 4 per SUMP spec
        sump_cfg.delay_count = (((uint32_t)params[3] << 8) | (uint32_t)params[2]) + 1;
        sump_cfg.delay_count <<= 2;
        break;

    case SUMP_READCOUNT:
        // Separate read count command (sigrok OLS driver, 0x84).
        // 4-byte little-endian value: (value + 1) * 4 = sample count.
        sump_cfg.read_count = (((uint32_t)params[1] << 8) | (uint32_t)params[0]) + 1;
        sump_cfg.read_count <<= 2;
        break;

    case SUMP_DELAYCOUNT:
        // Separate delay count command (sigrok OLS driver, 0x83).
        // 4-byte little-endian value: (value + 1) * 4 = delay sample count.
        sump_cfg.delay_count = (((uint32_t)params[1] << 8) | (uint32_t)params[0]) + 1;
        sump_cfg.delay_count <<= 2;
        break;

    case SUMP_FLAGS:
        sump_cfg.flags = (uint32_t)params[0]
                       | ((uint32_t)params[1] << 8)
                       | ((uint32_t)params[2] << 16)
                       | ((uint32_t)params[3] << 24);
        sump_cfg.sample_width = SUMP_ComputeSampleWidth(sump_cfg.flags);
        break;

    default:
        // Trigger mask/value commands (0xC0-0xCD range)
        if ((cmd & 0xF0) == 0xC0)
        {
            uint8_t stage = (cmd & 0x0C) >> 2;
            uint32_t val = (uint32_t)params[0]
                         | ((uint32_t)params[1] << 8)
                         | ((uint32_t)params[2] << 16)
                         | ((uint32_t)params[3] << 24);

            if ((cmd & 0x01) == 0)
            {
                // Even: trigger mask
                if (stage < 4) sump_cfg.trigger_mask[stage] = val;
            }
            else
            {
                // Odd: trigger value
                if (stage < 4) sump_cfg.trigger_values[stage] = val;
            }
        }
        break;
    }

    return False;
}

// Returns True if the byte is a SUMP "long command" (requires 4 parameter bytes)
static RAMN_Bool_t SUMP_IsLongCommand(uint8_t cmd)
{
    return (cmd >= 0x80) ? True : False;
}

RAMN_Bool_t RAMN_SUMP_ProcessByte(uint8_t byte)
{
    // Check for ESC exit
    if (byte == SUMP_EXIT_CHAR)
    {
        sump_cfg.state = SUMP_STATE_IDLE;
        return True;
    }

    // If we are collecting parameters for a long command
    if (sump_cmd_pending != 0)
    {
        sump_cmd_buf[sump_cmd_buf_idx++] = byte;
        if (sump_cmd_buf_idx >= 4)
        {
            RAMN_Bool_t exit = SUMP_ProcessCommand(sump_cmd_pending, sump_cmd_buf);
            sump_cmd_pending = 0;
            sump_cmd_buf_idx = 0;
            return exit;
        }
        return False;
    }

    // New command byte
    if (SUMP_IsLongCommand(byte) == True)
    {
        sump_cmd_pending = byte;
        sump_cmd_buf_idx = 0;
        return False;
    }
    else
    {
        // Short command - process immediately
        return SUMP_ProcessCommand(byte, NULL);
    }
}

RAMN_Bool_t RAMN_SUMP_IsSUMPProbe(const uint8_t* buffer, uint32_t length)
{
    // PulseView sends SUMP_ID (0x02) as its probe command.
    // In slcan mode, 0x02 is not a valid slcan command character.
    // A single-byte buffer containing only 0x02 indicates PulseView.
    if (length == 1 && buffer[0] == SUMP_ID)
    {
        return True;
    }
    return False;
}

void RAMN_SUMP_Enter(void)
{
    // Signal the CDC ISR to forward all bytes raw (no line-buffering)
    RAMN_SUMP_Active = True;

    // Initialize SUMP state
    sump_cmd_pending = 0;
    sump_cmd_buf_idx = 0;
    sump_cfg.state = SUMP_STATE_IDLE;
    sump_cfg.divider = 0;
    sump_cfg.read_count = SUMP_REPORTED_SAMPLE_MEM;
    sump_cfg.delay_count = SUMP_REPORTED_SAMPLE_MEM;
    sump_cfg.flags = 0;
    sump_cfg.sample_width = 4;  // Default: all channel groups enabled
    for (int i = 0; i < 4; i++)
    {
        sump_cfg.trigger_mask[i] = 0;
        sump_cfg.trigger_values[i] = 0;
    }

    // SUMP mode loop: read bytes directly from USB stream buffer
    for (;;)
    {
        uint16_t commandLength;
        size_t xBytesReceived;
        RAMN_Bool_t shouldExit = False;

        // Wait for a USB data chunk
        xBytesReceived = xStreamBufferReceive(USBD_RxStreamBufferHandle, (void*)&commandLength, 2U, portMAX_DELAY);
        if (xBytesReceived != 2U || commandLength == 0) continue;

        // Read the command data
        uint8_t rxBuf[64];
        uint32_t remaining = commandLength;

        while (remaining > 0 && shouldExit == False)
        {
            uint32_t chunk = remaining;
            if (chunk > sizeof(rxBuf)) chunk = sizeof(rxBuf);

            xBytesReceived = xStreamBufferReceive(USBD_RxStreamBufferHandle, (void*)rxBuf, chunk, portMAX_DELAY);
            if (xBytesReceived == 0) break;
            remaining -= xBytesReceived;

            for (uint32_t i = 0; i < xBytesReceived; i++)
            {
                if (RAMN_SUMP_ProcessByte(rxBuf[i]) == True)
                {
                    shouldExit = True;
                    break;
                }
            }
        }

        // Drain any leftover bytes from this command
        while (remaining > 0)
        {
            uint32_t drain = remaining;
            if (drain > sizeof(rxBuf)) drain = sizeof(rxBuf);
            size_t drained = xStreamBufferReceive(USBD_RxStreamBufferHandle, (void*)rxBuf, drain, 0);
            if (drained == 0) break;
            remaining -= drained;
        }

        if (shouldExit == True)
        {
            RAMN_SUMP_Active = False;
            return;
        }
    }
}

#endif /* ENABLE_SUMP_OLS && ENABLE_BITBANG */
