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

// Shared compressed sample buffer: circular, written by bitbang operations, read by SUMP protocol.
uint8_t  RAMN_SUMP_Samples[SUMP_SAMPLE_BUFFER_SIZE];
volatile uint32_t RAMN_SUMP_SampleCount     = 0;
volatile uint32_t RAMN_SUMP_CompressedBytes  = 0;
volatile uint32_t RAMN_SUMP_WriteIndex       = 0;
volatile uint32_t RAMN_SUMP_TriggerIndex     = SUMP_NO_TRIGGER;
volatile uint8_t  RAMN_SUMP_PrevSample       = 0xFFU;
volatile uint32_t RAMN_SUMP_SampleRate       = 500000U;  // Default 500 kHz (500 kbps CAN)

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
static inline void SUMP_EmitSample(uint8_t sample);
static inline void SUMP_EmitRLECount(uint32_t count);
static inline uint32_t SUMP_MaxRLECount(void);

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

// ---- SUMP Protocol Sample Emission Helpers ----

// Emit a single sample word using the current sample width.
// With 3 probes PulseView typically enables only channel group 1 (byte 0),
// so sample_width = 1 and only one byte is sent per sample.
static inline void SUMP_EmitSample(uint8_t sample)
{
    uint8_t sw = sump_cfg.sample_width;
    uint8_t buf[4] = {sample, 0, 0, 0};
    SUMP_SendBytes(buf, sw);
}

// Emit an RLE repeat-count word using the current sample width.
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

// ---- SUMP Metadata Response ----

static void SUMP_SendMetadata(void)
{
    // Device name
    SUMP_SendByte(SUMP_META_NAME);
    SUMP_SendBytes((const uint8_t*)SUMP_DEVICE_NAME, sizeof(SUMP_DEVICE_NAME)); // includes null terminator

    // Sample memory (4 bytes, big-endian).
    // Report the actual number of logical samples captured.
    // If no capture yet, report the physical buffer size as the minimum.
    uint32_t reported_mem = RAMN_SUMP_SampleCount;
    if (reported_mem < SUMP_SAMPLE_BUFFER_SIZE)
        reported_mem = SUMP_SAMPLE_BUFFER_SIZE;
    SUMP_SendByte(SUMP_META_SAMPLE_MEM);
    SUMP_SendByte((reported_mem >> 24) & 0xFF);
    SUMP_SendByte((reported_mem >> 16) & 0xFF);
    SUMP_SendByte((reported_mem >> 8) & 0xFF);
    SUMP_SendByte(reported_mem & 0xFF);

    // Max sample rate (4 bytes, big-endian) — actual capture rate.
    uint32_t rate = RAMN_SUMP_SampleRate;
    SUMP_SendByte(SUMP_META_SAMPLE_RATE);
    SUMP_SendByte((rate >> 24) & 0xFF);
    SUMP_SendByte((rate >> 16) & 0xFF);
    SUMP_SendByte((rate >> 8) & 0xFF);
    SUMP_SendByte(rate & 0xFF);

    // Number of probes (1 byte)
    SUMP_SendByte(SUMP_META_NUM_PROBES);
    SUMP_SendByte(SUMP_NUM_PROBES);

    // Protocol version (1 byte)
    SUMP_SendByte(SUMP_META_PROTO_VERS);
    SUMP_SendByte(2);

    // End of metadata
    SUMP_SendByte(SUMP_META_END);
}

// ---- Compressed buffer helpers ----

// Count the total logical samples in the available compressed data.
// Walks the compressed buffer forward from start_byte for avail_bytes bytes.
static uint32_t SUMP_CountLogicalSamples(uint32_t start_byte, uint32_t avail_bytes)
{
    uint32_t logical = 0;
    uint32_t pos = start_byte;
    for (uint32_t i = 0; i < avail_bytes; i++)
    {
        uint8_t b = RAMN_SUMP_Samples[pos];
        if (b & SUMP_STORAGE_RLE_FLAG)
            logical += (b & 0x7FU);   // RLE count = additional repeats
        else
            logical += 1U;            // Sample byte = 1 logical sample
        pos = (pos + 1U) & SUMP_SAMPLE_BUFFER_MASK;
    }
    return logical;
}

// Read one compressed run in reverse.
// *pos starts at the most-recent compressed byte and is moved backwards.
// *bytes_left is decremented accordingly.
// Returns the sample value in *sample and the logical count in *count.
static void SUMP_ReadRunReverse(uint32_t* pos, uint32_t* bytes_left,
                                uint8_t* sample, uint32_t* count)
{
    uint8_t b = RAMN_SUMP_Samples[*pos];
    *pos = (*pos - 1U) & SUMP_SAMPLE_BUFFER_MASK;
    (*bytes_left)--;

    if (b & SUMP_STORAGE_RLE_FLAG)
    {
        // RLE count byte — preceding byte must be the sample.
        uint32_t c = (b & 0x7FU);
        if (*bytes_left > 0U)
        {
            *sample = RAMN_SUMP_Samples[*pos];
            *pos = (*pos - 1U) & SUMP_SAMPLE_BUFFER_MASK;
            (*bytes_left)--;
            *count = c + 1U;   // sample itself (1) + additional repeats (c)
        }
        else
        {
            // Orphan RLE with no preceding sample — shouldn't happen after
            // the start-of-buffer fixup, but handle gracefully.
            *sample = SUMP_BIT_TX | SUMP_BIT_RX;
            *count = c;
        }
    }
    else
    {
        // Plain sample byte — represents 1 logical sample.
        *sample = b;
        *count  = 1U;
    }
}

// ---- Send Pre-Recorded Compressed Samples (windowed around trigger) ----

// Helper: emit sample+RLE for a run of `emit_count` copies of `sample`.
// Handles splitting at the SUMP protocol max-RLE boundary.
static void SUMP_EmitRun(uint8_t sample, uint32_t emit_count,
                         uint8_t* prev_sample, uint32_t* pending_rle,
                         RAMN_Bool_t rle_enabled, uint32_t max_rle)
{
    if (!rle_enabled)
    {
        for (uint32_t i = 0; i < emit_count; i++)
            SUMP_EmitSample(sample);
        return;
    }

    if (sample != *prev_sample)
    {
        // Flush any pending RLE count from the previous value.
        if (*pending_rle > 0U)
        {
            while (*pending_rle > max_rle)
            {
                SUMP_EmitRLECount(max_rle);
                SUMP_EmitSample(*prev_sample);
                *pending_rle -= max_rle;
                (*pending_rle)--;  // account for the re-emitted sample
            }
            if (*pending_rle > 0U)
                SUMP_EmitRLECount(*pending_rle);
        }
        // Start new run.
        SUMP_EmitSample(sample);
        *prev_sample = sample;
        *pending_rle = emit_count - 1U;
    }
    else
    {
        *pending_rle += emit_count;
    }

    // Flush pending_rle at max_rle boundaries to avoid overflow.
    while (*pending_rle > max_rle)
    {
        SUMP_EmitRLECount(max_rle);
        SUMP_EmitSample(sample);
        *pending_rle -= max_rle;
        (*pending_rle)--;  // account for the re-emitted sample
    }
}

static void SUMP_SendCapturedSamples(void)
{
    uint32_t total_logical     = RAMN_SUMP_SampleCount;
    uint32_t total_compressed  = RAMN_SUMP_CompressedBytes;
    uint32_t trig_idx          = RAMN_SUMP_TriggerIndex;

    // Determine how many compressed bytes survive in the circular buffer.
    uint32_t avail_bytes = total_compressed;
    if (avail_bytes > SUMP_SAMPLE_BUFFER_SIZE) avail_bytes = SUMP_SAMPLE_BUFFER_SIZE;

    // Determine how many samples PulseView wants.
    uint32_t read_count  = sump_cfg.read_count;
    RAMN_Bool_t rle_enabled = (sump_cfg.flags & SUMP_FLAG_RLE) ? True : False;

    // Clamp read_count.
    uint32_t max_requestable = rle_enabled ? SUMP_DEFAULT_SAMPLE_MEM : SUMP_SAMPLE_BUFFER_SIZE;
    if (read_count > max_requestable) read_count = max_requestable;
    if (read_count == 0U)             read_count = max_requestable;

    // ---- No captured data: send idle ----
    if (avail_bytes == 0U || total_logical == 0U)
    {
        uint8_t idle = SUMP_BIT_TX | SUMP_BIT_RX;
        if (rle_enabled && read_count > 1U)
        {
            uint32_t max_rle = SUMP_MaxRLECount();
            uint32_t pad = read_count - 1U;
            SUMP_EmitSample(idle);
            while (pad > 0U)
            {
                uint32_t batch = (pad > max_rle) ? max_rle : pad;
                SUMP_EmitRLECount(batch);
                pad -= batch;
                if (pad > 0U)
                {
                    SUMP_EmitSample(idle);
                    pad--;
                }
            }
        }
        else
        {
            for (uint32_t i = 0; i < read_count; i++)
                SUMP_EmitSample(idle);
        }
        return;
    }

    // ---- Determine oldest valid compressed byte ----
    uint32_t start_byte;
    if (total_compressed > SUMP_SAMPLE_BUFFER_SIZE)
    {
        start_byte = RAMN_SUMP_WriteIndex;
        // If the oldest byte is an orphan RLE (its sample was overwritten), skip it.
        if (RAMN_SUMP_Samples[start_byte] & SUMP_STORAGE_RLE_FLAG)
        {
            start_byte = (start_byte + 1U) & SUMP_SAMPLE_BUFFER_MASK;
            avail_bytes--;
        }
    }
    else
    {
        start_byte = 0;
    }

    // Count logical samples in the surviving compressed data.
    uint32_t available_logical = SUMP_CountLogicalSamples(start_byte, avail_bytes);
    uint32_t oldest_logical    = total_logical - available_logical;

    // ---- Window around trigger ----
    uint32_t delay_count = sump_cfg.delay_count;
    if (delay_count > read_count) delay_count = read_count;

    uint32_t end_linear, start_linear;

    if (trig_idx != SUMP_NO_TRIGGER && trig_idx <= total_logical)
    {
        end_linear = trig_idx + delay_count;
        if (end_linear > total_logical) end_linear = total_logical;
        start_linear = (end_linear > read_count) ? (end_linear - read_count) : 0;
    }
    else
    {
        end_linear   = total_logical;
        start_linear = (total_logical > read_count) ? (total_logical - read_count) : 0;
    }

    // Don't read overwritten samples.
    if (start_linear < oldest_logical)
        start_linear = oldest_logical;

    uint32_t send_count = end_linear - start_linear;
    if (send_count > read_count) send_count = read_count;

    // ---- Reverse walk: skip, emit, pad ----
    uint32_t rpos       = (RAMN_SUMP_WriteIndex - 1U) & SUMP_SAMPLE_BUFFER_MASK;
    uint32_t bytes_left = avail_bytes;
    uint32_t logical_pos = total_logical;   // newest logical sample
    uint32_t sent        = 0;

    // RLE protocol state for the output stream.
    uint32_t max_rle     = SUMP_MaxRLECount();
    uint8_t  prev_sample = 0xFFU;
    uint32_t pending_rle = 0;

    // Phase 1 — skip runs newer than end_linear.
    while (logical_pos > end_linear && bytes_left > 0U)
    {
        uint8_t  rsample;
        uint32_t rcount;
        SUMP_ReadRunReverse(&rpos, &bytes_left, &rsample, &rcount);

        if (logical_pos - rcount >= end_linear)
        {
            // Entire run is newer than end_linear — skip it.
            logical_pos -= rcount;
        }
        else
        {
            // Partially in the skip zone.
            uint32_t to_skip = logical_pos - end_linear;
            rcount -= to_skip;
            logical_pos = end_linear;

            // Emit the remaining part.
            uint32_t to_emit = rcount;
            if (to_emit > send_count) to_emit = send_count;
            SUMP_EmitRun(rsample, to_emit, &prev_sample, &pending_rle,
                         rle_enabled, max_rle);
            logical_pos -= to_emit;
            sent += to_emit;
        }
    }

    // Phase 2 — emit runs from end_linear backward to start_linear.
    while (sent < send_count && bytes_left > 0U && logical_pos > start_linear)
    {
        uint8_t  rsample;
        uint32_t rcount;
        SUMP_ReadRunReverse(&rpos, &bytes_left, &rsample, &rcount);

        uint32_t to_emit = rcount;
        if (logical_pos - to_emit < start_linear)
            to_emit = logical_pos - start_linear;
        if (to_emit > send_count - sent)
            to_emit = send_count - sent;

        SUMP_EmitRun(rsample, to_emit, &prev_sample, &pending_rle,
                     rle_enabled, max_rle);
        logical_pos -= to_emit;
        sent += to_emit;
    }

    // Flush any pending RLE count.
    if (rle_enabled && pending_rle > 0U)
    {
        while (pending_rle > max_rle)
        {
            SUMP_EmitRLECount(max_rle);
            SUMP_EmitSample(prev_sample);
            pending_rle -= max_rle;
            pending_rle--;
        }
        if (pending_rle > 0U)
            SUMP_EmitRLECount(pending_rle);
    }

    // Phase 3 — pad remaining with idle samples.
    if (sent < read_count)
    {
        uint32_t pad_count = read_count - sent;
        uint8_t idle = SUMP_BIT_TX | SUMP_BIT_RX;

        if (rle_enabled)
        {
            if (prev_sample != idle)
            {
                SUMP_EmitSample(idle);
                pad_count--;
            }
            while (pad_count > 0U)
            {
                uint32_t batch = (pad_count > max_rle) ? max_rle : pad_count;
                SUMP_EmitRLECount(batch);
                pad_count -= batch;
                if (pad_count > 0U)
                {
                    SUMP_EmitSample(idle);
                    pad_count--;
                }
            }
        }
        else
        {
            for (uint32_t i = 0; i < pad_count; i++)
                SUMP_EmitSample(idle);
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
        sump_cfg.read_count = SUMP_DEFAULT_SAMPLE_MEM;
        sump_cfg.delay_count = SUMP_DEFAULT_SAMPLE_MEM;
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
    sump_cfg.read_count = SUMP_DEFAULT_SAMPLE_MEM;
    sump_cfg.delay_count = SUMP_DEFAULT_SAMPLE_MEM;
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
