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

static void SUMP_SendByte(uint8_t byte)
{
    RAMN_USB_SendFromTask(&byte, 1U);
}

static void SUMP_SendBytes(const uint8_t* data, uint32_t len)
{
    RAMN_USB_SendFromTask(data, len);
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

    // Sample memory (4 bytes, big-endian)
    SUMP_SendByte(SUMP_META_SAMPLE_MEM);
    SUMP_SendByte((SUMP_SAMPLE_BUFFER_SIZE >> 24) & 0xFF);
    SUMP_SendByte((SUMP_SAMPLE_BUFFER_SIZE >> 16) & 0xFF);
    SUMP_SendByte((SUMP_SAMPLE_BUFFER_SIZE >> 8) & 0xFF);
    SUMP_SendByte(SUMP_SAMPLE_BUFFER_SIZE & 0xFF);

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

static void SUMP_SendCapturedSamples(void)
{
    uint32_t total_written = RAMN_SUMP_SampleCount;
    uint32_t trig_idx = RAMN_SUMP_TriggerIndex;

    // Determine how many samples are actually in the circular buffer
    uint32_t available = total_written;
    if (available > SUMP_SAMPLE_BUFFER_SIZE) available = SUMP_SAMPLE_BUFFER_SIZE;

    // Determine how many samples PulseView wants
    uint32_t read_count = sump_cfg.read_count;
    if (read_count > SUMP_SAMPLE_BUFFER_SIZE) read_count = SUMP_SAMPLE_BUFFER_SIZE;
    if (read_count == 0) read_count = SUMP_SAMPLE_BUFFER_SIZE;

    // If no samples captured yet, send all-recessive (both pins high, no trigger)
    if (available == 0)
    {
        uint8_t idle_sample[4] = {SUMP_BIT_TX | SUMP_BIT_RX, 0, 0, 0};
        for (uint32_t i = 0; i < read_count; i++)
        {
            SUMP_SendBytes(idle_sample, 4U);
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

    // Determine the trigger position in the circular buffer
    // trig_idx is in the linear SampleCount space.
    // The circular buffer position of sample N is: N & SUMP_SAMPLE_BUFFER_MASK
    // But we need to handle the case where samples have wrapped.
    uint32_t end_linear;  // The linear index of the last sample to send (+1)
    uint32_t start_linear; // The linear index of the first sample to send

    if (trig_idx != SUMP_NO_TRIGGER && trig_idx <= total_written)
    {
        // Trigger is valid. Window around it.
        end_linear = trig_idx + delay_count;
        // Clamp to what's actually written
        if (end_linear > total_written) end_linear = total_written;
        start_linear = (end_linear > read_count) ? (end_linear - read_count) : 0;
    }
    else
    {
        // No trigger. Send the most recent read_count samples.
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
    uint8_t buf[4];
    // First, send samples from end_linear down to start_linear
    {
        uint32_t idx = end_linear;
        uint32_t sent = 0;
        while (sent < send_count && idx > start_linear)
        {
            idx--;
            uint32_t circ_idx = idx & SUMP_SAMPLE_BUFFER_MASK;
            buf[0] = RAMN_SUMP_Samples[circ_idx];
            buf[1] = 0;
            buf[2] = 0;
            buf[3] = 0;
            SUMP_SendBytes(buf, 4U);
            sent++;
        }

        // If we need to pad to reach read_count, send idle samples
        while (sent < read_count)
        {
            buf[0] = SUMP_BIT_TX | SUMP_BIT_RX;  // idle: both recessive, no trigger
            buf[1] = 0;
            buf[2] = 0;
            buf[3] = 0;
            SUMP_SendBytes(buf, 4U);
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
        // Reset internal state (but preserve the captured sample buffer)
        sump_cfg.state = SUMP_STATE_IDLE;
        sump_cfg.divider = 0;
        sump_cfg.read_count = SUMP_SAMPLE_BUFFER_SIZE;
        sump_cfg.delay_count = SUMP_SAMPLE_BUFFER_SIZE;
        sump_cfg.flags = 0;
        for (int i = 0; i < 4; i++)
        {
            sump_cfg.trigger_mask[i] = 0;
            sump_cfg.trigger_values[i] = 0;
        }
        break;

    case SUMP_RUN:
        // Return the pre-recorded bitbang samples
        SUMP_SendCapturedSamples();
        break;

    case SUMP_ID:
        SUMP_SendID();
        break;

    case SUMP_DESC:
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
        sump_cfg.read_count  = (((uint32_t)params[1] << 8) | (uint32_t)params[0]) + 1;
        sump_cfg.read_count <<= 2;  // multiply by 4 per SUMP spec
        sump_cfg.delay_count = ((uint32_t)params[3] << 8) | (uint32_t)params[2];
        sump_cfg.delay_count <<= 2;
        break;

    case SUMP_FLAGS:
        sump_cfg.flags = (uint32_t)params[0];
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
    // Initialize SUMP state
    sump_cmd_pending = 0;
    sump_cmd_buf_idx = 0;
    sump_cfg.state = SUMP_STATE_IDLE;
    sump_cfg.divider = 0;
    sump_cfg.read_count = SUMP_SAMPLE_BUFFER_SIZE;
    sump_cfg.delay_count = SUMP_SAMPLE_BUFFER_SIZE;
    sump_cfg.flags = 0;
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
            return;
        }
    }
}

#endif /* ENABLE_SUMP_OLS && ENABLE_BITBANG */
