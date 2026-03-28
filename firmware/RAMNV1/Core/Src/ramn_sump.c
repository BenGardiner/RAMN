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
#include "ramn_canfd.h"
#include "cmsis_os.h"
#include "stream_buffer.h"

// Circular sample buffer
static uint8_t sump_samples[SUMP_SAMPLE_BUFFER_SIZE];
static volatile uint32_t sump_index = 0;

// SUMP configuration
static SUMP_Config_t sump_cfg;

// Timer for SUMP sampling (reuses TIM2 from bitbang)
#define SUMP_TIM (TIM2)

// CAN pin definitions (same as bitbang: PB8 = RX, PB9 = TX)
#define SUMP_PIN_RX  (1U << 8)
#define SUMP_PIN_TX  (1U << 9)

// Extern: USB RX stream buffer from main.c
extern StreamBufferHandle_t USBD_RxStreamBufferHandle;

// Forward declarations
static void SUMP_SendID(void);
static void SUMP_SendMetadata(void);
static void SUMP_ArmAndCapture(void);
static void SUMP_SetupGPIO(void);
static void SUMP_RestoreGPIO(void);
static void SUMP_SendByte(uint8_t byte);
static void SUMP_SendBytes(const uint8_t* data, uint32_t len);
static uint8_t SUMP_SamplePins(void);

// ---- GPIO Setup / Restore ----

static void SUMP_SetupGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    RAMN_FDCAN_Disable();

    // PB8 (CAN RX) as input
    GPIO_InitStruct.Pin  = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB9 (CAN TX) as input (we only observe in SUMP mode)
    GPIO_InitStruct.Pin  = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void SUMP_RestoreGPIO(void)
{
    RAMN_FDCAN_ResetPeripheral();
}

// ---- Pin Sampling ----

static inline uint8_t SUMP_SamplePins(void)
{
    uint32_t idr = GPIOB->IDR;
    uint8_t sample = 0;
    // Channel 0: CAN TX (PB9)
    if (idr & SUMP_PIN_TX) sample |= 0x01;
    // Channel 1: CAN RX (PB8)
    if (idr & SUMP_PIN_RX) sample |= 0x02;
    return sample;
}

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

// ---- SUMP Capture ----

__attribute__((optimize("Ofast"))) static void SUMP_ArmAndCapture(void)
{
    uint32_t read_count  = sump_cfg.read_count;
    uint32_t delay_count = sump_cfg.delay_count;
    uint32_t divider     = sump_cfg.divider;
    uint32_t trig_mask   = sump_cfg.trigger_mask[0];
    uint32_t trig_val    = sump_cfg.trigger_values[0];

    // Clamp counts to buffer size
    if (read_count > SUMP_SAMPLE_BUFFER_SIZE) read_count = SUMP_SAMPLE_BUFFER_SIZE;
    if (delay_count > SUMP_SAMPLE_BUFFER_SIZE) delay_count = SUMP_SAMPLE_BUFFER_SIZE;
    if (read_count == 0) read_count = SUMP_SAMPLE_BUFFER_SIZE;

    SUMP_SetupGPIO();

    // Configure timer for sampling
    SUMP_TIM->CR1 &= ~TIM_CR1_CEN;
    SUMP_TIM->PSC = 0;   // No prescaler (80 MHz base)
    SUMP_TIM->CNT = 0;
    SUMP_TIM->EGR = TIM_EGR_UG;
    SUMP_TIM->CR1 |= TIM_CR1_CEN;

    // Calculate period in timer ticks
    // SUMP divider: sample_rate = max_clock / (divider + 1)
    // We use 80 MHz base clock / (divider + 1)
    uint32_t period = divider + 1;
    if (period < 1) period = 1;

    sump_index = 0;
    sump_cfg.state = SUMP_STATE_ARMED;

    // ---- Armed: sample until trigger ----
    if (trig_mask != 0)
    {
        while (sump_cfg.state == SUMP_STATE_ARMED)
        {
            SUMP_TIM->CNT = 0;
            while (SUMP_TIM->CNT < period);

            uint8_t sample = SUMP_SamplePins();
            sump_samples[sump_index & (SUMP_SAMPLE_BUFFER_SIZE - 1)] = sample;
            sump_index++;

            if (((sample ^ trig_val) & trig_mask) == 0)
            {
                sump_cfg.state = SUMP_STATE_TRIGGED;
            }
        }
    }
    else
    {
        // No trigger: go straight to capturing
        sump_cfg.state = SUMP_STATE_TRIGGED;
    }

    // ---- Triggered: capture delay_count more samples ----
    {
        uint32_t remaining = delay_count;
        while (remaining > 0)
        {
            SUMP_TIM->CNT = 0;
            while (SUMP_TIM->CNT < period);

            uint8_t sample = SUMP_SamplePins();
            sump_samples[sump_index & (SUMP_SAMPLE_BUFFER_SIZE - 1)] = sample;
            sump_index++;
            remaining--;
        }
    }

    SUMP_RestoreGPIO();
    sump_cfg.state = SUMP_STATE_IDLE;

    // ---- Send samples (newest to oldest) ----
    {
        uint32_t idx = sump_index;
        uint32_t count = read_count;
        uint8_t buf[4];

        while (count > 0)
        {
            idx--;
            uint8_t s = sump_samples[idx & (SUMP_SAMPLE_BUFFER_SIZE - 1)];

            // Send sample as 4 bytes (SUMP expects 4 bytes per sample group)
            buf[0] = s;
            buf[1] = 0;
            buf[2] = 0;
            buf[3] = 0;
            SUMP_SendBytes(buf, 4U);
            count--;
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
        // Reset internal state
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
        SUMP_ArmAndCapture();
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

    RAMN_USB_SendStringFromTask("Entering SUMP/OLS mode. Connect PulseView or send ESC (0x1B) to exit.\r");

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
            RAMN_USB_SendStringFromTask("Exited SUMP/OLS mode.\r");
            return;
        }
    }
}

#endif /* ENABLE_SUMP_OLS && ENABLE_BITBANG */
