#include "ramn_can_database.h"
#include "ramn_signal_defs.h"
#include <string.h>

#ifdef CPYTHON_TESTING
#ifndef RAMN_memcpy
#define RAMN_memcpy memcpy
#endif
#endif

// -- Command Brake --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Brake(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, COMMAND_BRAKE_MASK, COMMAND_BRAKE_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Command_Brake(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, COMMAND_BRAKE_MASK, COMMAND_BRAKE_OFFSET);
}
#else
void RAMN_Encode_Command_Brake(uint16_t value, uint8_t* payload) {
    /* Initialize payload to J1939 "Not Available" */
    memset(payload, 0xFF, 8);
    /* SPN 2920 (XBR External Decel Demand). Bytes 1-2. Offset: -15.687 m/s2. Res: 1/2048 m/s2 per bit. */
    /* 0 m/s2 maps to raw 32127. We assume ramn_val is already in 1/2048 m/s2 units starting from 0 m/s2. */
    uint16_t j1939_val = value + 32127;
    payload[0] = (uint8_t)(j1939_val & 0xFF);
    payload[1] = (uint8_t)((j1939_val >> 8) & 0xFF);
}

uint16_t RAMN_Decode_Command_Brake(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    uint16_t j1939_val = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if (j1939_val < 32127) return 0;
    return j1939_val - 32127;
}
#endif

// -- Control Brake --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Brake(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, CONTROL_BRAKE_MASK, CONTROL_BRAKE_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Control_Brake(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, CONTROL_BRAKE_MASK, CONTROL_BRAKE_OFFSET);
}
#else
void RAMN_Encode_Control_Brake(uint16_t value, uint8_t* payload) {
    memset(payload, 0xFF, 8);
    /* SPN 521 (Brake Pedal Position). Byte 2. Scale: 0.4%/bit. */
    /* ramn_val 4095 maps to 100% (raw 250). j1939_val = (ramn_val * 250) / 4095. */
    uint8_t j1939_val = (uint8_t)((value * 250U) / 4095U);
    payload[1] = j1939_val;
}

uint16_t RAMN_Decode_Control_Brake(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    uint8_t j1939_val = payload[1];
    if (j1939_val > 250) return 4095;
    return (uint16_t)((j1939_val * 4095U) / 250U);
}
#endif

// -- Command Accel --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Accel(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, COMMAND_ACCEL_MASK, COMMAND_ACCEL_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Command_Accel(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, COMMAND_ACCEL_MASK, COMMAND_ACCEL_OFFSET);
}
#else
void RAMN_Encode_Command_Accel(uint16_t value, uint8_t* payload) {
    memset(payload, 0xFF, 8);
    /* SPN 898 (Engine Requested Speed). Bytes 2-3. Scale: 0.125 rpm/bit. */
    /* Assume ramn_val maps to RPM (e.g., 0-8000). j1939_val = rpm / 0.125 = rpm * 8. */
    /* We assume ramn_val is already in 0.125 rpm units for consistency with Status_RPM. */
    uint16_t j1939_val = value;
    payload[1] = (uint8_t)(j1939_val & 0xFF);
    payload[2] = (uint8_t)((j1939_val >> 8) & 0xFF);
}

uint16_t RAMN_Decode_Command_Accel(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 3U) return 0;
    return (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
}
#endif

// -- Control Accel --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Accel(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, CONTROL_ACCEL_MASK, CONTROL_ACCEL_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Control_Accel(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, CONTROL_ACCEL_MASK, CONTROL_ACCEL_OFFSET);
}
#else
void RAMN_Encode_Control_Accel(uint16_t value, uint8_t* payload) {
    memset(payload, 0xFF, 8);
    /* SPN 91 (Accelerator Pedal Position 1). Byte 2. Scale: 0.4%/bit. */
    uint8_t j1939_val = (uint8_t)((value * 250U) / 4095U);
    payload[1] = j1939_val;
}

uint16_t RAMN_Decode_Control_Accel(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    uint8_t j1939_val = payload[1];
    if (j1939_val > 250) return 4095;
    return (uint16_t)((j1939_val * 4095U) / 250U);
}
#endif

// -- Status RPM --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Status_RPM(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, STATUS_RPM_MASK, STATUS_RPM_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Status_RPM(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, STATUS_RPM_MASK, STATUS_RPM_OFFSET);
}
#else
void RAMN_Encode_Status_RPM(uint16_t value, uint8_t* payload) {
    memset(payload, 0xFF, 8);
    /* SPN 190 (Engine Speed). Bytes 4-5. Scale: 0.125 rpm/bit. */
    uint16_t j1939_val = value;
    payload[3] = (uint8_t)(j1939_val & 0xFF);
    payload[4] = (uint8_t)((j1939_val >> 8) & 0xFF);
}

uint16_t RAMN_Decode_Status_RPM(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 5U) return 0;
    return (uint16_t)payload[3] | ((uint16_t)payload[4] << 8);
}
#endif

// -- Command Steering --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Steering(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, COMMAND_STEERING_MASK, COMMAND_STEERING_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Command_Steering(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, COMMAND_STEERING_MASK, COMMAND_STEERING_OFFSET);
}
#else
void RAMN_Encode_Command_Steering(uint16_t value, uint8_t* payload) {
    /* PGN 61184 (Proprietary A). Bytes 1-2. */
    memset(payload, 0xFF, 8);
    payload[0] = (uint8_t)(value & 0xFF);
    payload[1] = (uint8_t)((value >> 8) & 0xFF);
}

uint16_t RAMN_Decode_Command_Steering(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    return (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
}
#endif

// -- Control Steering --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Steering(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, CONTROL_STEERING_MASK, CONTROL_STEERING_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Control_Steering(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, CONTROL_STEERING_MASK, CONTROL_STEERING_OFFSET);
}
#else
void RAMN_Encode_Control_Steering(uint16_t value, uint8_t* payload) {
    /* SPN 2928 (Steering Wheel Angle). Bytes 1-2. PGN 61451 (ESC1). */
    memset(payload, 0xFF, 8);
    payload[0] = (uint8_t)(value & 0xFF);
    payload[1] = (uint8_t)((value >> 8) & 0xFF);
}

uint16_t RAMN_Decode_Control_Steering(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    return (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
}
#endif

// -- Command Shift --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Shift(uint8_t value, uint8_t* payload) {
    uint8_t packed = (uint8_t)PACK_SIGNAL(value, COMMAND_SHIFT_MASK, COMMAND_SHIFT_OFFSET);
    payload[0] = packed;
}

uint8_t RAMN_Decode_Command_Shift(const uint8_t* payload, uint32_t dlc) {
    uint8_t packed = payload[0];
    return (uint8_t)UNPACK_SIGNAL(packed, COMMAND_SHIFT_MASK, COMMAND_SHIFT_OFFSET);
}
#else
void RAMN_Encode_Command_Shift(uint8_t value, uint8_t* payload) {
    /* SPN 525 (Transmission Requested Gear). Byte 3. PGN 256 (TC1). */
    memset(payload, 0xFF, 8);
    payload[2] = (uint8_t)(value + 125);
}

uint8_t RAMN_Decode_Command_Shift(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 3U) return 0;
    return (uint8_t)(payload[2] - 125);
}
#endif

// -- Control Shift & Joystick (Combined) --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Shift_Joystick(uint8_t shift_value, uint8_t joystick_value, uint8_t* payload) {
    memset(payload, 0xFF, 8);
    uint16_t packed = 0xFFFF;
    packed &= ~((uint16_t)(CONTROL_SHIFT_MASK << CONTROL_SHIFT_OFFSET));
    packed |= (uint16_t)PACK_SIGNAL(shift_value, CONTROL_SHIFT_MASK, CONTROL_SHIFT_OFFSET);
    packed &= ~((uint16_t)(JOYSTICK_MASK << JOYSTICK_OFFSET));
    packed |= (uint16_t)PACK_SIGNAL(joystick_value, JOYSTICK_MASK, JOYSTICK_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint8_t RAMN_Decode_Control_Shift(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    if (dlc >= 2U) {
        RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
        return (uint8_t)UNPACK_SIGNAL(packed, CONTROL_SHIFT_MASK, CONTROL_SHIFT_OFFSET);
    }
    return 0;
}

uint8_t RAMN_Decode_Joystick(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    if (dlc >= 2U) {
        RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
        return (uint8_t)UNPACK_SIGNAL(packed, JOYSTICK_MASK, JOYSTICK_OFFSET);
    }
    return 0;
}
#else
void RAMN_Encode_Control_Shift_Joystick(uint8_t shift_value, uint8_t joystick_value, uint8_t* payload) {
    /* SPN 523 and SPN 162. Bytes 4 and 5. PGN 61445 (ETC2). */
    memset(payload, 0xFF, 8);
    payload[3] = (uint8_t)(shift_value + 125);
    payload[4] = (uint8_t)(joystick_value + 125);
}

uint8_t RAMN_Decode_Control_Shift(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 4U) return 0;
    return (uint8_t)(payload[3] - 125);
}

uint8_t RAMN_Decode_Joystick(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 5U) return 0;
    return (uint8_t)(payload[4] - 125);
}
#endif

// -- Command Horn --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Horn(uint8_t value, uint8_t* payload) {
    uint8_t packed = (uint8_t)PACK_SIGNAL(value, COMMAND_HORN_MASK, COMMAND_HORN_OFFSET);
    payload[0] = packed;
}

uint8_t RAMN_Decode_Command_Horn(const uint8_t* payload, uint32_t dlc) {
    uint8_t packed = payload[0];
    return (uint8_t)UNPACK_SIGNAL(packed, COMMAND_HORN_MASK, COMMAND_HORN_OFFSET);
}
#else
void RAMN_Encode_Command_Horn(uint8_t value, uint8_t* payload) {
    /* PGN 61184 (Proprietary A). Byte 1. */
    memset(payload, 0xFF, 8);
    payload[0] = (uint8_t)(value & 0xFF);
}

uint8_t RAMN_Decode_Command_Horn(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 1U) return 0;
    return payload[0];
}
#endif

// -- Control Horn --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Horn(uint8_t value, uint8_t* payload) {
    uint8_t packed = (uint8_t)PACK_SIGNAL(value, CONTROL_HORN_MASK, CONTROL_HORN_OFFSET);
    payload[0] = packed;
}

uint8_t RAMN_Decode_Control_Horn(const uint8_t* payload, uint32_t dlc) {
    uint8_t packed = payload[0];
    return (uint8_t)UNPACK_SIGNAL(packed, CONTROL_HORN_MASK, CONTROL_HORN_OFFSET);
}
#else
void RAMN_Encode_Control_Horn(uint8_t value, uint8_t* payload) {
    /* PGN 65098 (Secondary Air / Horn Status). SPN 3832. Bytes 1-2. */
    memset(payload, 0xFF, 8);
    payload[0] = (uint8_t)(value & 0xFF);
    payload[1] = (uint8_t)((value >> 8) & 0xFF);
}

uint8_t RAMN_Decode_Control_Horn(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    return (uint8_t)(payload[0] | ((uint16_t)payload[1] << 8));
}
#endif

// -- Command Turn Indicator --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_TurnIndicator(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, COMMAND_TURNINDICATOR_MASK, COMMAND_TURNINDICATOR_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Command_TurnIndicator(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, COMMAND_TURNINDICATOR_MASK, COMMAND_TURNINDICATOR_OFFSET);
}
#else
void RAMN_Encode_Command_TurnIndicator(uint16_t value, uint8_t* payload) {
    /* SPN 2876 (Turn Signal Switch). Byte 2 (bits 8-11). PGN 64972 (OEL). */
    memset(payload, 0xFF, 8);
    payload[1] = (payload[1] & 0xF0) | (uint8_t)(value & 0x0F);
}

uint16_t RAMN_Decode_Command_TurnIndicator(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 2U) return 0;
    return (uint16_t)(payload[1] & 0x0F);
}
#endif

// -- Command Sidebrake --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Sidebrake(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, COMMAND_SIDEBRAKE_MASK, COMMAND_SIDEBRAKE_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Command_Sidebrake(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, COMMAND_SIDEBRAKE_MASK, COMMAND_SIDEBRAKE_OFFSET);
}
#else
void RAMN_Encode_Command_Sidebrake(uint16_t value, uint8_t* payload) {
    /* SPN 70 (Parking Brake Switch). Byte 1 (bits 2-3). PGN 65265 (CCVS1). */
    memset(payload, 0xFF, 8);
    payload[0] = (payload[0] & 0xF3) | (uint8_t)((value & 0x03) << 2);
}

uint16_t RAMN_Decode_Command_Sidebrake(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 1U) return 0;
    return (uint16_t)((payload[0] >> 2) & 0x03);
}
#endif

// -- Control Sidebrake --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Sidebrake(uint8_t value, uint8_t* payload) {
    uint8_t packed = (uint8_t)PACK_SIGNAL(value, CONTROL_SIDEBRAKE_MASK, CONTROL_SIDEBRAKE_OFFSET);
    payload[0] = packed;
}

uint8_t RAMN_Decode_Control_Sidebrake(const uint8_t* payload, uint32_t dlc) {
    uint8_t packed = payload[0];
    return (uint8_t)UNPACK_SIGNAL(packed, CONTROL_SIDEBRAKE_MASK, CONTROL_SIDEBRAKE_OFFSET);
}
#else
void RAMN_Encode_Control_Sidebrake(uint8_t value, uint8_t* payload) {
    /* SPN 619 (Parking Brake Actuator Control Command). Byte 1 (bits 0-1). PGN 512 (EBS1). */
    memset(payload, 0xFF, 8);
    payload[0] = (payload[0] & 0xFC) | (uint8_t)(value & 0x03);
}

uint8_t RAMN_Decode_Control_Sidebrake(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 1U) return 0;
    return payload[0] & 0x03;
}
#endif

// -- Control Engine Key --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_EngineKey(uint8_t value, uint8_t* payload) {
    uint8_t packed = (uint8_t)PACK_SIGNAL(value, CONTROL_ENGINEKEY_MASK, CONTROL_ENGINEKEY_OFFSET);
    payload[0] = packed;
}

uint8_t RAMN_Decode_Control_EngineKey(const uint8_t* payload, uint32_t dlc) {
    uint8_t packed = payload[0];
    return (uint8_t)UNPACK_SIGNAL(packed, CONTROL_ENGINEKEY_MASK, CONTROL_ENGINEKEY_OFFSET);
}
#else
void RAMN_Encode_Control_EngineKey(uint8_t value, uint8_t* payload) {
    /* SPN 3046 (Ignition Switch). Byte 1 (bits 0-1). PGN 64960. */
    memset(payload, 0xFF, 8);
    payload[0] = (payload[0] & 0xFC) | (uint8_t)((value != 0U) ? 1U : 0U);
}

uint8_t RAMN_Decode_Control_EngineKey(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 1U) return 0;
    return payload[0] & 0x03;
}
#endif

// -- Command Lights --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Command_Lights(uint16_t value, uint8_t* payload) {
    uint16_t packed = PACK_SIGNAL(value, COMMAND_LIGHTS_MASK, COMMAND_LIGHTS_OFFSET);
    RAMN_memcpy(payload, (uint8_t*)&packed, sizeof(packed));
}

uint16_t RAMN_Decode_Command_Lights(const uint8_t* payload, uint32_t dlc) {
    uint16_t packed = 0;
    RAMN_memcpy((uint8_t*)&packed, payload, sizeof(packed));
    if (dlc <= 1U) packed = packed & 0xFF;
    return UNPACK_SIGNAL(packed, COMMAND_LIGHTS_MASK, COMMAND_LIGHTS_OFFSET);
}
#else
void RAMN_Encode_Command_Lights(uint16_t value, uint8_t* payload) {
    /* PGN 65089 (Lighting Command). Byte 1. */
    memset(payload, 0xFF, 8);
    payload[0] = (uint8_t)(value & 0xFF);
}

uint16_t RAMN_Decode_Command_Lights(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 1U) return 0;
    return (uint16_t)payload[0];
}
#endif

// -- Control Lights --
#ifndef ENABLE_J1939_MODE
void RAMN_Encode_Control_Lights(uint8_t value, uint8_t* payload) {
    uint8_t packed = (uint8_t)PACK_SIGNAL(value, CONTROL_LIGHTS_MASK, CONTROL_LIGHTS_OFFSET);
    payload[0] = packed;
}

uint8_t RAMN_Decode_Control_Lights(const uint8_t* payload, uint32_t dlc) {
    uint8_t packed = payload[0];
    return (uint8_t)UNPACK_SIGNAL(packed, CONTROL_LIGHTS_MASK, CONTROL_LIGHTS_OFFSET);
}
#else
void RAMN_Encode_Control_Lights(uint8_t value, uint8_t* payload) {
    /* SPN 2872 (Main Light Switch). Byte 1 (bits 4-7). PGN 64972 (OEL). */
    memset(payload, 0xFF, 8);
    payload[0] = (payload[0] & 0x0F) | (uint8_t)((value & 0x0F) << 4);
}

uint8_t RAMN_Decode_Control_Lights(const uint8_t* payload, uint32_t dlc) {
    if (dlc < 1U) return 0;
    return (uint8_t)((payload[0] >> 4) & 0x0F);
}
#endif
