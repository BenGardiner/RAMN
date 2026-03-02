/*
 * ramn_actuators.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 TOYOTA MOTOR CORPORATION.
 * ALL RIGHTS RESERVED.</center></h2>
 *
 * This software component is licensed by TOYOTA MOTOR CORPORATION under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "ramn_actuators.h"
#include "ramn_signal_defs.h"

#ifdef EXPANSION_BODY
// Byte that store the state of each LED of ECU D.
static uint8_t LEDState;
#endif

#if (LED_TEST_DURATION_MS > 0U)
// Bool set to 1 when the LED Test over is over. Used to avoid redoing the test on SysTick overflow.
static RAMN_Bool_t LEDTestOver = False;
#endif

void RAMN_ACTUATORS_Init(void)
{
#ifdef EXPANSION_BODY
	LEDState = (uint8_t)(RAMN_DBC_Handle.control_lights&0xFF);
	RAMN_SPI_UpdateLED(&LEDState);
#endif
}

void RAMN_ACTUATORS_SetLampState(uint8_t mask, uint8_t val)
{
	if (val != 0U) RAMN_DBC_Handle.control_lights |= mask;
	else RAMN_DBC_Handle.control_lights &= ~mask;
}

void RAMN_ACTUATORS_ApplyControls(uint32_t tick)
{
#if defined(EXPANSION_CHASSIS) //CHASSIS
	uint16_t payload;

	payload = PACK_SIGNAL((uint16_t)RAMN_DBC_Handle.control_steer, CONTROL_STEERING_MASK, CONTROL_STEERING_OFFSET);
	RAMN_memcpy(&(msg_control_steering.data->rawData[CAN_SIM_CONTROL_STEERING_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL(RAMN_DBC_Handle.control_sidebrake, CONTROL_SIDEBRAKE_MASK, CONTROL_SIDEBRAKE_OFFSET);
	RAMN_memcpy(&(msg_control_sidebrake.data->rawData[CAN_SIM_CONTROL_SIDEBRAKE_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL(RAMN_DBC_Handle.command_lights, COMMAND_LIGHTS_MASK, COMMAND_LIGHTS_OFFSET);
	RAMN_memcpy(&(msg_command_lights.data->rawData[CAN_SIM_COMMAND_LIGHTS_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

#elif defined(EXPANSION_POWERTRAIN) //POWERTRAIN
	uint16_t payload;

	payload = PACK_SIGNAL((uint16_t)RAMN_DBC_Handle.control_brake, CONTROL_BRAKE_MASK, CONTROL_BRAKE_OFFSET);
	RAMN_memcpy(&(msg_control_brake.data->rawData[CAN_SIM_CONTROL_BRAKE_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL((uint16_t)RAMN_DBC_Handle.control_accel, CONTROL_ACCEL_MASK, CONTROL_ACCEL_OFFSET);
	RAMN_memcpy(&(msg_control_accel.data->rawData[CAN_SIM_CONTROL_ACCEL_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL(RAMN_DBC_Handle.control_shift, CONTROL_SHIFT_MASK, CONTROL_SHIFT_OFFSET) |
	          PACK_SIGNAL(RAMN_DBC_Handle.joystick, JOYSTICK_MASK, JOYSTICK_OFFSET);
	RAMN_memcpy(&(msg_control_shift.data->rawData[CAN_SIM_CONTROL_SHIFT_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL(RAMN_DBC_Handle.command_horn, COMMAND_HORN_MASK, COMMAND_HORN_OFFSET);
	RAMN_memcpy(&(msg_command_horn.data->rawData[CAN_SIM_COMMAND_HORN_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL(RAMN_DBC_Handle.command_turnindicator, COMMAND_TURNINDICATOR_MASK, COMMAND_TURNINDICATOR_OFFSET);
	RAMN_memcpy(&(msg_command_turnindicator.data->rawData[CAN_SIM_COMMAND_TURNINDICATOR_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

#elif defined(EXPANSION_BODY) //BODY
	uint16_t payload;

	payload = PACK_SIGNAL(RAMN_DBC_Handle.control_enginekey, CONTROL_ENGINEKEY_MASK, CONTROL_ENGINEKEY_OFFSET);
	RAMN_memcpy(&(msg_control_enginekey.data->rawData[CAN_SIM_CONTROL_ENGINEKEY_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

	payload = PACK_SIGNAL(RAMN_DBC_Handle.control_lights, CONTROL_LIGHTS_MASK, CONTROL_LIGHTS_OFFSET);
	RAMN_memcpy(&(msg_control_lights.data->rawData[CAN_SIM_CONTROL_LIGHTS_PAYLOAD_OFFSET / 8]), (uint8_t*)&payload, sizeof(payload));

#if (LED_TEST_DURATION_MS > 0)

	if((tick < LED_TEST_DURATION_MS) && (LEDTestOver == False)) RAMN_DBC_Handle.control_lights = 0xFF;
	else LEDTestOver = True;

#endif
	RAMN_SPI_UpdateLED((uint8_t*)&(RAMN_DBC_Handle.control_lights));
#endif
}
