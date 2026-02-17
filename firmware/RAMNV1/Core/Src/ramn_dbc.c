/*
 * dbc.c
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

#include "ramn_dbc.h"
#include "ramn_signal_defs.h"

#define NUMBER_OF_PERIODIC_MSG (sizeof(periodicTxCANMsgs)/sizeof(RAMN_PeriodicFDCANTx_t*))

volatile RAMN_Bool_t RAMN_DBC_RequestSilence = True;

RAMN_DBC_Handle_t RAMN_DBC_Handle = {.command_steer = 0x7FF, .control_shift =0x01, .command_shift = 0x01};

// array that holds messages to be sent periodically
static RAMN_PeriodicFDCANTx_t* periodicTxCANMsgs[] = {
#if defined(TARGET_ECUA)
		&msg_command_brake,&msg_command_accel,&msg_status_RPM,&msg_command_steering,&msg_command_shift,&msg_control_horn,&msg_command_parkingbrake
#endif
#if defined(TARGET_ECUB)
		&msg_control_steering, &msg_control_sidebrake, &msg_command_lights
#endif
#if defined(TARGET_ECUC)
		&msg_control_brake, &msg_control_accel, &msg_control_shift, &msg_command_horn, &msg_command_turnindicator
#endif
#if defined(TARGET_ECUD)
		&msg_control_enginekey, &msg_control_lights
#endif
};

// Function that formats messages with counter/checksum/random/etc.
static void RAMN_DBC_FormatDefaultPeriodicMessage(RAMN_PeriodicFDCANTx_t* msg)
{
	msg->data->ramnData.counter = applyEndian16(msg->counter);
	msg->data->ramnData.crc32 = RAMN_CRC_SoftCalculate(msg->data->rawData,4U);
	msg->header.ErrorStateIndicator = RAMN_FDCAN_Status.ErrorStateIndicator;
}

void RAMN_DBC_Init(void)
{
#if defined(TARGET_ECUA)
	RAMN_DBC_RequestSilence = True;
#else
	RAMN_DBC_RequestSilence = False;
#endif

}

void RAMN_DBC_ProcessCANMessage(uint32_t canid, uint32_t dlc, RAMN_CANFrameData_t* dataframe)
{
	// Ignore fields other than useful data
	dataframe->ramnData.payload = dataframe->ramnData.payload&0xFFFF;
	if (dlc <= 1U) dataframe->ramnData.payload = dataframe->ramnData.payload&0xFF; //TODO: reject dlc == 2U message instead of casting them?

	if (dlc != 0U)
	{
		switch(canid)
		{
		case CAN_SIM_CONTROL_BRAKE_CANID:
			RAMN_DBC_Handle.control_brake 				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_BRAKE_MASK, CONTROL_BRAKE_OFFSET));
			break;
		case CAN_SIM_COMMAND_BRAKE_CANID:
			RAMN_DBC_Handle.command_brake 				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_BRAKE_MASK, COMMAND_BRAKE_OFFSET));
			break;
		case CAN_SIM_CONTROL_ACCEL_CANID:
			RAMN_DBC_Handle.control_accel 				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_ACCEL_MASK, CONTROL_ACCEL_OFFSET));
			break;
		case CAN_SIM_COMMAND_ACCEL_CANID:
			RAMN_DBC_Handle.command_accel 				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_ACCEL_MASK, COMMAND_ACCEL_OFFSET));
			break;
		case CAN_SIM_STATUS_RPM_CANID:
			RAMN_DBC_Handle.status_rpm  				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, STATUS_RPM_MASK, STATUS_RPM_OFFSET));
			break;
		case CAN_SIM_CONTROL_STEERING_CANID:
			RAMN_DBC_Handle.control_steer 				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_STEERING_MASK, CONTROL_STEERING_OFFSET));
			break;
		case CAN_SIM_COMMAND_STEERING_CANID:
			RAMN_DBC_Handle.command_steer 				= applyEndian16(UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_STEERING_MASK, COMMAND_STEERING_OFFSET));
			break;
		case CAN_SIM_CONTROL_SHIFT_CANID:
			RAMN_DBC_Handle.control_shift				=  UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_SHIFT_MASK, CONTROL_SHIFT_OFFSET);
			if (dlc >= 2U)
			{
				RAMN_DBC_Handle.joystick					= UNPACK_SIGNAL(dataframe->ramnData.payload, JOYSTICK_MASK, JOYSTICK_OFFSET);
			#ifdef ENABLE_JOYSTICK_CONTROLS
				RAMN_Joystick_Update(RAMN_DBC_Handle.joystick);
			#endif
			}
			break;
		case CAN_SIM_COMMAND_SHIFT_CANID:
			RAMN_DBC_Handle.command_shift 				= UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_SHIFT_MASK, COMMAND_SHIFT_OFFSET);
			break;
		case CAN_SIM_COMMAND_HORN_CANID:
			RAMN_DBC_Handle.command_horn 				= UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_HORN_MASK, COMMAND_HORN_OFFSET);
			break;
		case CAN_SIM_CONTROL_HORN_CANID:
			RAMN_DBC_Handle.control_horn 				= UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_HORN_MASK, CONTROL_HORN_OFFSET);
			break;
		case CAN_SIM_CONTROL_SIDEBRAKE_CANID:
			RAMN_DBC_Handle.control_sidebrake 			= UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_SIDEBRAKE_MASK, CONTROL_SIDEBRAKE_OFFSET);
			break;
		case CAN_SIM_COMMAND_SIDEBRAKE_CANID:
			RAMN_DBC_Handle.command_sidebrake 			= UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_SIDEBRAKE_MASK, COMMAND_SIDEBRAKE_OFFSET);
			break;
		case CAN_SIM_COMMAND_TURNINDICATOR_CANID:
			RAMN_DBC_Handle.command_turnindicator		= UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_TURNINDICATOR_MASK, COMMAND_TURNINDICATOR_OFFSET);
			break;
		case CAN_SIM_CONTROL_ENGINEKEY_CANID:
			RAMN_DBC_Handle.control_enginekey 			= UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_ENGINEKEY_MASK, CONTROL_ENGINEKEY_OFFSET);
			break;
		case CAN_SIM_COMMAND_LIGHTS_CANID:
			RAMN_DBC_Handle.command_lights 				= UNPACK_SIGNAL(dataframe->ramnData.payload, COMMAND_LIGHTS_MASK, COMMAND_LIGHTS_OFFSET);
			break;
		case CAN_SIM_CONTROL_LIGHTS_CANID:
			RAMN_DBC_Handle.control_lights 				= UNPACK_SIGNAL(dataframe->ramnData.payload, CONTROL_LIGHTS_MASK, CONTROL_LIGHTS_OFFSET);
			break;
		default:
			break;
		}
	}
}

void RAMN_DBC_Send(uint32_t tick)
{
	for(uint16_t i = 0; i < NUMBER_OF_PERIODIC_MSG ; i++)
	{
		if((tick - periodicTxCANMsgs[i]->lastSent) >= periodicTxCANMsgs[i]->periodms)
		{
			RAMN_DBC_FormatDefaultPeriodicMessage(periodicTxCANMsgs[i]);
			RAMN_FDCAN_SendMessage(&(periodicTxCANMsgs[i]->header),(uint8_t*)(periodicTxCANMsgs[i]->data));
			periodicTxCANMsgs[i]->counter++;
			periodicTxCANMsgs[i]->lastSent = tick;
		}
	}
}

#if defined(ENABLE_USB)
void RAMN_DBC_ProcessUSBBuffer(const uint8_t* buf)
{
#if defined(TARGET_ECUA)
	msg_command_brake.data->ramnData.payload 			= applyEndian16(PACK_SIGNAL(ASCIItoUint12(&buf[1]), COMMAND_BRAKE_MASK, COMMAND_BRAKE_OFFSET));
	msg_command_accel.data->ramnData.payload 			= applyEndian16(PACK_SIGNAL(ASCIItoUint12(&buf[4]), COMMAND_ACCEL_MASK, COMMAND_ACCEL_OFFSET));
	msg_status_RPM.data->ramnData.payload 				= applyEndian16(PACK_SIGNAL(ASCIItoUint12(&buf[7]), STATUS_RPM_MASK, STATUS_RPM_OFFSET));
	msg_command_steering.data->ramnData.payload 		= applyEndian16(PACK_SIGNAL(ASCIItoUint12(&buf[10]), COMMAND_STEERING_MASK, COMMAND_STEERING_OFFSET));
	msg_command_shift.data->ramnData.payload 			= PACK_SIGNAL(ASCIItoUint8(&buf[13]), COMMAND_SHIFT_MASK, COMMAND_SHIFT_OFFSET);
	msg_control_horn.data->ramnData.payload 			= PACK_SIGNAL(ASCIItoUint8(&buf[15]), CONTROL_HORN_MASK, CONTROL_HORN_OFFSET);
	msg_command_parkingbrake.data->ramnData.payload 	= PACK_SIGNAL(ASCIItoUint8(&buf[17]), COMMAND_SIDEBRAKE_MASK, COMMAND_SIDEBRAKE_OFFSET);
#endif
}
#endif



