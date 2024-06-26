/*
 * ramn_utils.h
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 TOYOTA MOTOR CORPORATION.
 * ALL RIGHTS RESERVED.</center></h2>
 *
 * This software component is licensed by TOYOTA MOTOR CORPORATION under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

// This module implements common tools and definitions

#ifndef INC_RAMN_UTILS_H_
#define INC_RAMN_UTILS_H_

#include "main.h"

//Definition for Common operations
typedef enum
{
	False       = 0x00,
	True        = 0x01,
} RAMN_Bool_t;

typedef enum
{
	RAMN_OK        = 0x00,
	RAMN_ERROR     = 0x01,
	RAMN_TRY_LATER = 0x02,
} RAMN_Result_t;

//Functions to convert from to uint ASCII. Returns number of bytes written
uint32_t  rawtoASCII(uint8_t* dst, const uint8_t* src, uint32_t raw_size);
uint32_t  uint32toASCII(uint32_t src, uint8_t* dst);
uint32_t  uint16toASCII(uint16_t src, uint8_t* dst);
uint32_t  uint12toASCII(uint16_t src, uint8_t* dst);
uint32_t  uint8toASCII(uint8_t src, uint8_t* dst);
uint32_t  uint4toASCII(uint8_t src, uint8_t* dst);

//Functions to convert from ASCII to uint
void     ASCIItoRaw(uint8_t* dst, const uint8_t* src, uint32_t raw_size);
uint8_t  ASCIItoUint4 (const uint8_t* src);
uint8_t  ASCIItoUint8 (const uint8_t* src);
uint16_t ASCIItoUint16(const uint8_t* src);
uint16_t ASCIItoUint12(const uint8_t* src);
uint32_t ASCIItoUint32(const uint8_t* src);

//Functions to convert STM32 HAL FDCAN DLC format (uint32 enumeration) to actual payload size (0 to 64)
uint8_t  DLCtoUINT8(uint32_t dlc_enum);
uint32_t UINT8toDLC(uint8_t dlc);

//Regular memcpy operation
void RAMN_memcpy(uint8_t* dst, const uint8_t* src, uint32_t size);

//Apply required endian
uint16_t switchEndian16(uint16_t val);

#endif /* INC_RAMN_UTILS_H_ */
