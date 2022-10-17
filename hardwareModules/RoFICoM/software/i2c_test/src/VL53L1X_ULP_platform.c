/*
 Copyright (c) 2021, STMicroelectronics - All Rights Reserved

 This file : part of VL53L1X ULP and : dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L1X ULP may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

#include <stm32g0xx_ll_i2c.h>
#include <stm32g0xx_ll_utils.h> // LL_mDelay

#include "VL53L1X_ULP_platform.h"

static uint8_t _I2C_RegisterAdress(const uint32_t slaveAdress, const uint16_t RegisterAdress)
{
	const uint8_t adress[2] = { RegisterAdress >> 8, RegisterAdress & 0xFF };

	LL_I2C_HandleTransfer(I2C2, slaveAdress, LL_I2C_ADDRSLAVE_7BIT, sizeof(adress), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

    uint8_t i = 0;

    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_TXIS( I2C2 ) ) {
            LL_I2C_TransmitData8( I2C2, adress[i] );
        ++i;
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );

	return 0;
}

static uint8_t _I2C_Write(const uint32_t slaveAdress, const uint16_t RegisterAdress, const uint8_t * transmitBuffer, const uint32_t bufferSize)
{
	uint8_t status = _I2C_RegisterAdress(slaveAdress, RegisterAdress);
	if (status != 0) return status;	
	
	LL_I2C_HandleTransfer( I2C2, slaveAdress, LL_I2C_ADDRSLAVE_7BIT, bufferSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

    uint8_t i = 0;

    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_TXIS( I2C2 ) ) {
            LL_I2C_TransmitData8( I2C2, transmitBuffer[i] );
        ++i;
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );

	return 0;
}

static uint8_t _I2C_Read(const uint32_t slaveAddress, const uint16_t RegisterAdress, uint8_t * transmitBuffer, const uint32_t transmitSize)
{
	uint8_t status = _I2C_RegisterAdress(slaveAddress, RegisterAdress);
	if (status != 0) return status;
	
	// Actual read
	LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, transmitSize, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ );

    uint8_t i = 0;
    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_RXNE( I2C2 ) ) {
            transmitBuffer[i] = LL_I2C_ReceiveData8( I2C2 );
			++i;
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );

	return 0;
}

uint8_t VL53L1X_ULP_RdDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t transmitBuffer[4] = { 0, 0, 0, 0 };
	uint8_t status = _I2C_Read(dev, RegisterAdress, transmitBuffer, sizeof(transmitBuffer));

	*value = transmitBuffer[0] << 24;
	*value += transmitBuffer[1] << 16;
	*value += transmitBuffer[2] << 8;
	*value += transmitBuffer[3];
	
	return status;
}

uint8_t VL53L1X_ULP_RdWord(uint16_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t transmitBuffer[2] = { 0, 0 };
	uint8_t status = _I2C_Read(dev, RegisterAdress, transmitBuffer, sizeof(transmitBuffer));

	*value = transmitBuffer[0] << 8;
	*value += transmitBuffer[1];
	
	return status;
}

uint8_t VL53L1X_ULP_RdByte(uint16_t dev, uint16_t RegisterAdress, uint8_t *value)
{	
	return _I2C_Read(dev, RegisterAdress, value, 1);
}

uint8_t VL53L1X_ULP_WrByte(uint16_t dev, uint16_t RegisterAdress, uint8_t value)
{	
	return _I2C_Write(dev, RegisterAdress, &value, 1);
}

uint8_t VL53L1X_ULP_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value)
{
	// uint8_t status = 255;
	uint8_t transmitBuffer[2] = { value >> 8, value & 0xFF };
	
	return _I2C_Write(dev, RegisterAdress, transmitBuffer, sizeof(transmitBuffer));
}

uint8_t VL53L1X_ULP_WrDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t transmitBuffer[4] = { value >> 24, value >> 16, value >> 8, value & 0xFF };
	
	return _I2C_Write(dev, RegisterAdress, transmitBuffer, sizeof(transmitBuffer));
}

void VL53L1X_ULP_WaitMs(uint32_t TimeMs)
{
	LL_mDelay(TimeMs);
}
