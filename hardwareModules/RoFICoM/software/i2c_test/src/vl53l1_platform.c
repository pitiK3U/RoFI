
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include <stm32g0xx_ll_i2c.h>
#include <stm32g0xx_ll_utils.h> // LL_mDelay

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

static uint8_t _I2C_RegisterAdress(const uint32_t slaveAddress, const uint16_t RegisterAddress)
{
	const uint8_t address[2] = { RegisterAddress >> 8, RegisterAddress & 0xFF };

	LL_I2C_HandleTransfer(I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, sizeof(address), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

    uint8_t i = 0;

    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_TXIS( I2C2 ) ) {
            LL_I2C_TransmitData8( I2C2, address[i] );
        ++i;
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );

	return 0;
}

int8_t VL53L1_WriteMulti( uint16_t slaveAddress, uint16_t RegisterAddress, uint8_t *transmitBuffer, uint32_t bufferSize) {
	uint8_t status = _I2C_RegisterAdress( slaveAddress, RegisterAddress);
	if (status != 0) return status;	
	
	LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, bufferSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

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

int8_t VL53L1_ReadMulti(uint16_t slaveAddress, uint16_t RegisterAddress, uint8_t *transmitBuffer, uint32_t bufferSize){
	uint8_t status = _I2C_RegisterAdress(slaveAddress, RegisterAddress);
	if (status != 0) return status;
	
	// Actual read
	LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, bufferSize, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ );

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

int8_t VL53L1_WrByte(uint16_t dev, uint16_t registerAddress, uint8_t data) {
	return VL53L1_WriteMulti(dev, registerAddress, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t registerAddress, uint16_t data) {
	uint8_t transmitBuffer[2] = { data >> 8, data & 0xFF };
	
	return VL53L1_WriteMulti(dev, registerAddress, transmitBuffer, sizeof(transmitBuffer));}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t registerAddress, uint32_t data) {
	uint8_t transmitBuffer[4] = { data >> 24, data >> 16, data >> 8, data & 0xFF };
	
	return VL53L1_WriteMulti(dev, registerAddress, transmitBuffer, sizeof(transmitBuffer));
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t registerAddress, uint8_t *data) {
	uint8_t status = VL53L1_ReadMulti(dev, registerAddress, data, 1);
	
	return status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t registerAddress, uint16_t *data) {
	uint8_t transmitBuffer[2] = { 0, 0, };
	uint8_t status = VL53L1_ReadMulti(dev, registerAddress, transmitBuffer, sizeof(transmitBuffer));

	*data += transmitBuffer[0] << 8;
	*data += transmitBuffer[1];
	
	return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t registerAddress, uint32_t *data) {
	uint8_t transmitBuffer[4] = { 0, 0, 0, 0 };
	uint8_t status = VL53L1_ReadMulti(dev, registerAddress, transmitBuffer, sizeof(transmitBuffer));

	*data = transmitBuffer[0] << 24;
	*data += transmitBuffer[1] << 16;
	*data += transmitBuffer[2] << 8;
	*data += transmitBuffer[3];
	
	return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	LL_mDelay(wait_ms);

	return 0;
}
