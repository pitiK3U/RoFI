
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

#include <array>
#include <functional>

#include <stm32g0xx_ll_i2c.h>
#include <stm32g0xx_ll_gpio.h>
#include <stm32g0xx_ll_exti.h>
#include <stm32g0xx_ll_utils.h> // LL_mDelay

#include <drivers/gpio.hpp>
#include <drivers/i2c.hpp>
#include <drivers/timer.hpp>

#include "vl53l1_platform.h"

#include <string.h>
#include <time.h>
#include <math.h>

#include <lidar.hpp>

// Anonymous namespace is to hide symbols only into this compilation unit.
namespace {
// static Gpio::Pin INTPIN = Gpio( GPIOB )[ 0 ];
static Timer microTimer( TIM2, FreqAndRes( 1000000, UINT16_MAX ) );
I2C* pI2c = nullptr;
}

void _inner::initialize_platform( I2C* i2cPeriph ) {
	pI2c = i2cPeriph;
}

VL53L1_Error VL53L1_CommsInitialise(
	VL53L1_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz) {
	// Note: init is done in main.cpp `setupI2C`
	SUPPRESS_UNUSED_WARNING(pdev);
	SUPPRESS_UNUSED_WARNING(comms_type);
	SUPPRESS_UNUSED_WARNING(comms_speed_khz);

    microTimer.enable();

	
	// GPIO Ports Clock Enable 
	// LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	// INTPIN.port().enableClock();
	/*
	// Setup PB0 for lidar interrupt
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	*/
	// LL_IOP_GRP1_EnableClock( LL_IOP_GRP1_PERIPH_GPIOF );
	/* INTPIN.port().enableClock();
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
	INTPIN.setupPPOutput();
	*/

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev) {
	SUPPRESS_UNUSED_WARNING(pdev);

	microTimer.disable();

	// Return NRST pin to control of reseting the mcu
	// LL_GPIO_ResetOutputPin( GPIOF, LL_GPIO_PIN_2 );
    // LL_GPIO_DeInit( GPIOF );
	// // LL_IOP_GRP1_DisableClock( LL_IOP_GRP1_PERIPH_GPIOF )
	// INTPIN.port().disableClock();

	return VL53L1_ERROR_NONE;
}

/**
 * BUG: This is only usefull for `VL53L1_ReadMulti` since this is handled with autoend.
*/
static uint8_t _I2C_RegisterAdress( const VL53L1_Dev_t * pdev, const uint16_t RegisterAddress )
{
	const uint16_t slaveAddress = pdev->i2c_slave_address;
	const uint8_t address[2] = {
		static_cast<uint8_t>( RegisterAddress >> 8 ),
		static_cast<uint8_t>( RegisterAddress & 0xFF )
	};

	LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, sizeof(address), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

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

template< std::size_t N >
static VL53L1_Error _WriteMulti( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint8_t *transmitBuffer ) {
	const uint16_t slaveAddress = pdev->i2c_slave_address;

	// NOTE: register address must be sent with data in one transaction 
	const uint8_t address[2] = {
		static_cast<uint8_t>( registerAddress >> 8 ),
		static_cast<uint8_t>( registerAddress & 0xFF )
	};

	std::array< uint8_t, sizeof( address ) + N > buffer = { address[0], address[1] };
	std::copy( transmitBuffer, transmitBuffer + N, std::next( buffer.begin(), sizeof( address ) ) );

	assert( pI2c );

	pI2c->write( slaveAddress, buffer );

	return 0;
}

template< std::size_t N >
static VL53L1_Error _ReadMulti( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint8_t *transmitBuffer ) {
	const uint16_t slaveAddress = pdev->i2c_slave_address;

	_WriteMulti< 0 >( pdev, registerAddress, nullptr );

	assert( pI2c );
	auto result = pI2c->read< std::array< uint8_t, N > >( slaveAddress, N );
	std::copy( result.begin(), result.end(), transmitBuffer );

	return 0;
}

// FIX: `VL53L1_WriteMulti` and `VL53L1_ReadMulti` are C function thus they are declared to use runtime transfer size.
// 		However the I2C driver uses compile time size to enure enough data can be hold with type safety.
VL53L1_Error VL53L1_WriteMulti( VL53L1_Dev_t * pdev, uint16_t RegisterAddress, uint8_t *transmitBuffer, uint32_t bufferSize ) {
	const uint16_t slaveAddress = pdev->i2c_slave_address;

	// NOTE: register address must be sent with data in one transaction 
	const uint8_t address[2] = {
		static_cast<uint8_t>( RegisterAddress >> 8 ),
		static_cast<uint8_t>( RegisterAddress & 0xFF )
	};

	LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, bufferSize + sizeof( address ), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

    uint8_t i = 0;

    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_TXIS( I2C2 ) ) {
			uint8_t data = i < 2 ? address[i] : transmitBuffer[i-2];
            LL_I2C_TransmitData8( I2C2, data );
        ++i;
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );

	return 0;
}

VL53L1_Error VL53L1_ReadMulti( VL53L1_Dev_t * pdev, uint16_t RegisterAddress, uint8_t *transmitBuffer, uint32_t bufferSize ) {
	const uint16_t slaveAddress = pdev->i2c_slave_address;
	uint8_t status = _I2C_RegisterAdress( pdev, RegisterAddress );
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

VL53L1_Error VL53L1_WrByte( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint8_t data ) {
	return _WriteMulti< 1 >( pdev, registerAddress, &data );
}

VL53L1_Error VL53L1_WrWord( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint16_t data ) {
	uint8_t transmitBuffer[2] = {
		static_cast<uint8_t>( data >> 8 ),
		static_cast<uint8_t>( data & 0xFF )
	};
	
	return _WriteMulti< sizeof(transmitBuffer) >( pdev, registerAddress, transmitBuffer );
}

VL53L1_Error VL53L1_WrDWord( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint32_t data ) {
	uint8_t transmitBuffer[4] = {
		static_cast<uint8_t>( data >> 24 ),
		static_cast<uint8_t>( data >> 16 ),
		static_cast<uint8_t>( data >> 8 ),
		static_cast<uint8_t>( data & 0xFF )
	};
	
	return _WriteMulti< sizeof(transmitBuffer) >( pdev, registerAddress, transmitBuffer );
}

VL53L1_Error VL53L1_RdByte( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint8_t * data ) {
	uint8_t status = _ReadMulti< 1 >( pdev, registerAddress, data );

	return status;
}

VL53L1_Error VL53L1_RdWord( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint16_t * data ) {
	uint16_t slaveAddress = pdev->i2c_slave_address;
	uint8_t transmitBuffer[2] = { 0, 0 };
	uint8_t status = _ReadMulti< sizeof( transmitBuffer ) >( pdev, registerAddress, transmitBuffer );

	*data = transmitBuffer[0] << 8;
	*data += transmitBuffer[1];
	
	return status;
}

VL53L1_Error VL53L1_RdDWord( VL53L1_Dev_t * pdev, uint16_t registerAddress, uint32_t * data) {
	uint8_t transmitBuffer[4] = { 0, 0, 0, 0 };
	uint8_t status = _ReadMulti< sizeof( transmitBuffer ) >( pdev, registerAddress, transmitBuffer );

	*data = transmitBuffer[0] << 24;
	*data += transmitBuffer[1] << 16;
	*data += transmitBuffer[2] << 8;
	*data += transmitBuffer[3];
	
	return status;
}

VL53L1_Error VL53L1_WaitMs( VL53L1_Dev_t* dev, int32_t wait_ms ) {
	LL_mDelay(wait_ms);

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(
		VL53L1_Dev_t *pdev,
		int32_t       wait_us) {

	uint16_t start = microTimer.counter();

	while( ( microTimer.counter() - start ) < wait_us ) {};

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
		VL53L1_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	
	uint32_t timeout_timer = 0;
	uint8_t read_val;


	while ( ( status |= VL53L1_RdByte( pdev, index, &read_val ) ) == VL53L1_ERROR_NONE
			&& (read_val & mask) != value
			&& timeout_timer < timeout_ms ) {
		VL53L1_WaitMs( pdev, poll_delay_ms );
		timeout_timer += poll_delay_ms;
	}

	if ( status != VL53L1_ERROR_NONE )
		return status;

	if (timeout_timer >= timeout_ms )
		status |= VL53L1_ERROR_TIME_OUT;

	return status;
}

/**
 * Since VL53L1 is not wired to the mcu following functions are empty
 */

VL53L1_Error VL53L1_GpioXshutdown( uint8_t value ) {
	// INTPIN.write( static_cast< bool >( value ) );

	return VL53L1_ERROR_NONE;
}
VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value) { return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioPowerEnable(uint8_t value) { return VL53L1_ERROR_NONE; }

VL53L1_Error VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type) { 
	// NVIC_SetPriority( irq, 1 );
    // NVIC_EnableIRQ( irq );
	// std::function<void(bool)> fun([=](bool _) -> void { function(); } );
	// PIN.setupInterrupt( edge_type, fun );

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptDisable(void) {
	//LL_EXTI_DisableIT_0_31( LL_EXTI_LINE_0 );
	// PIN.disableInterrupt();

	return VL53L1_ERROR_NONE;
}
