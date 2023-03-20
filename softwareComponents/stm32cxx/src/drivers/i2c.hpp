#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <ranges>

#include <system/assert.hpp>

#include <drivers/peripheral.hpp>
#include <drivers/gpio.hpp>

#include <stm32g0xx_ll_i2c.h>

struct I2CPin : public Gpio::Pin
{
    I2CPin( Gpio::Pin&& pin ) : Gpio::Pin( pin ) {}

    I2CPin& setupODAlternate( ) {
        setupODOutput( true, true );

        assert( 0 <= _pos && _pos <= 15 );

        auto pin = 1 << _pos;
        LL_GPIO_SetPinMode( _periph, pin, LL_GPIO_MODE_ALTERNATE );

        // TODO: create i2c.port.hpp file for setting up alternate function
        auto alternateFunction = LL_GPIO_AF_6;
        if ( pin <= 7 ) {
            LL_GPIO_SetAFPin_0_7( _periph, pin, alternateFunction );
        } else {
            LL_GPIO_SetAFPin_8_15( _periph, pin, alternateFunction );
        }

        return *this;
    }
};

struct SdaPin : public I2CPin
{
    using I2CPin::I2CPin;
};

struct SclPin : public I2CPin
{
    using I2CPin::I2CPin;
};

// TODO: transaction address 7bit vs 10bit
struct I2C: public Peripheral< I2C_TypeDef > {
    I2C( I2C_TypeDef *i2c, SdaPin sdaPin, SclPin sclPin )
        : Peripheral< I2C_TypeDef >( i2c )
    {
        sdaPin.setupODAlternate();
        sclPin.setupODAlternate();

        // TODO: enableClock();
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

        LL_I2C_InitTypeDef I2C_InitStruct = { };

        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        // TODO: set speed
        // This parameter is computed by STM32cubemx tool
        I2C_InitStruct.Timing = 0x00303D5B;
        I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
        I2C_InitStruct.DigitalFilter = 0;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

        LL_I2C_Init( _periph, &I2C_InitStruct );
        LL_I2C_EnableAutoEndMode( _periph );
        LL_I2C_SetOwnAddress2( _periph, 0, LL_I2C_OWNADDRESS2_NOMASK );
        LL_I2C_DisableOwnAddress2( _periph );
        LL_I2C_DisableGeneralCall( _periph );
        LL_I2C_EnableClockStretching( _periph );

        LL_I2C_EnableIT_ERR( _periph );
    }

    template < std::ranges::contiguous_range container >
    int write( const uint32_t peripheralAddress, const container& buffer ) {
        return _write( peripheralAddress, buffer.data(), buffer.size() );
    }

    template < std::ranges::contiguous_range container >
    int read( const uint32_t peripheralAddress, container& buffer ) {
        return _read( peripheralAddress, buffer.data(), buffer.size() );
    }

private:
    /**
     * Checks whether an error occured on the i2c peripheral.
     * 
     * NOTE: This function clears given error flag after checking it.
    */
    int checkError() {
        if ( LL_I2C_IsActiveFlag_NACK( _periph ) ) {
            LL_I2C_ClearFlag_NACK( _periph );
            return 1;
        }

        if ( LL_I2C_IsActiveFlag_BERR( _periph ) ) {
            LL_I2C_ClearFlag_BERR( _periph );
            return 2;
        }

         if ( LL_I2C_IsActiveFlag_ARLO( _periph ) ) {
            LL_I2C_ClearFlag_ARLO( _periph );
            return 3;
        }

        if ( LL_I2C_IsActiveFlag_OVR( _periph ) ) {
            LL_I2C_ClearFlag_OVR( _periph );
            return 4;
        }

        return 0;
    }

    int _write( const uint32_t peripheralAddress, const uint8_t* data, const uint32_t transferSize ) {
        LL_I2C_HandleTransfer( _periph, peripheralAddress, LL_I2C_ADDRSLAVE_7BIT, transferSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

        while( !LL_I2C_IsActiveFlag_STOP( _periph ) ) {
            if ( LL_I2C_IsActiveFlag_TXIS( _periph ) ) {
                LL_I2C_TransmitData8( _periph, *(data++) );
            }
        }

        int error = checkError();

        LL_I2C_ClearFlag_STOP( _periph );
        
        return error;
    }

    int _read( const uint32_t peripheralAddress, uint8_t* buffer, const uint32_t transferSize ) {
        LL_I2C_HandleTransfer( _periph, peripheralAddress, LL_I2C_ADDRSLAVE_7BIT, transferSize, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ );

        uint8_t i = 0;
        while( !LL_I2C_IsActiveFlag_STOP( _periph ) ) {
            if ( LL_I2C_IsActiveFlag_RXNE( _periph ) ) {
                buffer[i] = LL_I2C_ReceiveData8( _periph );
                ++i;
            }
        }

        int error = checkError();

        LL_I2C_ClearFlag_STOP( _periph );

        return error;
    }
};

