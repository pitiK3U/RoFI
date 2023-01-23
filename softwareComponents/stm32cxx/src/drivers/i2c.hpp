#pragma once

#include <system/assert.hpp>
#include <functional>
#include <array>

#include <drivers/peripheral.hpp>
#include <drivers/gpio.hpp>

#include <stm32g0xx_ll_i2c.h>

struct I2C: public Peripheral< I2C_TypeDef > {
    I2C( I2C_TypeDef *i2c, Gpio::Pin sdaPin, Gpio::Pin sclPin )
        : Peripheral< I2C_TypeDef >( i2c )
    {
        sdaPin.setupODAlternate( true );
        sclPin.setupODAlternate( true );

        // enableClock();
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

        LL_I2C_InitTypeDef I2C_InitStruct = { };

        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
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
    }

    template < typename container >
    void write( const uint32_t peripheralAddress, container& data) {
        LL_I2C_HandleTransfer( _periph, peripheralAddress, LL_I2C_ADDRSLAVE_7BIT, data.size(), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

        typename container::size_type i = 0;

        while( !LL_I2C_IsActiveFlag_STOP( _periph ) ) {
            if ( LL_I2C_IsActiveFlag_TXIS( _periph ) ) {
                LL_I2C_TransmitData8( _periph, data[i] );
                ++i;
            }
        }

        LL_I2C_ClearFlag_STOP( _periph );
    }

    template < typename container >
    container read( const uint32_t peripheralAddress, const uint32_t transferSize ) {
        assert( container::max_size() >= transferSize );
        container buffer;

            LL_I2C_HandleTransfer( _periph, peripheralAddress, LL_I2C_ADDRSLAVE_7BIT, transferSize, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ );

        uint8_t i = 0;
        while( !LL_I2C_IsActiveFlag_STOP( _periph ) ) {
            if ( LL_I2C_IsActiveFlag_RXNE( _periph ) ) {
                buffer[i] = LL_I2C_ReceiveData8( _periph );
                ++i;
            }
        }

        LL_I2C_ClearFlag_STOP( _periph );

        return buffer;
    }

private:
    
};
