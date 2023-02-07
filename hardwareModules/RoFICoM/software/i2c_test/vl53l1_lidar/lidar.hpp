#pragma once

#include <cstdint>
#include <string>
#include <stdio.h>

#include <atoms/result.hpp>

#include <drivers/i2c.hpp>

#include <vl53l1_api.h>
#include <vl53l1_platform.h>
#include <vl53l1_platform_init.h>

struct Lidar
{
    using Void = atoms::Void;

    template< typename T, typename E = std::string >
    using Result = atoms::Result< T, E >;

    using result_type = Result< Void >;

    Lidar( I2C i2c, const uint32_t deviceAddress = 0x52, const uint32_t commSpeed = 400 )
        : _i2c( std::move( i2c ) )
        , _deviceAddress( deviceAddress )
        , _communicationSpeed( commSpeed )
    {

    }

    result_type initialize()
    {
        VL53L1_Error status;

        status = VL53L1_platform_init( &_device, _deviceAddress, VL53L1_I2C, _communicationSpeed );
        if (status != VL53L1_ERROR_NONE) {
            return result_type::error( errorToString( status ) );
        }

        status = VL53L1_WaitDeviceBooted( &_device );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        status = VL53L1_DataInit( &_device );
        if (status != VL53L1_ERROR_NONE) {
            return result_type::error( errorToString( status ) );
        }

        status = VL53L1_StaticInit( &_device );
        if (status != VL53L1_ERROR_NONE) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type startMeasurement()
    {
        VL53L1_Error status = VL53L1_StartMeasurement( &_device );
        if (status != VL53L1_ERROR_NONE) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type stopMeasurement()
    {
        VL53L1_Error status = VL53L1_ClearInterruptAndStartMeasurement( &_device );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        status = VL53L1_StopMeasurement( &_device );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type waitMeasurementDataReady()
    {
        VL53L1_Error status = VL53L1_WaitMeasurementDataReady( &_device );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type clearInterruptAndStartMeasurement()
    {
        VL53L1_Error status = VL53L1_ClearInterruptAndStartMeasurement( &_device );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    Result< VL53L1_RangingMeasurementData_t, std::string> getRangingMeasurementData()
    {
        VL53L1_RangingMeasurementData_t rangingMeasurementData;

        VL53L1_Error status = VL53L1_GetRangingMeasurementData( &_device, &rangingMeasurementData );
        if ( status != VL53L1_ERROR_NONE ) {
            return atoms::result_error( errorToString( status ) );
        }

        return atoms::result_value< VL53L1_RangingMeasurementData_t >( rangingMeasurementData );
    }

private:
    // TODO: find alternative to `std::string`
    std::string errorToString( VL53L1_Error err )
    {
        char buf[VL53L1_MAX_STRING_LENGTH] = {};
        VL53L1_GetPalErrorString( err, buf );
        return std::string( buf, strlen( buf ) );
    }

    I2C _i2c;
    VL53L1_Dev_t _device;

    const uint32_t _deviceAddress;
    const uint32_t _communicationSpeed;
};

