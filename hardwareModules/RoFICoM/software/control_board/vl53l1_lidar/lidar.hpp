#pragma once

#include <cstdint>
#include <string_view>

#include <atoms/result.hpp>

#include <drivers/timer.hpp>
#include <drivers/i2c.hpp>

#include <vl53l1_api.h>
#include <vl53l1_platform.h>
#include <vl53l1_platform_init.h>

namespace _inner {
    void initialize_platform( I2C* i2cPeriph, Timer microTimer );
}

struct Lidar
{
    using Void = atoms::Void;

    template< typename T, typename E = std::string >
    using Result = atoms::Result< T, E >;

    using result_type = Result< Void, std::string_view >;

    Lidar( const uint32_t deviceAddress = 0x52, const uint32_t commSpeed = 400 )
        : _deviceAddress( deviceAddress )
        , _communicationSpeed( commSpeed )
    {
        // As mentioned in datasheet VL53L1X has maximum speed of 400 kbits/s
        assert( commSpeed <= 400 );
    }

    ~Lidar()
    {
        VL53L1_platform_terminate( &_device );
    }

    // TODO: make sure this function is called
    // TOOD: might not need to take controll over the timer only need function to call to wait 1us
    result_type initialize( I2C* i2cPeriph, Timer microTimer )
    {
        _inner::initialize_platform( i2cPeriph, std::move( microTimer ) );

        VL53L1_Error status;

        status = VL53L1_platform_init( &_device, _defaultAddress, VL53L1_I2C, _communicationSpeed );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        if ( _deviceAddress != _defaultAddress ) {
            status = VL53L1_SetDeviceAddress( &_device, _deviceAddress );
            if ( status != VL53L1_ERROR_NONE ) {
                return result_type::error( errorToString( status ) );
            }
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

    Result< bool, std::string_view > getMeasurementDataReady()
    {
        uint8_t ready;
        VL53L1_Error status = VL53L1_GetMeasurementDataReady( &_device, &ready );
        if ( status != VL53L1_ERROR_NONE ) {
            return Result< bool, std::string_view >::error( errorToString( status ) );
        }

        return atoms::make_result_value< bool >( bool(ready) );
    }

    result_type clearInterruptAndStartMeasurement()
    {
        VL53L1_Error status = VL53L1_ClearInterruptAndStartMeasurement( &_device );
        if ( status != VL53L1_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    Result< VL53L1_RangingMeasurementData_t, std::string_view > getRangingMeasurementData()
    {
        VL53L1_RangingMeasurementData_t rangingMeasurementData;

        VL53L1_Error status = VL53L1_GetRangingMeasurementData( &_device, &rangingMeasurementData );
        if ( status != VL53L1_ERROR_NONE ) {
            return atoms::result_error( errorToString( status ) );
        }

        return atoms::result_value< VL53L1_RangingMeasurementData_t >( rangingMeasurementData );
    }

private:
    std::string_view errorToString( VL53L1_Error err );

    // I2C _i2c;
    VL53L1_Dev_t _device;

    uint32_t _deviceAddress;
    uint32_t _communicationSpeed;

    const uint32_t _defaultAddress = 0x52;
};

