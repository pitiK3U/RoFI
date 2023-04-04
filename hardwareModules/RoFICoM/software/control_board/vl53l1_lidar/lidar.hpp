#pragma once

#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <string_view>

#include <atoms/result.hpp>

#include <drivers/timer.hpp>
#include <drivers/i2c.hpp>

#include <VL53L1X_api.h>
#include <vl53l1_platform.h>
// #include <vl53l1_platform_init.h>

// namespace that's used to propagate needed peripherals/functions
// to library's `vl53l1_plaftorm.cpp`
namespace _inner {
    // using waitUsFn = void (*)(uint32_t);

    void initialize_platform( I2C* );
}

struct Lidar
{
    using Void = atoms::Void;

    template< typename T, typename E = std::string >
    using Result = atoms::Result< T, E >;

    using result_type = Result< Void, std::string_view >;
    // using waitUsFn = _inner::waitUsFn;

    const int8_t VL53L1X_ERROR_NONE = 0;

    Lidar( I2C* i2c, const Gpio::Pin lidarEnable, const uint32_t deviceAddress = 0x52, const uint32_t commSpeed = 400 )
        : _i2c( i2c )
        , _lidarEnable( lidarEnable )
        , _deviceAddress( deviceAddress )
        , _communicationSpeed( commSpeed )
    {
        // As mentioned in datasheet VL53L1X has maximum speed of 400 kbits/s
        assert( commSpeed <= 400 );
    }

    ~Lidar()
    {
        // VL53L1_platform_terminate( &_device );
    }

    result_type initialize()
    {
        _lidarEnable.setupPPOutput( );
        _lidarEnable.write( true );
        assert( _lidarEnable.read() );
        _inner::initialize_platform( _i2c );

        VL53L1X_ERROR status;

        if ( _deviceAddress != _defaultAddress ) {
            status = VL53L1X_SetI2CAddress( _defaultAddress, _deviceAddress );
            if ( status != VL53L1X_ERROR_NONE ) {
                return result_type::error( errorToString( status ) );
            }
        }

        uint8_t booted;
        do {
            status = VL53L1X_BootState( _deviceAddress, &booted );
        } while ( !booted );

        status = VL53L1X_SensorInit( _deviceAddress );
        if (status != VL53L1X_ERROR_NONE) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type startMeasurement()
    {
        VL53L1X_ERROR status = VL53L1X_StartRanging( _deviceAddress );
        if (status != VL53L1X_ERROR_NONE) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type stopMeasurement()
    {
        VL53L1X_ERROR status = VL53L1X_ClearInterrupt( _deviceAddress );
        if ( status != VL53L1X_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        status = VL53L1X_StopRanging( _deviceAddress );
        if ( status != VL53L1X_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    result_type waitMeasurementDataReady()
    {
        /*
        VL53L1X_ERROR status = VL53L1_WaitMeasurementDataReady( &_device );
        if ( status != VL53L1X_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }
        */

        return atoms::make_result_value< Void >();
    }

    Result< bool, std::string_view > getMeasurementDataReady()
    {
        uint8_t ready;
        VL53L1X_ERROR status = VL53L1X_CheckForDataReady( _deviceAddress, &ready );
        if ( status != VL53L1X_ERROR_NONE ) {
            return Result< bool, std::string_view >::error( errorToString( status ) );
        }

        return atoms::make_result_value< bool >( bool(ready) );
    }

    result_type clearInterruptAndStartMeasurement()
    {
        VL53L1X_ERROR status = VL53L1X_ClearInterrupt( _deviceAddress );
        if ( status != VL53L1X_ERROR_NONE ) {
            return result_type::error( errorToString( status ) );
        }

        return atoms::make_result_value< Void >();
    }

    Result< VL53L1X_Result_t, std::string_view > getRangingMeasurementData()
    {
        VL53L1X_Result_t rangingMeasurementData;

        VL53L1X_ERROR status = VL53L1X_GetResult( _deviceAddress, &rangingMeasurementData );
        if ( status != VL53L1X_ERROR_NONE ) {
            return atoms::result_error( errorToString( status ) );
        }

        return atoms::result_value< VL53L1X_Result_t >( rangingMeasurementData );
    }

    Result< uint16_t, int8_t > getSensorId()
    {
        _lidarEnable.setupODOutput( true, true );
        _lidarEnable.write( true );
        _inner::initialize_platform( _i2c );

        uint16_t id = 0;
        VL53L1X_ERROR status = VL53L1X_GetSensorId( _deviceAddress, &id );
        if ( status != VL53L1X_ERROR_NONE ) {
            return atoms::result_error( status );
        }

        return atoms::result_value< uint16_t >( id );
    }

private:
    std::string_view errorToString( VL53L1X_ERROR err );

    I2C* _i2c;
    Gpio::Pin _lidarEnable;
    // waitUsFn _waitUs;

    // VL53L1_Dev_t _device;

    uint32_t _deviceAddress;
    uint32_t _communicationSpeed;

    const uint32_t _defaultAddress = 0x52;
};

