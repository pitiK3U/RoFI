#include "bsp.hpp"

#include <atoms/result.hpp>
// #include <atoms/util.hpp>

#include <cassert>

#include <system/idle.hpp>
#include <system/dbg.hpp>

#include <drivers/clock.hpp>
#include <drivers/gpio.hpp>
#include <drivers/timer.hpp>
#include <drivers/i2c.hpp>

#include <lidar.hpp>

using Block = memory::Pool::Block;

int main() {
    // constexpr auto dbgInstance = bsp::dbgInstance;
    // bsp::setupSystemClock();
    // SystemCoreClockUpdate();
    // HAL_Init();
    
    bsp::setupBoard();

    Dbg::info( "Main clock: %d", SystemCoreClock );

    // TODO: Lidar might not need to own `i2c` since it might be shared by 
    // other devices. Only requirements are that some i2c is configure and there
    // is given `write` and `read` function.
    Lidar lidar;

    Dbg::info("VL53L1 init start\n");

    auto result = lidar.initialize( &*bsp::i2c ).and_then( [&] ( auto ) {
        Dbg::info("start measuring\n");
        return lidar.startMeasurement();
    });

    if ( !result ) {
        Dbg::error( "\n" );
        Dbg::error( result.assume_error().data() );
        return 1;
    }

    while ( true ) {
        // uint16_t sensor_id;
        // VL53L1X_ULP_GetSensorId(0x52, &sensor_id);

        result = lidar.getMeasurementDataReady().and_then( [&] ( bool ready ) {
            return !ready
            ? atoms::make_result_value< atoms::Void >()
            : lidar.getRangingMeasurementData()
                .and_then( [&] ( auto rangingMeasurementData ) {
                    Dbg::info( "Range status: %d, Range: %d mm\n",
                        rangingMeasurementData.RangeStatus,
                        rangingMeasurementData.RangeMilliMeter );

                    return lidar.clearInterruptAndStartMeasurement();
                });
        });

        if ( !result ) {
            Dbg::error( "\n" );
            Dbg::error( result.assume_error().data() );
            return 2;
        }

        if ( Dbg::available() ) {
            switch( Dbg::get() ) {
            default:
                Dbg::error( "DBG received: %c", Dbg::get() );
            }
        }
        if ( IdleTask::run() ) {
            // Dbg::error("D\n");
        }
    }

    Dbg::info( "End measure()" );

    lidar.stopMeasurement();
}

