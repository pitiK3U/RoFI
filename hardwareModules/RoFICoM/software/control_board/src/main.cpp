#include <cassert>
#include <functional>

#include <system/idle.hpp>
#include <system/dbg.hpp>

#include <drivers/clock.hpp>
#include <drivers/gpio.hpp>
#include <drivers/timer.hpp>
#include <drivers/spi.hpp>
#include <drivers/uart.hpp>
#include <drivers/adc.hpp>

#include <motor.hpp>
#include "lidar.hpp"
#include <util.hpp>

#include <spiInterface.hpp>
#include <connInterface.hpp>

#include <stm32g0xx_hal.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_system.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_utils.h>

#include <bsp.hpp>
#include <configuration.hpp>


using Block = memory::Pool::Block;

enum ConnectorStateFlags {
    PositionExpanded  = 1 << 0,
    InternalConnected = 1 << 1,
    ExternalConnected = 1 << 2,
    MatingSide        = 1 << 8,
    Orientation       = 0b11 << 9,
};


void onCmdVersion( SpiInterface& interf ) {
    auto block = memory::Pool::allocate( 4 );
    viewAs< uint16_t >( block.get() ) = 1;
    viewAs< uint16_t >( block.get() + 2 ) = 1;
    interf.sendBlock( std::move( block ), 4 );
}

void onCmdStatus( SpiInterface& interf, Block header,
    ConnComInterface& connInt, Slider& slider )
{
    uint16_t status = viewAs< uint16_t >( header.get() );
    uint16_t mask = viewAs< uint16_t >( header.get() + 2 );
    if ( mask & ConnectorStateFlags::PositionExpanded ) {
        if ( status & ConnectorStateFlags::PositionExpanded )
            slider.expand();
        else
            slider.retract();
    }

    auto block = memory::Pool::allocate( 12 );
    memset( block.get(), 0xAA, 12 );
    viewAs< uint8_t >( block.get() + 2 ) = connInt.pending();
    viewAs< uint8_t >( block.get() + 3 ) = connInt.available();
    viewAs< uint8_t >( block.get() + 4 ) = 42;
    // ToDo: Assign remaining values
    interf.sendBlock( std::move( block ), 12 );
    // ToDo Interpret the header
}

void onCmdInterrupt( SpiInterface& interf, Block /*header*/ ) {
    auto block = memory::Pool::allocate( 2 );
    viewAs< uint16_t >( block.get() ) = 0;
    interf.sendBlock( std::move( block ), 2 );
}

void onCmdSendBlob( SpiInterface& spiInt, ConnComInterface& connInt ) {
    spiInt.receiveBlob([&spiInt, &connInt]( Block blob, int size ) {
        int blobLen = viewAs< uint16_t >( blob.get() + 2 );
        if ( size != 4 + blobLen )
            return;
        connInt.sendBlob( std::move( blob ) );
    } );
}

void onCmdReceiveBlob( SpiInterface& spiInt, ConnComInterface& connInt ) {
    if ( connInt.available() > 0 ) {
        Block blob = connInt.getBlob();
        spiInt.sendBlob( std::move( blob ) );
    }
    else {
        auto block = memory::Pool::allocate( 4 );
        viewAs< uint16_t >( block.get() ) = 0; // content type
        viewAs< uint16_t >( block.get() + 2 ) = 0; // length
        spiInt.sendBlock( std::move( block ), 4 );
    }
}


void lidarGet( Lidar& lidar );

void lidarInit( Lidar& lidar ) {
    auto result = lidar.initialize().and_then( [&] ( auto ) {
            Dbg::blockingInfo("start measuring\n");
            return lidar.startMeasurement();
        });

        if ( !result ) {
            Dbg::error( "\n Error init:" );
            Dbg::error( result.assume_error().data() );
            IdleTask::defer( std::bind( lidarInit, lidar ) );
        } else {
            IdleTask::defer( std::bind( lidarGet, lidar ) );
        }
}

void lidarGet( Lidar& lidar ) {

    auto result = lidar.getMeasurementDataReady().and_then( [&] ( bool ready ) {
            return !ready
            ? atoms::make_result_value< atoms::Void >()
            : lidar.getRangingMeasurementData()
                .and_then( [&] ( auto rangingMeasurementData ) {
                    Dbg::blockingInfo( "Range status: %d, Range: %d mm\n",
                        rangingMeasurementData.RangeStatus,
                        rangingMeasurementData.RangeMilliMeter );

                    return lidar.clearInterruptAndStartMeasurement();
                });
        });

        if ( !result ) {
            Dbg::error( "\nError get: " );
            Dbg::error( result.assume_error().data() );
            IdleTask::defer( std::bind( lidarInit, lidar ) );
        } else {
            IdleTask::defer( std::bind( lidarGet, lidar ) );
        }

}


int main() {
    bsp::setupBoard();
    HAL_Init();

    Dbg::blockingInfo( "Starting" );

    Adc1.setup();
    Adc1.enable();

/*    Slider slider( Motor( bsp::pwm.value(), bsp::sliderMotorPin ), bsp::sliderRetrationLimit, bsp::sliderExpansionLimit );

    PowerSwitch powerInterface;
    ConnectorStatus connectorStatus ( bsp::connectorSenseA, bsp::connectorSenseB );
 */
    // TODO: use IdleTask::defer for hotplug lidar
    // TODO: solve ownership of peripherals when initializing lidar
    constexpr auto wait = [] ( uint32_t wait ) { bsp::microTimer->blockingWait( wait ); };

    Lidar lidar( &*bsp::i2c, wait );

    using LidarResult = decltype( std::declval<Lidar>().getMeasurementDataReady() );
    std::optional< LidarResult > result;


    lidarInit( lidar );
    // IdleTask::defer( std::bind( lidarInit, lidar ) );

    /* ConnComInterface connComInterface( std::move( bsp::uart ).value() );

    using Command = SpiInterface::Command;
    SpiInterface spiInterface( std::move( bsp::spi ).value(), GpioA[ 4 ],
        [&]( Command cmd, Block b ) {
            switch( cmd ) {
            case Command::VERSION:
                onCmdVersion( spiInterface );
                break;
            case Command::STATUS:
                onCmdStatus( spiInterface, std::move( b ), connComInterface, slider );
                break;
            case Command::INTERRUPT:
                onCmdInterrupt( spiInterface, std::move( b ) );
                break;
            case Command::SEND_BLOB:
                onCmdSendBlob( spiInterface, connComInterface );
                break;
            case Command::RECEIVE_BLOB:
                onCmdReceiveBlob( spiInterface, connComInterface );
                break;
            default:
                Dbg::warning( "Unknown command %d", cmd );
            };
        } );
    connComInterface.onNewBlob( [&] { spiInterface.interruptMaster(); } ); */

    Dbg::blockingInfo( "Ready for operation" );

    while ( true ) {
        /* slider.run();
        powerInterface.run();
        if ( connectorStatus.run() )
            spiInterface.interruptMaster(); */

        if ( Dbg::available() ) {
            char chr = Dbg::get();
            switch( chr ) {
            case 'e':
                // slider.expand();
                Dbg::info("Expanding");
                break;
            case 'r':
                // slider.retract();
                Dbg::info("Retracting");
                break;
            default:
                Dbg::error( "DBG received: %c, %d", chr, int( chr ) );
            }
        }
        // Dbg::error("%d, %d", GpioB[ 4 ].read(), GpioB[ 8 ].read());
        // connComInterface.run();

        IdleTask::run();
    }
}

