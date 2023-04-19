#undef NDEBUG
#include <cassert>
#include <functional>
#include <type_traits>

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

using LidarResult = decltype( std::declval< Lidar >().getRangingMeasurementData() );

enum ConnectorStateFlags {
    PositionExpanded  = 1 << 0,
    InternalConnected = 1 << 1,
    ExternalConnected = 1 << 2,
    /**
     * Usually is affected by ambient light.
     * Ob00 = autonomous distance measurement
     * 0b01 = short distance measurement <= 1.3m
     * 0b10 = medium distance measurement 
     * 0b11 = long distance measurement <= 4m
    */
    LidarDistanceMode = 0b11 << 3,
    MatingSide        = 1 << 8,
    Orientation       = 0b11 << 9,
    /**
     * 0b00 = valid measurement data
     * 0b01 = Data outside measured range (data DOESN'T HAVE TO BE valid, but can be, usually means is below or above of range we can measure )
     * 0b10 = Data not yet measured
     * 0b11 = error
    */
    LidarStatus       = 0b11 << 11,
};


void onCmdVersion( SpiInterface& interf ) {
    auto block = memory::Pool::allocate( 4 );
    viewAs< uint16_t >( block.get() ) = 1;
    viewAs< uint16_t >( block.get() + 2 ) = 1;
    interf.sendBlock( std::move( block ), 4 );
}

void onCmdStatus( SpiInterface& interf, Block header,
    ConnComInterface& connInt, Slider& slider, std::optional< LidarResult >& lidarResult )
{
    uint16_t status = viewAs< uint16_t >( header.get() );
    uint16_t mask = viewAs< uint16_t >( header.get() + 2 );
    if ( mask & ConnectorStateFlags::PositionExpanded ) {
        if ( status & ConnectorStateFlags::PositionExpanded )
            slider.expand();
        else
            slider.retract();
    }

    uint8_t lidarStatus;
    if ( ! lidarResult.has_value() ) {
        lidarStatus = 0b10;
    } else if ( ! ( lidarResult.value().has_value() ) ) {
        lidarStatus = 0b11;
    } else if ( auto& lidarData = lidarResult.value().assume_value();
                lidarData.Status == 0 ) {
        lidarStatus = 0b00;
    } else {
        lidarStatus = 0b01;
    }

    auto block = memory::Pool::allocate( 14 );
    memset( block.get(), 0xAA, 14 );
    viewAs< uint8_t >( block.get() + 1 ) = lidarStatus << 3;
    viewAs< uint8_t >( block.get() + 2 ) = connInt.pending();
    viewAs< uint8_t >( block.get() + 3 ) = connInt.available();
    viewAs< uint8_t >( block.get() + 4 ) = 42;
    if ( !(lidarStatus & 0b10) )
        viewAs< uint16_t >( block.get() + 12 ) = lidarResult.value().assume_value().Distance;
    // ToDo: Assign remaining values
    interf.sendBlock( std::move( block ), 14 );
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


void lidarGet( Lidar& lidar, std::optional< LidarResult >& );

void lidarInit( Lidar& lidar, std::optional< LidarResult >& currentLidarMeasurement ) {
    auto result = lidar.initialize().and_then( [&] ( auto ) {
            Dbg::blockingInfo("start measuring\n");
            return lidar.startMeasurement();
    });

    if ( !result ) {
        // *currentLidarMeasurement = result;
        Dbg::error( "\n Error init:" );
        Dbg::error( result.assume_error().data() );
        IdleTask::defer( [&]( ) { lidarInit( lidar, currentLidarMeasurement ); } );
    } else {
        IdleTask::defer( [&]( ) { lidarGet( lidar, currentLidarMeasurement ); } );
    }
}

void lidarGet( Lidar& lidar, std::optional< LidarResult >& currentLidarMeasurement ) {
    using Data = LidarResult::value_type;

    auto result = lidar.getMeasurementDataReady().and_then( [&] ( bool ready ) -> atoms::Result< std::optional< Data >, std::string_view >  {
        if (!ready) {
            return atoms::Result< std::optional< Data >, std::string_view >::value( std::nullopt );
        }

        return lidar.getRangingMeasurementData()
            .and_then( [&] ( auto rangingMeasurementData ) {
                return lidar.clearInterruptAndStartMeasurement().and_then( [&] ( auto ) {
                    return atoms::Result< std::optional< Data >, std::string_view >::value( std::make_optional( rangingMeasurementData ) );
                } );
            });
    });

    
    if ( !result ) {
        currentLidarMeasurement = LidarResult::error( result.assume_error() );
        Dbg::error( "\nError get: " );
        Dbg::error( result.assume_error().data() );
        IdleTask::defer( [&]( ) { lidarInit( lidar, currentLidarMeasurement ); } );
    } else {
        std::optional< Data > data = result.assume_value();
        if ( !data ) {
            // currentLidarMeasurement = std::nullopt;
        } else {
            currentLidarMeasurement = LidarResult::value( data.value() );
        }
        IdleTask::defer( [&]( ) { lidarGet( lidar, currentLidarMeasurement ); } );
    }

}


int main() {
    bsp::setupBoard();
    HAL_Init();

    Dbg::blockingInfo( "Starting" );

    Adc1.setup();
    Adc1.enable();

    Slider slider( std::move( bsp::motor ).value(), bsp::posPins );
    PowerSwitch powerInterface;
    ConnectorStatus connectorStatus ( bsp::connectorSenseA, bsp::connectorSenseB );
 
    // TODO: use IdleTask::defer for hotplug lidar
    // TODO: solve ownership of peripherals when initializing lidar
    // constexpr auto wait = [] ( uint32_t wait ) { bsp::microTimer->blockingWait( wait ); };

    Lidar lidar( &*bsp::i2c, bsp::lidarEnablePin );

    std::optional< LidarResult > currentLidarMeasurement;
 
    // while (true) {
    //     auto id = lidar.getSensorId();
    //     if ( id.has_value() ) {
    //         Dbg::blockingInfo("ID: %u", id.assume_value() );
    //     } else {
    //         Dbg::error("Err: %d", id.assume_error() );
    //     }
    // }
    // assert( false );

    // auto init = lidar.initialize();
    // assert( init );

    // lidarInit( lidar, currentLidarMeasurement );

    ConnComInterface connComInterface( std::move( bsp::uart ).value() );

    using Command = SpiInterface::Command;
    SpiInterface spiInterface( std::move( bsp::spi ).value(), GpioA[ 4 ],
        [&]( Command cmd, Block b ) {
            switch( cmd ) {
            case Command::VERSION:
                onCmdVersion( spiInterface );
                break;
            case Command::STATUS:
                onCmdStatus( spiInterface, std::move( b ), connComInterface, slider, currentLidarMeasurement );
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
    connComInterface.onNewBlob( [&] { spiInterface.interruptMaster(); } );

    Dbg::blockingInfo( "Ready for operation" );

    while ( true ) {
        Dbg::blockingInfo( "state: %d", slider._currentState );
        auto readCount = 0;
        auto readPosSum = 0;
        for ( auto i = 0; auto posPin : bsp::posPins ) {
            if ( ! posPin.read() ) {
                ++readCount;
                readPosSum += i;
            }
            ++i;
        }
        Dbg::blockingInfo( "position: %d", 100 * readPosSum / ( readCount != 0 ? readCount : 1 )  / ( bsp::posPins.size() - 1 ) );
        // Dbg::blockingInfo( "Raw: %d,\t\tVoltage: %f", read, ( (float)read * 6.8f * 3.3f ) / ( 4096 * ( 100.0f + 6.8f ) ) );

        slider.run();

        powerInterface.run();
        if ( connectorStatus.run() )
            spiInterface.interruptMaster();

        if ( Dbg::available() ) {
            char chr = Dbg::get();
            switch( chr ) {
            case 'e':
                slider.expand();
                Dbg::blockingInfo("Expanding");
                break;
            case 'r':
                slider.retract();
                Dbg::blockingInfo("Retracting");
                break;
            case 's':
                slider.stop();
                Dbg::blockingInfo("Stopping");
                break;
            default:
                Dbg::error( "DBG received: %c, %d", chr, int( chr ) );
            }
        }
        // Dbg::error("%d, %d", GpioB[ 4 ].read(), GpioB[ 8 ].read());
        // connComInterface.run();

        IdleTask::run();
        // REMOVE:
        // if ( currentLidarMeasurement.has_value() ) {
        //     if ( currentLidarMeasurement.value().has_value() ){
        //         const auto rangingMeasurementData = currentLidarMeasurement.value().assume_value();
        //         Dbg::blockingInfo( "Range status: %hhu, Range: %hu mm, Ambient: %hu\n",
        //             rangingMeasurementData.Status,
        //             rangingMeasurementData.Distance );
        //     } else {
        //         Dbg::error( "Error while measurement: %s\n", currentLidarMeasurement.value().assume_error() );
        //     }
        // } else {
        //     Dbg::blockingInfo( "Data not yet measured\n" );
        // }


    }
}

