#include <cassert>

#include <system/idle.hpp>
#include <system/dbg.hpp>

#include <drivers/clock.hpp>
#include <drivers/gpio.hpp>
#include <drivers/timer.hpp>
#include <drivers/spi.hpp>
#include <drivers/uart.hpp>
#include <drivers/adc.hpp>

#include <motor.hpp>
#include <lidar.hpp>
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

int main() {
    bsp::setupBoard();

    Dbg::blockingInfo( "HELP" );

    Dbg::blockingInfo( "Starting" );


    Lidar lidar;
    auto result = lidar.initialize( &*bsp::i2c, *std::move( bsp::microTimer ) ).and_then( [&] ( auto ) {
        Dbg::blockingInfo("start measuring\n");
        return lidar.startMeasurement();
    });

    if ( !result ) {
        Dbg::error( "\n" );
        Dbg::error( result.assume_error().data() );
        return 1;
    }

    Adc1.setup();
    Adc1.enable();

    Timer timer( TIM1, FreqAndRes( 1000, 2000 ) );
    auto pwm = timer.pwmChannel( LL_TIM_CHANNEL_CH1 );
    pwm.attachPin( GpioA[ 8 ] );
    timer.enable();

    Motor motor( pwm, GpioB[ 1 ] );
    motor.enable();
    motor.set( 0 );

    Slider slider( Motor( pwm, GpioC[ 14 ] ), GpioB[ 4 ], GpioB[ 8 ] );

    PowerSwitch powerInterface;
    ConnectorStatus connectorStatus ( GpioC[ 6 ], GpioA[ 15 ] );


    Spi spi( SPI1,
        Slave(),
        MisoOn( GpioA[ 6 ] ),
        SckOn( GpioB[ 3 ] ),
        CsOn( GpioA[ 4 ] )
    );

    Uart uart( USART2,
        Baudrate( cfg::TRANSMIT_BAUDRATE ),
        TxOn( GpioA[ 2 ] ),
        RxOn( GpioA[ 3 ] ),
        UartOversampling( 8 ) );
    uart.enable();

    ConnComInterface connComInterface( std::move( uart ) );

    using Command = SpiInterface::Command;
    SpiInterface spiInterface( std::move( spi ), GpioA[ 4 ],
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
    connComInterface.onNewBlob( [&] { spiInterface.interruptMaster(); } );

    Dbg::blockingInfo( "Ready for operation" );

    while ( true ) {
        slider.run();
        powerInterface.run();
        if ( connectorStatus.run() )
            spiInterface.interruptMaster();

        if ( Dbg::available() ) {
            char chr = Dbg::get();
            switch( chr ) {
            case 'e':
                slider.expand();
                Dbg::info("Expanding");
                break;
            case 'r':
                slider.retract();
                Dbg::info("Retracting");
                break;
            default:
                Dbg::error( "DBG received: %c, %d", chr, int( chr ) );
            }
        }
        // Dbg::error("%d, %d", GpioB[ 4 ].read(), GpioB[ 8 ].read());
        connComInterface.run();

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

        IdleTask::run();
    }
}

