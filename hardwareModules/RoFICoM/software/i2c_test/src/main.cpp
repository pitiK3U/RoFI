#include <atoms/result.hpp>
// #include <atoms/util.hpp>

#include <cassert>

#include <system/defer.hpp>
#include <system/dbg.hpp>

#include <drivers/clock.hpp>
#include <drivers/gpio.hpp>
#include <drivers/timer.hpp>
#include <drivers/i2c.hpp>

#include <lidar.hpp>

#include <stm32g0xx_hal.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_system.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_utils.h>

#include <stm32g0xx_ll_i2c.h>


void setupSystemClock() {
    LL_FLASH_SetLatency( LL_FLASH_LATENCY_2 );
    LL_RCC_HSI_Enable();
    while( LL_RCC_HSI_IsReady() != 1 );

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2 );
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while( LL_RCC_PLL_IsReady() != 1 );

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );
    while( LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL );

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_1 );

    LL_Init1msTick(64000000);

    LL_SYSTICK_SetClkSource( LL_SYSTICK_CLKSOURCE_HCLK );
    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock( 64000000 );
    LL_RCC_SetUSARTClockSource( LL_RCC_USART1_CLKSOURCE_PCLK1 );
    LL_RCC_SetUSARTClockSource( LL_RCC_USART2_CLKSOURCE_PCLK1 );
}

using Block = memory::Pool::Block;

Dbg& dbgInstance() {
    static Dbg inst(
        USART1, Dma::allocate( DMA1, 1 ), Dma::allocate( DMA1, 2 ),
        TxOn( GpioB[ 6 ] ),
        RxOn( GpioB[ 7 ] ),
        Baudrate( 115200 ) );
    return inst;
}

int main() {
    setupSystemClock();
    SystemCoreClockUpdate();
    HAL_Init();

    Dbg::info( "Main clock: %d", SystemCoreClock );

    Lidar lidar( I2C( I2C2, SdaPin( GpioA[ 12 ] ), SclPin( GpioA[ 11 ] ) ) );

    Dbg::info("VL53L1 init start\n");

    auto result = lidar.initialize().and_then( [&] ( auto ) {
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
            Dbg::error( result.assume_error().data() );
            return 2;
        }

        if ( Dbg::available() ) {
            switch( Dbg::get() ) {
            default:
                Dbg::error( "DBG received: %c", Dbg::get() );
            }
        }
        if ( Defer::run() ) {
            // Dbg::error("D\n");
        }
    }

    Dbg::info( "End measure()" );

    lidar.stopMeasurement();
}

