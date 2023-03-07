#include "bsp.hpp"

#include "configuration.hpp"
#include "motor.hpp"

#include <optional>

#include <stm32g0xx_hal.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_system.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_utils.h>
#include <stm32g0xx_ll_i2c.h>

namespace {
    // TODO <stm32cxx/drivers/clock.hpp>
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

        LL_Init1msTick( 64000000 );

        LL_SYSTICK_SetClkSource( LL_SYSTICK_CLKSOURCE_HCLK );
        /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
        LL_SetSystemCoreClock( 64000000 );
        LL_RCC_SetUSARTClockSource( LL_RCC_USART1_CLKSOURCE_PCLK1 );
        LL_RCC_SetUSARTClockSource( LL_RCC_USART2_CLKSOURCE_PCLK1 );
    }
}

namespace bsp {
    const Gpio::Pin connectorSenseA = { 6, GPIOC };
    const Gpio::Pin connectorSenseB = { 15, GPIOA };
    const Gpio::Pin sliderRetrationLimit = { 4, GPIOB };
    const Gpio::Pin sliderExpansionLimit = { 8, GPIOB };
    const Gpio::Pin sliderMotorPin = { 14, GPIOC };
    const Gpio::Pin spiCSPin = { 4, GPIOA };


    std::optional< Timer > timer;
    std::optional< Timer::Pwm > pwm;
    std::optional< Spi > spi;
    std::optional< Uart > uart;

    std::optional< I2C > i2c;
    std::optional< Timer > microTimer;

    void setupBoard() {
        setupSystemClock();
        SystemCoreClockUpdate();

        timer = Timer( TIM1, FreqAndRes( 1000, 2000 ) );

        pwm = timer->pwmChannel( LL_TIM_CHANNEL_CH1 );
        pwm->attachPin( GpioA[ 8 ] );
        timer->enable();

        Motor motor( bsp::pwm.value(), GpioB[ 1 ] );
        motor.enable();
        motor.set( 0 );
        
        spi = Spi( SPI1,
            Slave(),
            MisoOn( GpioA[ 6 ] ),
            SckOn( GpioB[ 3 ] ),
            CsOn( spiCSPin )
        );

        uart = Uart( USART2,
            Baudrate( cfg::TRANSMIT_BAUDRATE ),
            TxOn( GpioA[ 2 ] ),
            RxOn( GpioA[ 3 ] ),
            UartOversampling( 8 ) );
        uart->enable();

        i2c = I2C( I2C2, SdaPin( GpioA[12] ), SclPin( GpioA[11] ) );
        microTimer = Timer( TIM2, FreqAndRes( cfg::MICROSECOND_FREQUENCY, UINT16_MAX ) );
    }


} // namespace bsp

Dbg& dbgInstance() {
    static Dbg inst(
        USART1, Dma::allocate( DMA1, 1 ), Dma::allocate( DMA1, 2 ),
        TxOn( GpioB[ 6 ] ),
        RxOn( GpioB[ 7 ] ),
        Baudrate( cfg::DBG_BAUDRATE ) );
    return inst;
}
