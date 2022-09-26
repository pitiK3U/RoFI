
#include <system/defer.hpp>

#include <cassert>
#include <system/dbg.hpp>
#include <drivers/clock.hpp>
#include <drivers/gpio.hpp>
#include <drivers/timer.hpp>

#include <stm32g0xx_hal.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_system.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_utils.h>

#include <stm32g0xx_ll_i2c.h>

#include "VL53L1X_ULP_api.h"

/* Disable handling uncaught exceptions to save flash space */
/*
namespace __gnu_cxx {
    void __verbose_terminate_handler() {
        for(;;);
    }
}
*/

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

void setupI2C() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = { };

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /**I2C2 GPIO Configuration
    PA11 [PA9]   ------> I2C2_SCL
    PA12 [PA10]   ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Peripheral clock enable */
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

    LL_I2C_Init(I2C2, &I2C_InitStruct);
    LL_I2C_EnableAutoEndMode(I2C2);
    LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);
    LL_I2C_DisableOwnAddress2(I2C2);
    LL_I2C_DisableGeneralCall(I2C2);
    LL_I2C_EnableClockStretching(I2C2);
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

    Dbg::error( "Main clock: %d", SystemCoreClock );

    Timer timer( TIM1, FreqAndRes( 1000, 2000 ) );
    auto pwm = timer.pwmChannel( LL_TIM_CHANNEL_CH1 );
    pwm.attachPin( GpioA[ 8 ] );
    timer.enable();

    setupI2C();
    
    /*
    const uint32_t slaveAddress = 0x52;
    // Index of Model ID. The result should be 0xEA 
    const uint16_t index = 0x010F;
    const uint8_t transmitBuffer[2] = { index >> 8, index & 0xFF };
    
    
    LL_I2C_SetSlaveAddr( I2C2, slaveAddress );

    LL_I2C_SetTransferRequest( I2C2, LL_I2C_REQUEST_WRITE );
    LL_I2C_SetTransferSize( I2C2, 2 );

    LL_I2C_GenerateStartCondition( I2C2 );

    LL_I2C_TransmitData8( I2C2, index >> 8 );
    LL_I2C_TransmitData8( I2C2, index & 8 );

    LL_I2C_GenerateStopCondition( I2C2 );
    

    Dbg::error( "Ready for operation" );

    LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE );

    uint8_t i = 0;

    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_TXIS( I2C2 ) ) {
            LL_I2C_TransmitData8( I2C2, transmitBuffer[i] );
        Dbg::info( "sent %d. byte", i );
        ++i;
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );

    LL_I2C_HandleTransfer( I2C2, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ );

    uint8_t data = 0;
    while( !LL_I2C_IsActiveFlag_STOP( I2C2 ) ) {
        if ( LL_I2C_IsActiveFlag_RXNE( I2C2 ) ) {
            data = LL_I2C_ReceiveData8( I2C2 );
            Dbg::info( "I2C received: %d", data );
        }
    }

    LL_I2C_ClearFlag_STOP( I2C2 );
    */

    /*********************************/
	/*   VL53L1X ranging variables  */
	/*********************************/

	uint8_t 				status, loop;
	uint8_t 				dev;
	uint16_t 				sensor_id;


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Default VL53L1X Ultra Low Power I2C address */
	dev = 0x52;

	/* (Optional) Change I2C address */
	// status = VL53L1X_ULP_SetI2CAddress(dev, 0x20);
	// dev = 0x20;


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L1X sensor connected */
	status = VL53L1X_ULP_GetSensorId(dev, &sensor_id);
	if(status || (sensor_id != 0xEACC))
	{
		Dbg::error("VL53L1X not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L1X sensor */
	status = VL53L1X_ULP_SensorInit(dev);
	if(status)
	{
		Dbg::error("VL53L1X ultra low power Loading failed\n");
		return status;
	}

	Dbg::info("VL53L1X ultra low power ready !\n");

    while ( true ) {
        if ( Dbg::available() ) {
            switch( Dbg::get() ) {
            default:
                Dbg::error( "DBG received: %c", Dbg::get() );
            }
        }
        if (Defer::run()) {
            // Dbg::error("D\n");
        }
    }
}

