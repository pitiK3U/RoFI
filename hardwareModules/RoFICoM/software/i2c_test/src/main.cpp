
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
#include "VL53L1X_api.h"

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

uint8_t ULPMeasure() {
    /*********************************/
	/*   VL53L1X ranging variables  */
	/*********************************/

	uint8_t 				status;
	uint8_t 				dev;
	uint16_t 				sensor_id;


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Default VL53L1X Ultra Low Power I2C address */
	dev = 0x52;

    Dbg::info("Debug main start\n");


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L1X sensor connected */
	status = VL53L1X_ULP_GetSensorId(dev, &sensor_id);
	if(status || (sensor_id != 0xEACC))
	{
		Dbg::error("VL53L1X not detected at requested address status: %d\n", status);
		return status;
	}

	/* (Mandatory) Init VL53L1X sensor */
    // do {
        status = VL53L1X_ULP_SensorInit(dev);
        if (status)
        {
            Dbg::error("VL53L1X ultra low power Loading failed status: %d\n", status);
            return status;
        }
    // } while (status != VL53L1X_ULP_ERROR_NONE);


	Dbg::info("VL53L1X ultra low power ready !\n");

    /*********************************/
	/*         Ranging loop          */
	/*********************************/

	// status = VL53L1X_ULP_StartRanging(dev);
	// if(status)
	// {
	// 	Dbg::error("VL53L1X_ULP_StartRanging failed with status %u\n", status);
    //     return status;
	// }

	// Dbg::info("Ranging started. Put your hand close to the sensor to generate an interrupt...\n");


    // uint8_t ready = 0;
    // do
    // {
    //     VL53L1X_ULP_CheckForDataReady(dev, &ready);
    //     LL_mDelay(100);
    // } while (ready != 1);
    
    // uint8_t measurement_status;
    // uint16_t estimated_distance_mm, sigma_mm, signal_kcps, ambient_kcps;

    // /* Dump debug data */
    // status = VL53L1X_ULP_DumpDebugData(dev, &measurement_status,
    //                                    &estimated_distance_mm, &sigma_mm, &signal_kcps, &ambient_kcps);

    // /* Print debug data. Measurement status 0 means that a valid target
    //  * has been detected */
    // Dbg::info("DEBUG DATA : Status = %2u, Estimated distance = %4u mm, Signal = %6u kcps, Sigma = %3u mm\n",
    //           measurement_status,
    //           estimated_distance_mm,
    //           signal_kcps,
    //           sigma_mm);

    // status = VL53L1X_ULP_StopRanging(dev);

	// Dbg::info("End of VL53L1X ultra low power demo\n");

    return status;

}

uint8_t ULMeasure() {
    uint8_t dev = 0x52;
    uint8_t status;

    uint8_t state;
    status = VL53L1X_BootState(dev, &state);
    if (status) {
        Dbg::error("VL53L1X error booting: %d\n", status);
        return status;
    }
    Dbg::info("VL53L1X boot state: %d\n", state);

    uint16_t sensor_id;
    status = VL53L1X_GetSensorId(dev, &sensor_id);
	if(status || (sensor_id != 0xEACC))
	{
		Dbg::error("VL53L1X not detected at requested address status: %d\n", status);
		return status;
	}

    status = VL53L1X_SensorInit(dev);
    if (status)
    {
        Dbg::error("VL53L1X ultra lite power Loading failed status: %d\n", status);
        return status;
    }

    return status;
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

    ULMeasure();

    while ( true ) {
        // uint16_t sensor_id;
        // VL53L1X_ULP_GetSensorId(0x52, &sensor_id);
        // Dbg::info("received: %d", sensor_id);

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

