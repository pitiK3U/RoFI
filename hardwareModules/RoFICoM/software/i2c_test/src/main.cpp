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

#include <vl53l1_api.h>
#include <vl53l1_platform.h>
#include <vl53l1_platform_init.h>


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

VL53L1_Error Measure() {
    char buf[VL53L1_MAX_STRING_LENGTH];
    VL53L1_Error status;
    VL53L1_Dev_t device;

    Dbg::info("VL53L1 init start\n");    

    status = VL53L1_platform_init( &device, 0x52, VL53L1_I2C, 400 );
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Platform init error: %s\n", buf);
        return status;
    }

    uint16_t uid = 0;
    status = VL53L1_RdWord( &device, 0x010F, &uid );
    if ( status != VL53L1_ERROR_NONE ) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Get UID (0x%X) error %d: %s\n", uid, status, buf);
        return status;
    } else if ( uid != 0xEACC ) {
        Dbg::info("Intial register is not 0xEACC but: 0x%X\n", uid);
    }

    status = VL53L1_WaitDeviceBooted(&device);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Wait device booted error %d: %s\n", status, buf);
        return status;
    }

    status = VL53L1_DataInit(&device);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Device data init error %d: %s\n", status, buf);
        return status;
    }

    status = VL53L1_StaticInit(&device);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Device static init error %d: %s\n", status, buf);
        return status;
    }
    Dbg::info("Device successfully initiliased\n");

    /* LIDAR CALIBRATION

    status = VL53L1_PerformRefSpadManagement( &device );
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Ref Spad Management error %d: %s\n", status, buf);
        return status;
    }

    status = VL53L1_PerformOffsetSimpleCalibration(&device, 100);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Offset calibration error %d: %s\n", status, buf);
        return status;
    }

    status = VL53L1_SetPresetMode(&device, VL53L1_PRESETMODE_AUTONOMOUS);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Set preset mode error %d: %s\n", status, buf);
        return status;
    }

    status = VL53L1_PerformSingleTargetXTalkCalibration(&device, 100);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Xtalk calibration error %d: %s\n", status, buf);
        return status;
    }

    VL53L1_CalibrationData_t calibrationData;
    status = VL53L1_GetCalibrationData(&device, &calibrationData);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Get calibration data error %d: %s\n", status, buf);
        return status;
    }

    Dbg::error("Calibration data collected: \n"); */


    uint32_t timing_budget = 0;
    status = VL53L1_GetMeasurementTimingBudgetMicroSeconds( &device, &timing_budget );
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Get measurement timing budget error %d: %s\n", status, buf);
        return status;
    }
    Dbg::info( "Measurement timing budget: %d us", timing_budget );

    status = VL53L1_GetInterMeasurementPeriodMilliSeconds( &device, &timing_budget );
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Get intermeasurement period error %d: %s\n", status, buf);
        return status;
    }
    Dbg::info( "Intermeasument period: %d ms", timing_budget );


    /* status = VL53L1_wait_for_test_completion( &device );
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Wait for test completion error %d: %s\n", status, buf);
        return status;
    } */


    Dbg::info("start measuring\n");

    status = VL53L1_StartMeasurement(&device);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Start measurement error %d: %s\n", status, buf);
        return status;
    }

    while ( true ) {
        status = VL53L1_WaitMeasurementDataReady( &device );
        if (status != VL53L1_ERROR_NONE) {
            VL53L1_GetPalErrorString(status, buf);
            Dbg::error("Wait measurement data ready error %d: %s\n", status, buf);

            /*
            VL53L1_system_control_t data;
            status =  VL53L1_get_system_control( &device, &data );
            if (status != VL53L1_ERROR_NONE) {
                VL53L1_GetPalErrorString(status, buf);
                Dbg::error("Get system control error %d: %s\n", status, buf);
                return status;
            }
            Dbg::info( "Sys int clear: 0x%x\nSys mode start: 0x%x", data.system__interrupt_clear, data.system__mode_start );


            VL53L1_system_results_t results;
            VL53L1_get_system_results( &device, &results );
            if (status != VL53L1_ERROR_NONE) {
                VL53L1_GetPalErrorString(status, buf);
                Dbg::error("Get system results error %d: %s\n", status, buf);
                return status;
            }
            Dbg::info( "Result range status: 0x%x", results.result__range_status ); */

            return status;
        }

        VL53L1_RangingMeasurementData_t rangingMeasurementData;
        status = VL53L1_GetRangingMeasurementData(&device, &rangingMeasurementData);
        if (status != VL53L1_ERROR_NONE) {
            VL53L1_GetPalErrorString(status, buf);
            Dbg::error("Get ranging measurement data error %d: %s\n", status, buf);
            return status;
        }
    
        Dbg::error("Range status: %d ,Range: %d mm\n",
            rangingMeasurementData.RangeStatus,
            rangingMeasurementData.RangeMilliMeter);

        status = VL53L1_ClearInterruptAndStartMeasurement(&device);
        if (status != VL53L1_ERROR_NONE) {
            VL53L1_GetPalErrorString(status, buf);
            Dbg::error("Clear interrupt error %d: %s\n", status, buf);
            return status;
        }

    }

    status = VL53L1_StopMeasurement(&device);
    if (status != VL53L1_ERROR_NONE) {
        VL53L1_GetPalErrorString(status, buf);
        Dbg::error("Stop Measurement error %d: %s\n", status, buf);
        return status;
    }

    Dbg::info( "End measure()" );

    return VL53L1_ERROR_NONE;
}

int main() {
    setupSystemClock();
    SystemCoreClockUpdate();
    HAL_Init();

    Dbg::info( "Main clock: %d", SystemCoreClock );

    setupI2C();

    // LL_mDelay(200);

    Measure();

    while ( true ) {
        // uint16_t sensor_id;
        // VL53L1X_ULP_GetSensorId(0x52, &sensor_id);

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

