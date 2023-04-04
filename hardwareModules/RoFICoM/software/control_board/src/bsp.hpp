#include <cassert>
#include <optional>

#include <system/dbg.hpp>


#include <drivers/gpio.hpp>
#include <drivers/i2c.hpp>
#include <drivers/spi.hpp>
#include <drivers/timer.hpp>
#include <drivers/uart.hpp>

#pragma once

Dbg &dbgInstance();

extern void SystemCoreClockUpdate(void);

/**
 * This is Board Support Package integration into RoFI.
 * 
 * It's main purpose is to specify objects, functions, ... 
 * that are needed for given firmware in a way which enables 
 * better and easier portability for different hardware boards.
 * 
 * 
*/
namespace bsp {
    extern const Gpio::Pin connectorSenseA;
    extern const Gpio::Pin connectorSenseB;
    extern const Gpio::Pin internalSwitchPin;
    extern const Gpio::Pin internalVoltagePin;
    extern const Gpio::Pin internalCurrentPin;
    extern const Gpio::Pin externalSwitchPin;
    extern const Gpio::Pin externalVoltagePin;
    extern const Gpio::Pin externalCurrentPin;


    extern const Gpio::Pin spiCSPin;
    extern const Gpio::Pin lidarEnablePin;
    extern const Gpio::Pin lidarIRQPin;
    extern const std::array< Gpio::Pin, 9 > posPins;


    extern std::optional< Timer > timer;
    extern std::optional< Timer::Pwm > pwm;
    extern std::optional< Spi > spi;
    extern std::optional< Uart > uart;

    extern std::optional< I2C > i2c;
    // microsecond timer needed for lidar waiting
    // extern std::optional< Timer > microTimer;

    /**
     * Function to setup your board, MUST be called before using 
     * any members of `bsp` namespace. Setups board specific
     * clock, timers, pins, peripherals, etc.
     * 
     * Should be implemented in `bsp.cpp` with board specific 
     * initialization.
    */
    void setupBoard();
}
