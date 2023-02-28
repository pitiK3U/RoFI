#include <cassert>
#include <optional>

#include <system/dbg.hpp>

#include <drivers/gpio.hpp>
#include <drivers/i2c.hpp>
#include <drivers/timer.hpp>

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
    extern std::optional< I2C > i2c;
    // us timer needed for lidar waiting
    extern std::optional< Timer > microTimer;

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
