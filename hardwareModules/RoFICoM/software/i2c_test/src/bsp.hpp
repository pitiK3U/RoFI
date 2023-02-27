#include <cassert>
#include <optional>

#include <system/dbg.hpp>

#include <drivers/clock.hpp>
#include <drivers/gpio.hpp>
#include <drivers/i2c.hpp>
#include <drivers/timer.hpp>

#pragma once

Dbg &dbgInstance();

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
    // TODO: microsecond timer

    extern std::optional< I2C > i2c;

    void setupBoard();
    void setupSystemClock();
}
