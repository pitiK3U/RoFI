#pragma once

#include <cassert>
#include <array>
#include <stm32g0xx_hal.h>
#include <drivers/timer.hpp>
#include <drivers/gpio.hpp>
// #include <bsp.hpp>

class Motor {
public:
    Motor( Timer::Pwm pwm, Gpio::Pin pin )
        : _pwm( pwm ), _pin( pin )
    {
        set( 0 );
    }

    void enable() {
        _pin.setupPPOutput();
        _pwm.enable();
    }

    void disable() {
        _pin.setupAnalog( false );
        _pwm.disable();
    }

    void set( int val ) {
        if (val < -100 || val > 100 ) {
            Dbg::info("Invalid motor value %d", val);
        }
        if ( val < -100 )
            val = -100;
        if ( val > 100 )
            val = 100;
        if ( val < 0 ) {
            int setpoint = _pwm.top() + val * _pwm.top() / 100;
            _pwm.set( setpoint );
            _pin.write( true );
        }
        else {
            int setpoint = val * _pwm.top() / 100;
            _pwm.set( setpoint );
            _pin.write( false );
        }
    }
private:
    Timer::Pwm _pwm;
    Gpio::Pin _pin;
};

class Slider {
public:
    Slider( Motor motor, const std::array< Gpio::Pin, 10 > & positionPins )
        : _motor( std::move( motor ) ),
          _positionPins( positionPins ),
          _goal( State::Retracted ),
          _currentState( State::Unknown )
    {
        motor.set( 0 );
        motor.enable();

        for ( auto posPin : _positionPins ) {
            posPin.setupInput( false );
        }
    }

    enum class State { Unknown, Retracted, Expanding, Expanded, Retracting };

    void expand() {
        _goal = State::Expanded;
        _onStateChange();
    }

    void retract() {
        _goal = State::Retracted;
        _onStateChange();
    }

    void stop() {
        _goal = State::Unknown;
        _currentState = State::Unknown;
        _onStateChange();
        _move();
    }

    void run() {
        const auto pos = _position();
        if ( _goal == State::Retracted ) {
            if ( pos - _endThreshold <= _retractedPosition )
                _set( State::Retracted );
            else
                _set( State::Retracting );
        }
        else if ( _goal == State::Expanded ) {
            if ( ( pos + _endThreshold >= _expandedPosition ) )
                _set( State::Expanded );
            else
                _set( State::Expanding );
        }
        _move();
    }
// TODO: private:
    void _move() {
        const int MAX_POWER = 40;
        const int pos = _position();
        if ( _currentState == State::Expanding )
            _motor.set( _coef( pos ) * -MAX_POWER );
        else if ( _currentState == State::Retracting )
            _motor.set( _coef( pos ) * MAX_POWER );
        else
            _motor.set( 0 );
    }

    void _set( State s ) {
        if ( s != _currentState )
            _onStateChange();
        _currentState = s;
        switch (_goal)
        {
        case State::Retracting:
        case State::Retracted:
            _goalPosition = _retractedPosition;
            break;
        case State::Expanding:
        case State::Expanded:
            _goalPosition = _expandedPosition;
            break;
        default:
            break;
        }
    }

    void _onStateChange() {
        // REMOVE:
    }

    float _coef( int position ) {
        const int threshold = 30;
        const int positionFromGoal = std::abs( position - _goalPosition );
        if ( positionFromGoal <= _endThreshold ) {
            return 0;
        } else if ( positionFromGoal <= threshold ) {
            const float min_coef = 0.5f;
            float coef = float(positionFromGoal) / 100 + 0.5f;
            return std::max(coef, min_coef);
        }
        return 1;
    }

    int _position() {
        auto readCount = 0;
        auto readPosSum = 0;
        for ( auto i = 0; auto posPin : _positionPins ) {
            if ( ! posPin.read() ) {
                ++readCount;
                readPosSum += i;
            }
            ++i;
        }
        return ( 100 * readPosSum / ( readCount != 0 ? readCount : 1 ) )  / ( _positionPins.size() - 1 );
    }

    Motor _motor;
    const std::array< Gpio::Pin, 10 > /* Error with cycle referencing?: decltype( bsp::posPins )*/ & _positionPins;
    State _goal;
    State _currentState;
    uint8_t _goalPosition;
    const uint8_t _retractedPosition = 0;
    const uint8_t _expandedPosition = 100;
    const uint8_t _endThreshold = 15;

};
