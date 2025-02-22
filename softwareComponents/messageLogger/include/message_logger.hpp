#pragma once

#include <chrono>
#include <optional>
#include <ostream>
#include <string_view>

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <google/protobuf/message.h>


namespace rofi::msgs
{

class MessageLogger {
public:
    MessageLogger( bool verbose ) : _verbose( verbose ), _ostr( nullptr ) {}
    MessageLogger( bool verbose, std::ostream & ostr ) : _verbose( verbose ), _ostr( &ostr ) {}

    void logSending( std::string_view topic, const google::protobuf::Message & msg )
    {
        if ( !createFormattedString() ) {
            return;
        }

        log( fmt::format( "{:%H:%M:%S} Sending to '{}':\n{}\n",
                          std::chrono::system_clock::now(),
                          topic,
                          msg.ShortDebugString() ) );
    }
    void logReceived( std::string_view topic, const google::protobuf::Message & msg )
    {
        if ( !createFormattedString() ) {
            return;
        }

        log( fmt::format( "{:%H:%M:%S} Received from '{}':\n{}\n",
                          std::chrono::system_clock::now(),
                          topic,
                          msg.ShortDebugString() ) );
    }

private:
    bool createFormattedString()
    {
        return _verbose || _ostr;
    }

    void log( const std::string & logStr )
    {
        if ( _verbose ) {
            std::cerr << logStr;
        }
        if ( _ostr ) {
            *_ostr << logStr;
        }
    }

    bool _verbose;
    std::ostream * _ostr;
};


} // namespace rofi::msgs
