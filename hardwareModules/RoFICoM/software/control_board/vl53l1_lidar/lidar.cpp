#include "lidar.hpp"

#include <string_view>

// REMOVE:

// #include <vl53l1_api.h>
// #undef VL53L1_USE_EMPTY_STRING
// #include <vl53l1_error_strings.h>
       /*
std::string_view Lidar::errorToString( VL53L1X_ERROR err )
    {
        static char buffer[32] = {};
        std::snprintf(buffer, 32, "%d", err);
        return std::string_view( buffer );
        // Stolen from `core/src/vl53l1_error_strings.c:102`
 
        switch ( err )
        {
        case VL53L1_ERROR_NONE:
            return std::string_view(VL53L1_STRING_ERROR_NONE);

        case VL53L1_ERROR_CALIBRATION_WARNING:
            return std::string_view(VL53L1_STRING_ERROR_CALIBRATION_WARNING);

        case VL53L1_ERROR_MIN_CLIPPED:
            return std::string_view(VL53L1_STRING_ERROR_MIN_CLIPPED);

        case VL53L1_ERROR_UNDEFINED:
            return std::string_view(VL53L1_STRING_ERROR_UNDEFINED);

        case VL53L1_ERROR_INVALID_PARAMS:
            return std::string_view(VL53L1_STRING_ERROR_INVALID_PARAMS);

        case VL53L1_ERROR_NOT_SUPPORTED:
            return std::string_view(VL53L1_STRING_ERROR_NOT_SUPPORTED);

        case VL53L1_ERROR_RANGE_ERROR:
            return std::string_view(VL53L1_STRING_ERROR_RANGE_ERROR);

        case VL53L1_ERROR_TIME_OUT:
            return std::string_view(VL53L1_STRING_ERROR_TIME_OUT);

        case VL53L1_ERROR_MODE_NOT_SUPPORTED:
            return std::string_view(VL53L1_STRING_ERROR_MODE_NOT_SUPPORTED);

        case VL53L1_ERROR_BUFFER_TOO_SMALL:
            return std::string_view(VL53L1_STRING_ERROR_BUFFER_TOO_SMALL);

        case VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL:
            return std::string_view(VL53L1_STRING_ERROR_COMMS_BUFFER_TOO_SMALL);

        case VL53L1_ERROR_GPIO_NOT_EXISTING:
            return std::string_view(VL53L1_STRING_ERROR_GPIO_NOT_EXISTING);

        case VL53L1_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
            return std::string_view(VL53L1_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);

        case VL53L1_ERROR_CONTROL_INTERFACE:
            return std::string_view(VL53L1_STRING_ERROR_CONTROL_INTERFACE);

        case VL53L1_ERROR_INVALID_COMMAND:
            return std::string_view(VL53L1_STRING_ERROR_INVALID_COMMAND);

        case VL53L1_ERROR_DIVISION_BY_ZERO:
            return std::string_view(VL53L1_STRING_ERROR_DIVISION_BY_ZERO);

        case VL53L1_ERROR_REF_SPAD_INIT:
            return std::string_view(VL53L1_STRING_ERROR_REF_SPAD_INIT);

        case VL53L1_ERROR_GPH_SYNC_CHECK_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_GPH_SYNC_CHECK_FAIL);

        case VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_STREAM_COUNT_CHECK_FAIL);

        case VL53L1_ERROR_GPH_ID_CHECK_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_GPH_ID_CHECK_FAIL);

        case VL53L1_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL);

        case VL53L1_ERROR_ZONE_GPH_ID_CHECK_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_ZONE_GPH_ID_CHECK_FAIL);


        case VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_XTALK_EXTRACTION_NO_SAMPLES_FAIL);

        case VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL);


        case VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL);

        case VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL);

        case VL53L1_ERROR_ZONE_CAL_NO_SAMPLE_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_ZONE_CAL_NO_SAMPLE_FAIL);


        case VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES:
            return std::string_view(VL53L1_STRING_WARNING_OFFSET_CAL_MISSING_SAMPLES);

        case VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH:
            return std::string_view(VL53L1_STRING_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH);

        case VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH:
            return std::string_view(VL53L1_STRING_WARNING_OFFSET_CAL_RATE_TOO_HIGH);

        case VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW:
            return std::string_view(VL53L1_STRING_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW);


        case VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES:
            return std::string_view(VL53L1_STRING_WARNING_ZONE_CAL_MISSING_SAMPLES);

        case VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH:
            return std::string_view(VL53L1_STRING_WARNING_ZONE_CAL_SIGMA_TOO_HIGH);

        case VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH:
            return std::string_view(VL53L1_STRING_WARNING_ZONE_CAL_RATE_TOO_HIGH);


        case VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS:
            return std::string_view(VL53L1_STRING_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS);

        case VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH:
            return std::string_view(VL53L1_STRING_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH);

        case VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW:
            return std::string_view(VL53L1_STRING_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW);


        case VL53L1_WARNING_XTALK_MISSING_SAMPLES:
            return std::string_view(VL53L1_STRING_WARNING_XTALK_MISSING_SAMPLES);

        case VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT:
            return std::string_view(VL53L1_STRING_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT);

        case VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT:
            return std::string_view(VL53L1_STRING_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT);


        case VL53L1_ERROR_DEVICE_FIRMWARE_TOO_OLD:
            return std::string_view(VL53L1_STRING_ERROR_DEVICE_FIRMWARE_TOO_OLD);

        case VL53L1_ERROR_DEVICE_FIRMWARE_TOO_NEW:
            return std::string_view(VL53L1_STRING_ERROR_DEVICE_FIRMWARE_TOO_NEW);

        case VL53L1_ERROR_UNIT_TEST_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_UNIT_TEST_FAIL);

        case VL53L1_ERROR_FILE_READ_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_FILE_READ_FAIL);

        case VL53L1_ERROR_FILE_WRITE_FAIL:
            return std::string_view(VL53L1_STRING_ERROR_FILE_WRITE_FAIL);

        case VL53L1_ERROR_NOT_IMPLEMENTED:
            return std::string_view(VL53L1_STRING_ERROR_NOT_IMPLEMENTED);

        default:
            return std::string_view(VL53L1_STRING_UNKNOW_ERROR_CODE);
        }
    }
    */