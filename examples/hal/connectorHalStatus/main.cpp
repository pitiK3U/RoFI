#include <iostream>
#include <sstream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rofi_hal.hpp>

std::string getstatus(rofi::hal::LidarStatus status) {
    using namespace rofi::hal;
    switch (status) {
    case LidarStatus::Valid: return "Valid";
    case LidarStatus::OutsideRange: return "OutsideRange";
    case LidarStatus::NotMeasured: return "NotMeasured";
    case LidarStatus::Error: return "Error";
    default: assert(false && "unreachable");
    }

}

extern "C" void app_main() {
    try {
        std::cout << "Program starts\n";
        auto localRoFI = rofi::hal::RoFI::getLocalRoFI();
        std::cout << "Got local RoFI\n";
        auto conn = localRoFI.getConnector( 0 );

        while( true ) {
            const auto state = conn.getState();
            std::cout << "Voltage: " << state.internalVoltage << "\n";
            std::cout << "Lidar status: " << getstatus(state.lidarStatus) << " measured distance: " << state.distance << " mm\n";
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
        }


    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << "\n";
    }

    while ( true ) {
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}
