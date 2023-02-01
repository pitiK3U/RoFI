cmake_minimum_required(VERSION 3.6)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")

# project(i2c_test C CXX ASM)

set(ROFICOM_MCU "STM32G071xx")
set(ROFICOM_MCU_SPEC "STM32G071GBUx")

file(GLOB_RECURSE LIDAR vl53l1_lidar/*.cpp vl53l1_lidar/*.c)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmax-errors=5")

# setup_stm32cxx("g0" ${ROFICOM_MCU} ${ROFICOM_MCU_SPEC})

add_stm32_target(
        LIB
        TARGET vl53l1_lidar
        LIBTYPE STATIC
        MCU ${ROFICOM_MCU}
        MCU_SPEC ${ROFICOM_MCU_SPEC}
        LINKER_SCRIPT $ENV{ROFI_ROOT}/hardwareModules/RoFICoM/software/i2c_test/stm32/STM32G071GBUx_FLASH.ld
        FILES ${LIDAR})
target_include_directories(vl53l1_lidar PUBLIC
        vl53l1_lidar/core/inc
        vl53l1_lidar/platform/inc)
target_link_libraries(vl53l1_lidar
        g0_stm32cxx_i2c g0_stm32cxx_gpio)
target_compile_definitions(vl53l1_lidar PUBLIC VL53L1_DEBUG=1 PAL_EXTENDED=1)

# add_executable(${PROJECT_NAME}.elf ${SRC})
# set_property(TARGET ${PROJECT_NAME}.elf PROPERTY CXX_STANDARD 17)
# target_include_directories(${PROJECT_NAME}.elf PRIVATE src)
# target_compile_options(${PROJECT_NAME}.elf PRIVATE -Wall -Wextra -pedantic -funwind-tables)
# target_link_options(${PROJECT_NAME}.elf PRIVATE "-Wl,-Map=${PROJECT_NAME}.map,--cref" "-Wl,--print-memory-usage" "-funwind-tables" "-fasynchronous-unwind-tables")
# target_link_libraries(${PROJECT_NAME}.elf stm32cxx function2)
# # set_target_properties(${PROJECT_NAME}.elf PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")

# add_custom_target(flash
#     COMMAND $ENV{ROFI_ROOT}/hardwareModules/RoFICoM/software/i2c_test/tools/flashAll.sh --format ihex write ${HEX_FILE}
#     DEPENDS ${HEX_FILE})
