cmake_minimum_required(VERSION 3.6)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")

project(i2c_test C CXX ASM)

set(ROFICOM_MCU "STM32G071xx")
set(ROFICOM_MCU_SPEC "STM32G071GBUx")

file(GLOB_RECURSE SRC src/*.cpp src/*.c)
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


add_stm32_target(
        EXECUTABLE
        TARGET i2cFirmware
        MCU ${ROFICOM_MCU}
        MCU_SPEC ${ROFICOM_MCU_SPEC}
        LINKER_SCRIPT $ENV{ROFI_ROOT}/hardwareModules/RoFICoM/software/i2c_test/stm32/STM32G071GBUx_FLASH.ld
        FILES ${SRC})
set_property(TARGET i2cFirmware PROPERTY CXX_STANDARD 17)
target_include_directories(i2cFirmware PRIVATE src)
target_compile_options(i2cFirmware PRIVATE
        -Wall -Wextra -pedantic -funwind-tables)
target_link_libraries(i2cFirmware
        g0_startup function2 g0_stm32cxx g0_HAL_CRC
        g0_stm32cxx_system g0_stm32cxx_i2c g0_stm32cxx_uart g0_stm32cxx_gpio
        g0_stm32cxx_dma g0_stm32cxx_timer vl53l1_lidar)

# add_executable(${PROJECT_NAME}.elf ${SRC})
# set_property(TARGET ${PROJECT_NAME}.elf PROPERTY CXX_STANDARD 17)
# target_include_directories(${PROJECT_NAME}.elf PRIVATE src)
# target_compile_options(${PROJECT_NAME}.elf PRIVATE -Wall -Wextra -pedantic -funwind-tables)
# target_link_options(${PROJECT_NAME}.elf PRIVATE "-Wl,-Map=${PROJECT_NAME}.map,--cref" "-Wl,--print-memory-usage" "-funwind-tables" "-fasynchronous-unwind-tables")
# target_link_libraries(${PROJECT_NAME}.elf stm32cxx function2)
# # set_target_properties(${PROJECT_NAME}.elf PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")

set(EXE_FILE i2cFirmware)
set(HEX_FILE ${PROJECT_BINARY_DIR}/${EXE_FILE}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${EXE_FILE}.bin)
message(${PROJECT_BINARY_DIR}/${EXE_FILE}.bin)
add_custom_command(TARGET ${EXE_FILE} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXE_FILE}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXE_FILE}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")

# add_custom_target(flash
#     COMMAND $ENV{ROFI_ROOT}/hardwareModules/RoFICoM/software/i2c_test/tools/flashAll.sh --format ihex write ${HEX_FILE}
#     DEPENDS ${HEX_FILE})
