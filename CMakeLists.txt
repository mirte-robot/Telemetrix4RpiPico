cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 20)

set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

project(Telemetrix4RpiPico)
pico_sdk_init()
add_executable(Telemetrix4RpiPico
        src/Telemetrix4RpiPico.cpp
        src/VL53L0.cpp
        src/i2c_helpers.cpp
        src/HX711.cpp
        src/INA226.cpp
        src/INA226v2.cpp
        src/HiwonderServo.cpp
)
target_include_directories(Telemetrix4RpiPico PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include/)
# target_compile_options(Telemetrix4RpiPico PRIVATE -Wall -Wextra -Wpedantic -Werror)
pico_generate_pio_header(Telemetrix4RpiPico ${CMAKE_CURRENT_LIST_DIR}/src/Telemetrix4RpiPico.pio)
pico_enable_stdio_usb(Telemetrix4RpiPico 1)
pico_enable_stdio_uart(Telemetrix4RpiPico 1)
pico_add_extra_outputs(Telemetrix4RpiPico)
target_link_libraries(Telemetrix4RpiPico pico_stdlib hardware_pwm
        pico_unique_id hardware_watchdog hardware_adc hardware_i2c
        hardware_pio hardware_clocks hardware_spi)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(lint
        COMMAND sh -c "run-clang-tidy ${CMAKE_SOURCE_DIR}/src/Telemetrix4RpiPico.cpp -checks=`tr '\\n' , <${CMAKE_SOURCE_DIR}/checks.txt` >lint.out 2>lint.err"
        COMMAND sh -c "grep warning: lint.out || true"
        COMMAND ls -lh ${CMAKE_BINARY_DIR}/lint.out
        VERBATIM
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)