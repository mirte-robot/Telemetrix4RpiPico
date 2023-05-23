cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 14)

set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)
project(Telemetrix4RpiPico)
pico_sdk_init()
add_executable(Telemetrix4RpiPico
        Telemetrix4RpiPico.c
)

pico_generate_pio_header(Telemetrix4RpiPico ${CMAKE_CURRENT_LIST_DIR}/Telemetrix4RpiPico.pio)
pico_enable_stdio_usb(Telemetrix4RpiPico 1)
#pico_enable_stdio_uart(Telemetrix4RpiPico 1)
pico_add_extra_outputs(Telemetrix4RpiPico)
target_link_libraries(Telemetrix4RpiPico pico_stdlib hardware_pwm
        pico_unique_id hardware_watchdog hardware_adc hardware_i2c
        hardware_pio hardware_clocks hardware_spi)
