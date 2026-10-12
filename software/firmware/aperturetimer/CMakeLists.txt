cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(
    daspi_aperture_timer
    VERSION 1.0
    LANGUAGES C CXX ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

add_executable(
    daspi-aperture-timer
    src/main.cpp
)

target_link_libraries(
    daspi-aperture-timer
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(
    daspi-aperture-timer
    1
)

pico_enable_stdio_uart(
    daspi-aperture-timer
    0
)

pico_add_extra_outputs(
    daspi-aperture-timer
)
