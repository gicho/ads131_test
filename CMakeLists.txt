cmake_minimum_required(VERSION 3.13)

# Force Pico 2 / RP2350
set(PICO_PLATFORM rp2350)
set(PICO_BOARD pico2)

# Try to get PICO_SDK_PATH from cache or environment
if (NOT DEFINED PICO_SDK_PATH)
    if (DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    else()
        message(FATAL_ERROR "PICO_SDK_PATH is not set. Please set it in your environment or CMake cache.")
    endif()
endif()

# This is the file that was missing before
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ads131_test C CXX)

pico_sdk_init()

add_executable(ads131_test
    main.c
    ads131a04.c
)

target_include_directories(ads131_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(ads131_test
    pico_stdlib
    hardware_spi
)

pico_enable_stdio_usb(ads131_test 1)
pico_enable_stdio_uart(ads131_test 0)

pico_add_extra_outputs(ads131_test)
