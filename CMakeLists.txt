cmake_minimum_required(VERSION 3.13)

# Force Pico 2 / RP2350
set(PICO_PLATFORM rp2350)
set(PICO_BOARD pico2)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if (NOT DEFINED PICO_SDK_PATH)
    if (DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    else()
        message(FATAL_ERROR "PICO_SDK_PATH is not set")
    endif()
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ads131_test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ads131_test
    main.c
    ads131a04.c
    ads131a04_capture.pio
)

# Generate PIO header from the .pio source in the project root
pico_generate_pio_header(ads131_test
    ${CMAKE_CURRENT_LIST_DIR}/ads131a04_capture.pio
)

target_include_directories(ads131_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(ads131_test
    pico_stdlib
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_clocks
)

pico_enable_stdio_usb(ads131_test 1)
pico_enable_stdio_uart(ads131_test 0)

pico_add_extra_outputs(ads131_test)
