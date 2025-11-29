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

project(ads131_dma_bench C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ads131_dma_bench
    main.c
    ads131a04.c
)

target_include_directories(ads131_dma_bench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(ads131_dma_bench
    pico_stdlib
    hardware_spi
    hardware_dma
)

pico_enable_stdio_usb(ads131_dma_bench 1)
pico_enable_stdio_uart(ads131_dma_bench 0)

pico_add_extra_outputs(ads131_dma_bench)
