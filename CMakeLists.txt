cmake_minimum_required(VERSION 3.20)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(pcm1808_stereo_mic C CXX ASM)

pico_sdk_init()

add_executable(pcm1808_stereo_mic
    main.c
    usb_descriptors.c
)

pico_generate_pio_header(
    pcm1808_stereo_mic
    ${CMAKE_CURRENT_LIST_DIR}/i2s_slave.pio
)

target_include_directories(pcm1808_stereo_mic PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(pcm1808_stereo_mic
    pico_stdlib

    hardware_pio
    hardware_dma
    hardware_irq
    hardware_sync
    hardware_clocks

    tinyusb_device
    tinyusb_board
)

pico_enable_stdio_usb(pcm1808_stereo_mic 0)
pico_enable_stdio_uart(pcm1808_stereo_mic 0)

pico_add_extra_outputs(pcm1808_stereo_mic)
