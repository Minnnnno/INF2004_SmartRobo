cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Pico W calibration board")
include(../../pico_sdk_import.cmake)
project(buddy2_encoder_calibration C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(buddy2_encoder_calibration main.c
    ../../app_program/subsystem2/encoder_capture.c
    ../../app_program/subsystem2/math.c)
target_include_directories(buddy2_encoder_calibration PRIVATE ../../include)
target_link_libraries(buddy2_encoder_calibration PRIVATE pico_stdlib hardware_gpio hardware_irq m)
set_source_files_properties(main.c ../../app_program/subsystem2/encoder_capture.c
    ../../app_program/subsystem2/math.c PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wconversion;-Wshadow")
string(TIMESTAMP calibration_stamp "%Y%m%dT%H%M%SZ" UTC)
set(calibration_id "b2-passive-sdk-usb-v1-${calibration_stamp}")
target_compile_definitions(buddy2_encoder_calibration PRIVATE
    CALIBRATION_BUILD_ID="${calibration_id}"
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=0
    PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE=0
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=20000)
file(WRITE "${CMAKE_BINARY_DIR}/firmware-build-id.txt" "${calibration_id}\n")
pico_enable_stdio_usb(buddy2_encoder_calibration 1)
pico_enable_stdio_uart(buddy2_encoder_calibration 0)
pico_add_extra_outputs(buddy2_encoder_calibration)
