# micro T-Kernel 3.0 for Raspberry Pi Pico W.
# This is the CMake equivalent of build_make/pico_rp2040.mk.
cmake_minimum_required(VERSION 3.13)
option(ROBO_HOST_TESTS "Build portable Buddy 2 host tests only" OFF)
option(ROBO_MANUAL_CONSOLE "Enable deliberately armed motion calibration console" OFF)
option(ROBO_ENCODER_DIAGNOSTICS_ONLY "Read-only encoder/USB diagnostics; compile out motor activation" OFF)
option(ROBO_SAFE_MOTOR_TEST "Isolated 25 percent, one-second motor tests" OFF)
if(ROBO_HOST_TESTS)
    project(smart_robo_tests C)
    set(CMAKE_C_STANDARD 11)
    enable_testing()
    add_executable(motion_tests app_program/subsystem2/math.c app_program/subsystem2/motion.c
        app_program/subsystem2/board_config.c app_program/subsystem2/encoder_capture.c
        app_program/controller/vehicle_controller.c tests/test_motion.c)
    target_include_directories(motion_tests PRIVATE include)
    if(NOT MSVC)
        target_compile_options(motion_tests PRIVATE -Wall -Wextra -Werror -Wconversion -Wshadow)
        target_link_libraries(motion_tests m)
    endif()
    add_test(NAME motion_tests COMMAND motion_tests)
    add_executable(motor_test_checks app_program/subsystem2/motor_test.c
        app_program/subsystem2/board_config.c tests/test_motor_test.c)
    target_include_directories(motor_test_checks PRIVATE include tests)
    target_compile_definitions(motor_test_checks PRIVATE ROBO_SAFE_MOTOR_TEST=1 ROBO_MOTOR_TEST_HOST=1)
    if(NOT MSVC)
        target_compile_options(motor_test_checks PRIVATE -Wall -Wextra -Werror -Wconversion -Wshadow)
        target_link_libraries(motor_test_checks m)
    endif()
    add_test(NAME motor_test_checks COMMAND motor_test_checks)
    add_executable(line_test_checks app_program/subsystem3/subsystem3.c tests/test_line_sensor.c)
    target_include_directories(line_test_checks PRIVATE include tests)
    target_compile_definitions(line_test_checks PRIVATE ROBO_SAFE_MOTOR_TEST=1 ROBO_LINE_TEST_HOST=1)
    if(NOT MSVC)
        target_compile_options(line_test_checks PRIVATE -Wall -Wextra -Werror -Wconversion -Wshadow)
    endif()
    add_test(NAME line_test_checks COMMAND line_test_checks)
    return()
endif()

if(ROBO_SAFE_MOTOR_TEST AND (NOT ROBO_MANUAL_CONSOLE OR ROBO_ENCODER_DIAGNOSTICS_ONLY))
    message(FATAL_ERROR "Motor tests require the console and cannot use the read-only encoder profile.")
endif()
if(ROBO_ENCODER_DIAGNOSTICS_ONLY AND NOT ROBO_MANUAL_CONSOLE)
    message(FATAL_ERROR "Encoder diagnostics require ROBO_MANUAL_CONSOLE=ON for serial input.")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode "${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake")
if(EXISTS "${picoVscode}")
    include("${picoVscode}")
endif()
# ====================================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(mtkernel_3 C CXX ASM)
pico_sdk_init()

if(NOT PICO_PLATFORM STREQUAL "rp2040" OR NOT PICO_BOARD STREQUAL "pico_w")
    message(FATAL_ERROR "This kernel configuration targets the RP2040 Pico W; select board pico_w.")
endif()

# These settings have the same names and defaults as the GNU Make build.
option(SMP "Run the kernel on both RP2040 cores" OFF)
set(CONSOLE usb_cdc CACHE STRING "Console: usb_cdc or uart")
set_property(CACHE CONSOLE PROPERTY STRINGS usb_cdc uart)
set(WIFI none CACHE STRING "Radio: cyw43 or none")
set_property(CACHE WIFI PROPERTY STRINGS cyw43 none)
set(NET none CACHE STRING "Network stack: none or lwip")
set_property(CACHE NET PROPERTY STRINGS none lwip)
option(MQTT "Enable the lwIP MQTT client" OFF)
option(TEST "Run subsystem self-tests at startup" OFF)
option(CRASH_REPORT "Mirror console and crash reports to UART0 on GP0/GP1" OFF)
if(SMP)
    message(FATAL_ERROR "Buddy 2 GPIO snapshot/IRQ ownership currently requires SMP=OFF. Validate a cross-core port before enabling SMP.")
endif()
if(TEST)
    message(FATAL_ERROR "Template startup demos have been replaced. Use ROBO_HOST_TESTS for tests or ROBO_MANUAL_CONSOLE for deliberate hardware calibration.")
endif()

if(NOT CONSOLE MATCHES "^(usb_cdc|uart)$")
    message(FATAL_ERROR "CONSOLE must be usb_cdc or uart.")
endif()
if(NOT WIFI MATCHES "^(cyw43|none)$")
    message(FATAL_ERROR "WIFI must be cyw43 or none.")
endif()
if(NOT NET MATCHES "^(none|lwip)$")
    message(FATAL_ERROR "NET must be none or lwip.")
endif()
if(NET STREQUAL "lwip" AND NOT WIFI STREQUAL "cyw43")
    message(FATAL_ERROR "NET=lwip requires WIFI=cyw43.")
endif()
if(ROBO_SAFE_MOTOR_TEST AND (NOT WIFI STREQUAL "none" OR NOT NET STREQUAL "none" OR NOT CONSOLE STREQUAL "usb_cdc"))
    message(FATAL_ERROR "Safe motor tests require USB CDC and disabled WiFi/networking.")
endif()
if(MQTT AND NOT NET STREQUAL "lwip")
    message(FATAL_ERROR "MQTT requires NET=lwip.")
endif()
if(CONSOLE STREQUAL "uart")
    set(CRASH_REPORT ON)
endif()

# Only the common code and the RP2040/ARMv6-M platform are included.
# Application sources follow the Makefile's root + one subdirectory layout.
file(GLOB MTK_SOURCES CONFIGURE_DEPENDS
    "app_program/*.c"
    "app_program/*/*.c"
    "kernel/inittask/*.c"
    "kernel/sysinit/*.c"
    "kernel/usermain/*.c"
    "kernel/tstdlib/*.c"
    "kernel/tkernel/*.c"
    "kernel/sysdepend/pico_rp2040/*.c"
    "kernel/sysdepend/cpu/rp2040/*.c"
    "kernel/sysdepend/cpu/core/armv6m/*.c"
    "lib/libtk/*.c"
    "lib/libtk/sysdepend/cpu/rp2040/*.c"
    "lib/libtk/sysdepend/cpu/core/armv6m/*.c"
    "lib/libtm/*.c"
    "lib/libtm/sysdepend/pico_rp2040/*.c"
    "lib/libtm/sysdepend/no_device/*.c"
    "lib/libbsp/sysdepend/cpu/rp2040/*.c"
    "device/common/drvif/*.c"
    "device/adc/*.c"
    "device/adc/sysdepend/rp2040/*.c"
    "device/i2c/*.c"
    "device/i2c/sysdepend/rp2040/*.c"
    "device/ser/*.c"
    "device/ser/sysdepend/rp2040/*.c"
)

add_executable(mtkernel_3
    ${MTK_SOURCES}
    kernel/sysdepend/cpu/core/armv6m/dispatch.S
)
set_target_properties(mtkernel_3 PROPERTIES LINKER_LANGUAGE C)
target_include_directories(mtkernel_3 PRIVATE include config kernel/knlinc)
target_compile_definitions(mtkernel_3 PRIVATE
    $<$<BOOL:${ROBO_MANUAL_CONSOLE}>:ROBO_MANUAL_CONSOLE=1>
    $<$<BOOL:${ROBO_ENCODER_DIAGNOSTICS_ONLY}>:ROBO_ENCODER_DIAGNOSTICS_ONLY=1>
    $<$<BOOL:${ROBO_SAFE_MOTOR_TEST}>:ROBO_SAFE_MOTOR_TEST=1>
    _PICO_RP2040_
    CNF_SMP=$<BOOL:${SMP}>
    APP_TEST=$<BOOL:${TEST}>
    TM_CONSOLE_UART=$<BOOL:${CRASH_REPORT}>
    BOARD_UART0_FREE=$<NOT:$<BOOL:${CRASH_REPORT}>>
    TM_CONSOLE_USB_CDC=$<STREQUAL:${CONSOLE},usb_cdc>
    TM_WIFI_CYW43=$<STREQUAL:${WIFI},cyw43>
    TM_NET_LWIP=$<STREQUAL:${NET},lwip>
    TM_MQTT=$<BOOL:${MQTT}>
)
if(ROBO_ENCODER_DIAGNOSTICS_ONLY OR ROBO_SAFE_MOTOR_TEST)
    string(TIMESTAMP encoder_build_stamp "%Y%m%dT%H%M%SZ" UTC)
    if(ROBO_SAFE_MOTOR_TEST)
        set(encoder_build_id "b2-motor-test-v1-${encoder_build_stamp}")
    else()
        set(encoder_build_id "b2-encdiag-v1-${encoder_build_stamp}")
    endif()
    target_compile_definitions(mtkernel_3 PRIVATE ROBO_ENCODER_BUILD_ID="${encoder_build_id}")
    file(WRITE "${CMAKE_BINARY_DIR}/encoder-build-id.txt" "${encoder_build_id}\n")
endif()
target_compile_options(mtkernel_3 PRIVATE
    -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -ffreestanding -O2 -g3
    "$<$<COMPILE_LANGUAGE:ASM>:-x;assembler-with-cpp>"
)

# The kernel supplies boot2, vectors, Reset_Handler, main, and SDK hooks.
# Linking pico_stdlib would also introduce the SDK runtime into that startup.
set(MTK_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/etc/linker/pico_rp2040/tkernel_map.ld")
target_link_options(mtkernel_3 PRIVATE
    -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -ffreestanding
    -nostartfiles -O2 -g3 "-T${MTK_LINKER_SCRIPT}"
)
set_property(TARGET mtkernel_3 APPEND PROPERTY LINK_DEPENDS "${MTK_LINKER_SCRIPT}")

set(MTK_USB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/libtm/sysdepend/pico_rp2040/usb")
set(MTK_WIFI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/libwifi/sysdepend/pico_rp2040")
set(MTK_LWIP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/liblwip/sysdepend/pico_rp2040")

if(CONSOLE STREQUAL "usb_cdc" OR WIFI STREQUAL "cyw43")
    # Use the same compatibility headers and SDK header paths as the Makefile.
    # SDK CMake libraries are not linked: the kernel supplies their OS hooks.
    target_include_directories(mtkernel_3 PRIVATE
        "${MTK_USB_DIR}"
        "${PICO_SDK_PATH}/src/common/pico_base_headers/include"
        "${PICO_SDK_PATH}/src/common/pico_binary_info/include"
        "${PICO_SDK_PATH}/src/rp2040/pico_platform/include"
        "${PICO_SDK_PATH}/src/rp2040/hardware_regs/include"
        "${PICO_SDK_PATH}/src/rp2040/hardware_structs/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_common/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_sections/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_panic/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_base/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_irq/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_resets/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_sync_spin_lock/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include"
    )
    target_compile_definitions(mtkernel_3 PRIVATE NDEBUG)
endif()

if(CONSOLE STREQUAL "usb_cdc")
    set(TINYUSB_PATH "${PICO_SDK_PATH}/lib/tinyusb")
    if(NOT EXISTS "${TINYUSB_PATH}/src/tusb.c")
        message(FATAL_ERROR "TinyUSB is missing. Initialize the Pico SDK lib/tinyusb submodule.")
    endif()
    target_include_directories(mtkernel_3 PRIVATE "${TINYUSB_PATH}/src")
    target_sources(mtkernel_3 PRIVATE
        "${MTK_USB_DIR}/usb_console.c"
        "${MTK_USB_DIR}/usb_descriptors.c"
        "${MTK_USB_DIR}/usb_sdk_compat.c"
        "${MTK_USB_DIR}/usb_tinyusb_glue.c"
        "${TINYUSB_PATH}/src/tusb.c"
        "${TINYUSB_PATH}/src/common/tusb_fifo.c"
        "${TINYUSB_PATH}/src/device/usbd.c"
        "${TINYUSB_PATH}/src/device/usbd_control.c"
        "${TINYUSB_PATH}/src/class/cdc/cdc_device.c"
        "${TINYUSB_PATH}/src/portable/raspberrypi/rp2040/rp2040_usb.c"
        "${TINYUSB_PATH}/src/portable/raspberrypi/rp2040/dcd_rp2040.c"
    )
endif()

if(WIFI STREQUAL "cyw43")
    set(CYW43_DRIVER_PATH "${PICO_SDK_PATH}/lib/cyw43-driver")
    set(CYW43_PICO_PATH "${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver")
    if(NOT EXISTS "${CYW43_DRIVER_PATH}/src/cyw43_ll.c")
        message(FATAL_ERROR "CYW43 is missing. Initialize the Pico SDK lib/cyw43-driver submodule.")
    endif()
    target_sources(mtkernel_3 PRIVATE
        "${MTK_WIFI_DIR}/cyw43_utk.c"
        "${MTK_WIFI_DIR}/cyw43_utk_kernel.c"
        "${MTK_WIFI_DIR}/cyw43_utk_compat.c"
        "${CYW43_DRIVER_PATH}/src/cyw43_ll.c"
        "${CYW43_DRIVER_PATH}/src/cyw43_ctrl.c"
        "${CYW43_DRIVER_PATH}/src/cyw43_stats.c"
        "${CYW43_PICO_PATH}/cyw43_bus_pio_spi.c"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_pio/pio.c"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_dma/dma.c"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/gpio.c"
    )
    target_include_directories(mtkernel_3 PRIVATE
        "${MTK_WIFI_DIR}"
        "${CYW43_DRIVER_PATH}/src"
        "${CYW43_DRIVER_PATH}/firmware"
        "${CYW43_PICO_PATH}/include"
        "${PICO_SDK_PATH}/src/boards/include"
        "${PICO_SDK_PATH}/src/common/hardware_claim/include"
        "${PICO_SDK_PATH}/src/common/pico_util/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_pio/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_dma/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_clocks/include"
    )
    target_compile_definitions(mtkernel_3 PRIVATE
        CYW43_LWIP=$<STREQUAL:${NET},lwip>
        CYW43_ENABLE_BLUETOOTH=0 CYW43_USE_OTP_MAC=1 PICO_CYW43_LOGGING_ENABLED=0
        PICO_RP2040=1 PICO_32BIT=1 PICO_ON_DEVICE=1 PICO_BUILD=1 PICO_NO_HARDWARE=0
        RASPBERRYPI_PICO_W=1
        CYW43_DEFAULT_PIN_WL_REG_ON=23u
        CYW43_DEFAULT_PIN_WL_DATA_OUT=24u
        CYW43_DEFAULT_PIN_WL_DATA_IN=24u
        CYW43_DEFAULT_PIN_WL_HOST_WAKE=24u
        CYW43_DEFAULT_PIN_WL_CLOCK=29u
        CYW43_DEFAULT_PIN_WL_CS=25u
    )
    target_compile_options(mtkernel_3 PRIVATE -ffunction-sections -fdata-sections)
    target_link_options(mtkernel_3 PRIVATE -Wl,--gc-sections)
endif()

if(NET STREQUAL "lwip")
    set(LWIP_PATH "${PICO_SDK_PATH}/lib/lwip/src")
    if(NOT EXISTS "${LWIP_PATH}/core/init.c")
        message(FATAL_ERROR "lwIP is missing. Initialize the Pico SDK lib/lwip submodule.")
    endif()
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/wifi_secrets.h")
        message(FATAL_ERROR "NET=lwip needs config/wifi_secrets.h; use wifi_secrets.example.h as the template.")
    endif()
    set(LWIP_SOURCES
        core/init.c core/def.c core/dns.c core/inet_chksum.c
        core/ip.c core/mem.c core/memp.c core/netif.c core/pbuf.c
        core/raw.c core/stats.c core/sys.c
        core/altcp.c core/altcp_alloc.c core/altcp_tcp.c
        core/tcp.c core/tcp_in.c core/tcp_out.c core/timeouts.c core/udp.c
        core/ipv4/acd.c core/ipv4/autoip.c core/ipv4/dhcp.c
        core/ipv4/etharp.c core/ipv4/icmp.c core/ipv4/igmp.c
        core/ipv4/ip4_frag.c core/ipv4/ip4.c core/ipv4/ip4_addr.c netif/ethernet.c
    )
    foreach(LWIP_SOURCE IN LISTS LWIP_SOURCES)
        target_sources(mtkernel_3 PRIVATE "${LWIP_PATH}/${LWIP_SOURCE}")
    endforeach()
    target_sources(mtkernel_3 PRIVATE
        "${MTK_LWIP_DIR}/sys_arch.c"
        "${CYW43_DRIVER_PATH}/src/cyw43_lwip.c"
    )
    target_include_directories(mtkernel_3 PRIVATE
        "${MTK_LWIP_DIR}/include" "${MTK_LWIP_DIR}" "${LWIP_PATH}/include"
    )
    if(MQTT)
        target_sources(mtkernel_3 PRIVATE
            "${LWIP_PATH}/apps/mqtt/mqtt.c"
            "${MTK_LWIP_DIR}/mqtt_utk.c"
        )
    endif()
endif()

# Use the extension's picotool to produce UF2 without a separate host compiler.
pico_add_extra_outputs(mtkernel_3)
if(ROBO_ENCODER_DIAGNOSTICS_ONLY)
    add_custom_command(TARGET mtkernel_3 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_BINARY_DIR}/mtkernel_3.uf2"
        "${CMAKE_CURRENT_BINARY_DIR}/buddy2_encoder_diag.uf2")
endif()
target_link_libraries(mtkernel_3 PRIVATE m)
if(ROBO_SAFE_MOTOR_TEST)
    add_custom_command(TARGET mtkernel_3 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_BINARY_DIR}/mtkernel_3.uf2"
        "${CMAKE_CURRENT_BINARY_DIR}/buddy2_safe_motor_test.uf2")
endif()
