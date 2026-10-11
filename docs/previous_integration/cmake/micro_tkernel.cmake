# Official TRON Forum Pico BSP, pinned. SDK owns CRT/boot2/clocks; kernel owns scheduler/SysTick.
set(MTKERNEL_PATH "${CMAKE_SOURCE_DIR}/third_party/mtk3_bsp" CACHE PATH "Official pico_rp2040 BSP checkout")
if(NOT EXISTS "${MTKERNEL_PATH}/kernel/sysdepend/cpu/core/armv6m/dispatch.S")
    message(FATAL_ERROR "Missing real micro T-Kernel Pico BSP. Follow docs/BUILD.md; no RTOS substitute is provided.")
endif()
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/tk_config")
file(READ "${MTKERNEL_PATH}/config/config.h" tk_config)
foreach(feature USE_TMONITOR USE_SYSTEM_MESSAGE USE_SDEV_DRV USE_PTMR)
    string(REGEX REPLACE "(#define[ \t]+${feature}[ \t]+)\\([01]\\)" "\\1(0)" tk_config "${tk_config}")
endforeach()
# 1 ms tick permits the proposed 10 ms blocking loop without systematic extra tick delay.
string(REGEX REPLACE "(#define[ \t]+CNF_TIMER_PERIOD[ \t]+)[0-9]+" "\\1 1" tk_config "${tk_config}")
file(WRITE "${CMAKE_BINARY_DIR}/tk_config/config.h" "${tk_config}")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/tk_config/sys")
file(READ "${MTKERNEL_PATH}/include/sys/inittask.h" tk_initial_task)
string(REPLACE "(1*1024)" "(4*1024)" tk_initial_task "${tk_initial_task}")
file(WRITE "${CMAKE_BINARY_DIR}/tk_config/sys/inittask.h" "${tk_initial_task}")
file(GLOB tk_common CONFIGURE_DEPENDS "${MTKERNEL_PATH}/kernel/tkernel/*.c"
     "${MTKERNEL_PATH}/kernel/tstdlib/*.c" "${MTKERNEL_PATH}/lib/libtk/*.c")
set(tk_cpu "${MTKERNEL_PATH}/kernel/sysdepend/cpu/core/armv6m")
add_library(micro_tkernel STATIC ${tk_common}
    ${tk_cpu}/cpu_cntl.c ${tk_cpu}/dispatch.S ${tk_cpu}/exc_hdr.c ${tk_cpu}/interrupt.c
    ${MTKERNEL_PATH}/kernel/sysdepend/pico_rp2040/devinit.c
    ${MTKERNEL_PATH}/kernel/sysdepend/pico_rp2040/power_save.c
    ${MTKERNEL_PATH}/kernel/inittask/inittask.c
    ${MTKERNEL_PATH}/kernel/sysinit/sysinit.c
    ${MTKERNEL_PATH}/lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c
    ${MTKERNEL_PATH}/lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c)
set_source_files_properties(${MTKERNEL_PATH}/kernel/sysinit/sysinit.c
    PROPERTIES COMPILE_DEFINITIONS "main=robo_kernel_main")
target_include_directories(micro_tkernel PUBLIC "${CMAKE_BINARY_DIR}/tk_config"
    "${MTKERNEL_PATH}/include" "${MTKERNEL_PATH}/config"
    PRIVATE "${MTKERNEL_PATH}/kernel/knlinc")
target_compile_definitions(micro_tkernel PUBLIC _PICO_RP2040_)
target_compile_options(micro_tkernel PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wno-incompatible-pointer-types;-Wno-int-conversion>)
target_link_libraries(micro_tkernel PRIVATE pico_stdlib)
