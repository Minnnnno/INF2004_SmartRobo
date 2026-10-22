cmake_minimum_required(VERSION 3.13)
project(buddy2_compact_checks C)
set(CMAKE_C_STANDARD 11)
enable_testing()
add_executable(buddy2_compact_checks test_compact.c
    ../../app_program/subsystem2/motion.c ../../app_program/subsystem2/math.c
    ../../app_program/subsystem2/board_config.c ../../app_program/subsystem2/encoder_capture.c)
target_include_directories(buddy2_compact_checks PRIVATE ../../include)
if(NOT MSVC)
    target_compile_options(buddy2_compact_checks PRIVATE
        -Wall -Wextra -Werror -Wconversion -Wshadow -fsanitize=undefined)
    target_link_options(buddy2_compact_checks PRIVATE -fsanitize=undefined)
    target_link_libraries(buddy2_compact_checks m)
endif()
add_test(NAME buddy2_compact_checks COMMAND buddy2_compact_checks)
add_executable(buddy2_driver_checks test_driver.c ../../app_program/subsystem2/drv_motion.c
    ../../app_program/subsystem2/math.c ../../app_program/subsystem2/board_config.c
    ../../app_program/subsystem2/encoder_capture.c)
target_include_directories(buddy2_driver_checks PRIVATE ../../include .)
target_compile_definitions(buddy2_driver_checks PRIVATE ROBO_TEMPLATE_HAL_H=1 TM_CONSOLE_USB_CDC=1)
target_compile_options(buddy2_driver_checks PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/driver_fake.h
    -Wall -Wextra -Werror -Wconversion -Wshadow -fsanitize=undefined)
target_link_options(buddy2_driver_checks PRIVATE -fsanitize=undefined)
target_link_libraries(buddy2_driver_checks m)
add_test(NAME buddy2_driver_checks COMMAND buddy2_driver_checks)
