
cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BNGU_Control_System)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    # ─── BSP ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp/bsp_uart.c

    # ─── DRV ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_buzzer/drv_buzzer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_dbus/drv_dbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_imu/drv_imu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_led/drv_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_motor/drv_motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_power_measure/drv_power_measure.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_vofa/drv_vofa.c

    # ─── LIB ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_filter/lib_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_math/lib_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_pid/lib_pid.c

    # ─── APP ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/App/common/bridge/app_buzzer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/App/common/bridge/app_dbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/App/common/bridge/app_imu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/App/common/bridge/app_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/App/common/bridge/app_vofa.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

    # ─── BSP ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Bsp

    # ─── DRV ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_buzzer
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_dbus
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_imu
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_led
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_power_measure
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Drv/Drv_vofa

    # ─── LIB ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_Typedef
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_filter
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_math
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/Lib/Lib_pid

    # ─── APP ────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core/App/common/bridge

    # ─── 项目根配置 ───────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/Self_core
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
