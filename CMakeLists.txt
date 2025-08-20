cmake_minimum_required(VERSION 3.13)

# IMPORTANT: include Pico SDK import BEFORE project() so the toolchain is set for cross-compiling
include(cmake/pico_sdk_import.cmake)

project(pico_multi_apps C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Libraries
add_library(runcam
    lib/runcam/runcam_uart.c
    lib/runcam/runcam_crc.c
)
target_include_directories(runcam PUBLIC lib/runcam)
target_link_libraries(runcam PUBLIC pico_stdlib hardware_uart)

add_library(motion
    lib/motion/motion_detect.c
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/sensor/accel/mc36xx/m_drv_mc36xx.c
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/sensor/accel/m_drv_mc_utility.c
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/platform/bus/m_drv_interface.c
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/platform/console/m_drv_console.c
)
target_include_directories(motion PUBLIC lib/motion
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/sensor/accel
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/sensor/accel/mc36xx
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/platform/bus
    ${CMAKE_SOURCE_DIR}/EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv/platform/console)
target_compile_definitions(motion PRIVATE MCUBE_PICO_I2C_INSTANCE=i2c0 MCUBE_PICO_I2C_SDA_GPIO=4 MCUBE_PICO_I2C_SCL_GPIO=5 MCUBE_PICO_I2C_BAUD_HZ=100000)
target_link_libraries(motion PUBLIC pico_stdlib hardware_i2c)

# Apps
add_executable(camera_control
    apps/camera_control/main.c
)
target_link_libraries(camera_control PRIVATE runcam pico_stdlib hardware_uart)
pico_enable_stdio_usb(camera_control 1)
pico_enable_stdio_uart(camera_control 0)
pico_add_extra_outputs(camera_control)

add_executable(activity_detect
    apps/activity_detect/main.c
)
target_link_libraries(activity_detect PRIVATE motion pico_stdlib hardware_i2c)
pico_enable_stdio_usb(activity_detect 1)
pico_enable_stdio_uart(activity_detect 0)
pico_add_extra_outputs(activity_detect)

add_executable(hysteresis_recorder
    apps/hysteresis_recorder/main.c
)
target_link_libraries(hysteresis_recorder PRIVATE runcam motion pico_stdlib hardware_uart hardware_i2c pico_cyw43_arch_none)
pico_enable_stdio_usb(hysteresis_recorder 1)
pico_enable_stdio_uart(hysteresis_recorder 0)
pico_add_extra_outputs(hysteresis_recorder)


