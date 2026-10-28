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
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Gimbal_DM_Shoot)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/main.cpp
        User/Tasks/INS_Task/INSTask.hpp
        User/MCUDriver/CANio/CANio.cpp
        User/MCUDriver/CANio/CANio.hpp
        User/Tasks/PRINT/Task_Print.cpp
        User/Tasks/PRINT/PrintTelemetry.cpp
        User/Tasks/PRINT/PrintTelemetry.hpp
        User/Tasks/GIMBAL/Task_Gimbal.cpp
        User/Tasks/GIMBAL/Task_Gimbal.hpp
        User/Framework/GIMBAL/Gimbal.cpp
        User/Framework/GIMBAL/PitchTilt.cpp
        User/Framework/GIMBAL/legacy/CanDemoPid.cpp
        User/Framework/GIMBAL/PitchTilt.hpp
        User/Framework/MOTOR/Motor.cpp
        User/Framework/MOTOR/Motor.hpp
        User/Definitions.hpp
        User/Framework/WDG/Wdg.cpp
        User/Framework/PID/Pid.hpp
        User/Framework/PID/Pid.cpp
        User/Framework/REMOTEC/remotec.cpp
        User/Framework/REMOTEC/remotec.hpp
        User/Framework/REMOTEC/remote_struct.h
        User/Framework/REMOTEC/VTMC/CRCCheck.cpp
        User/Framework/REMOTEC/VTMC/CRCCheck.h
        User/Framework/REMOTEC/VTMC/vtm_struct.h
        User/Framework/USARTIO/usartio.cpp
        User/Framework/AUTOAIM/AutoAim.cpp
        User/Framework/AUTOAIM/AutoAim.hpp
        User/Tasks/VISION/Task_Vision.cpp
        User/Framework/LQR/Lqr.cpp
        User/Framework/LQR/Lqr.hpp
        User/Framework/DEBUGC/debugc.cpp
        User/Framework/DEBUGC/debugc.hpp
        User/Framework/MDO/MDO.cpp
        User/Framework/MDO/MDO.hpp

)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    User/Framework/IMU/algorithm
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # IMU Framework
    User/Framework/IMU/algorithm/AHRS_middleware.c
    User/Framework/IMU/algorithm/QuaternionEKF.c
    User/Framework/IMU/algorithm/user_lib.c
    User/Framework/IMU/controller/controller.c
    User/Framework/IMU/controller/pid.c
    User/Framework/IMU/devices/BMI088Middleware.c
    User/Framework/IMU/devices/BMI088driver.c
    User/Framework/IMU/devices/ist8310driver.c
    User/Framework/IMU/devices/ist8310driver_middleware.c
    User/Framework/IMU/devices/kalman_filter.c
    # MCUDriver
        User/MCUDriver/DELAY/bsp_delay.c
        User/MCUDriver/IMU/bsp_dwt.c
        User/MCUDriver/IMU/bsp_flash.c
        User/MCUDriver/IMU/bsp_imu_pwm.c
        User/MCUDriver/SPI/bsp_spi.c
    # Vision
    User/Framework/Vision/crc.cpp
    # Tasks
    User/Tasks/INS_Task/INSTask.cpp



)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # IMU
    User/Framework/IMU
    User/Framework/IMU/devices
    User/Framework/IMU/algorithm
    User/Framework/IMU/algorithm/Include
    User/Framework/IMU/controller
    # MCUDriver
        User/MCUDriver/DELAY
        User/MCUDriver/IMU
        User/MCUDriver/SPI

        User/MCUDriver/CANio

    User/Framework/GIMBAL
    User/Framework/PID
    User/Framework/WDG
    User/Framework/MOTOR

    User/Framework/USARTIO
    User/Framework/REMOTEC
    User/Framework/REMOTEC/VTMC
    User/Framework/AUTOAIM
    User/Framework/LQR
    User/Framework/DEBUGC
    User/Framework/MDO

    User
    # Vision
    User/Framework/Vision
    User/Framework/AUTOAIM

    # Tasks
    User/Tasks/INS_Task
    User/Tasks/PRINT
    User/Tasks/GIMBAL

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        -DDEBUG
        -DUSE_HAL_DRIVER
        -DSTM32F407xx

        -DARM_MATH_CM4
        -D__FPU_USED=1U
        -D__FPU_PRESENT=1U
        -DARM_MATH_MATRIX_CHECK
        -DARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    AHRS
    # Add user defined libraries
)
# 启用硬浮点 ABI 和 FPU（适用于 STM32F4/F7）
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
target_link_options(${PROJECT_NAME} PRIVATE -u _printf_float)
