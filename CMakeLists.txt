# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.20)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Project definition
project(DogRobot)

# Enable CMake support for ASM and C/CPP languages
enable_language(C ASM)

set(BUILD_UNIT_0_NAME DogRobot)

# Create an executable object type
add_executable(${BUILD_UNIT_0_NAME})

# STM32CubeIDE for Visual Studio Code managed CMake file
include(cmake/vscode_generated.cmake)

# Compiler options
target_compile_options(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined compiler options
)

# Linker options
target_link_options(${BUILD_UNIT_0_NAME} PRIVATE 
  # User defined linker options
)

# Add sources to executable/library
target_sources(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined sources
  Core/Src/tim.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
  User/bus_servo.c
  User/dog_servo_config.c
  User/dog_servo.c
  User/dog_gait.c
  User/image_command.c
  User/dog_task.c
  User/throw_servo.c
)

# Add include directories
target_include_directories(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined include directories
  User
)

target_link_libraries(${BUILD_UNIT_0_NAME} PRIVATE
  m
)

add_executable(DogRobotTrackBlueBoostTest)

target_compile_options(DogRobotTrackBlueBoostTest PRIVATE
    $<$<COMPILE_LANGUAGE:C>: ${CUBE_CMAKE_C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${CUBE_CMAKE_CXX_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${CUBE_CMAKE_ASM_FLAGS}>
)

target_link_options(DogRobotTrackBlueBoostTest PRIVATE ${CUBE_CMAKE_EXE_LINKER_FLAGS})

target_sources(DogRobotTrackBlueBoostTest PRIVATE
    Core/Src/gpio.c
    Core/Src/main_track_blue_boost_test.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/stm32f1xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f1xx.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Startup/startup_stm32f103c8tx.s
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    User/bus_servo.c
    User/dog_servo_config.c
    User/dog_servo.c
    User/dog_gait.c
    User/image_command.c
    User/track_blue_boost_test_program.c
)

target_include_directories(DogRobotTrackBlueBoostTest PRIVATE
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
    User
)

target_link_libraries(DogRobotTrackBlueBoostTest PRIVATE
  m
)

set_target_properties(DogRobotTrackBlueBoostTest PROPERTIES LINK_DEPENDS "STM32F103C8TX_FLASH.ld")
