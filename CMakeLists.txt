# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SourceCode C CXX ASM)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(SourceCode src/Main.cpp src/functions.cpp src/Flood.cpp src/API_Actual.cpp src/drivers/vl6180x.cpp src/drivers/driver_mpu6500.c src/drivers/driver_mpu6500_fifo.c src/drivers/driver_mpu6500_interface.c src/drivers/vl53l1x.cpp)
# pico_generate_pio_header(SourceCode  ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder_substep.pio)

target_compile_definitions(SourceCode PRIVATE EIGEN_NO_DEBUG=1 EIGEN_NO_EXCEPTIONS)
target_include_directories(SourceCode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/eigen)

pico_set_program_name(SourceCode "SourceCode")
pico_set_program_version(SourceCode "1.0")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(SourceCode 0)
pico_enable_stdio_usb(SourceCode 1)

# Add the standard library to the build
target_link_libraries(SourceCode
        pico_stdlib
        pico_stdio_usb
        hardware_i2c
        hardware_pwm
        # hardware_pio
        )

# Add the standard include files to the build
target_include_directories(SourceCode PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(SourceCode)

