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
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MQTT-Testing C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

# Add executable. Default name is the project name, version 0.1

add_executable(MQTT-Testing MQTT-Testing.c )

pico_set_program_name(MQTT-Testing "MQTT-Testing")
pico_set_program_version(MQTT-Testing "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(MQTT-Testing 1)
pico_enable_stdio_usb(MQTT-Testing 1)

# Add the standard library to the build
target_link_libraries(MQTT-Testing
    pico_stdlib
    pico_lwip_mbedtls
    pico_mbedtls
    pico_lwip_mqtt )

# Add the standard include files to the build
target_include_directories(MQTT-Testing PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(MQTT-Testing 
        hardware_pio
        pico_cyw43_arch_lwip_threadsafe_background
        )

pico_add_extra_outputs(MQTT-Testing)

