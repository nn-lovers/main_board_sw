# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_policy(SET CMP0076 NEW)

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
set(PICO_BOARD wiznet_w5100s_evb_pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(w5100_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

set(WIZNET_CHIP W5100S)
add_definitions(-D_WIZCHIP_=W5100S)
add_definitions(-DDEVICE_BOARD_NAME=W5100S_EVB_PICO)

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

if(NOT DEFINED FREERTOS_DIR)
    set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/freertos)
    set(FREERTOS_KERNEL_PATH ${FREERTOS_DIR})
    message(STATUS "FREERTOS_DIR = ${FREERTOS_DIR}")
endif()
set(FREERTOS_CONFIG_FILE_DIRECTORY ${PORT_DIR}/freertos)
set(FREERTOS_PORT GCC_RP2040)

if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED INC_DIR)
    set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
    message(STATUS "INC_DIR = ${INC_DIR}")
endif()

add_subdirectory(${PORT_DIR})
add_subdirectory(lib)
include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(src)