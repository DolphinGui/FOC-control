# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.25)
if(DEFINED ENV{PICO_SDK_PATH})
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
# set(PICO_CLIB picolibc)
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m33 -mfloat-abi=hard")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(foc-driver LANGUAGES CXX C ASM)
set(CMAKE_EXECUTABLE_SUFFIX .elf)
# Always check that the $ENV{LIBHAL_PLATFORM_LIBRARY} & $ENV{LIBHAL_PLATFORM}
# environment variables are set by the profile.
if("$ENV{LIBHAL_PLATFORM_LIBRARY}" STREQUAL "")
    message(FATAL_ERROR
        "Build environment variable LIBHAL_PLATFORM_LIBRARY is required for " "this project.")
endif()
if("$ENV{LIBHAL_PLATFORM}" STREQUAL "")
    message(FATAL_ERROR
        "Build environment variable LIBHAL_PLATFORM is required for "
        "this project.")
endif()

find_package(libhal-$ENV{LIBHAL_PLATFORM_LIBRARY} REQUIRED CONFIG)
find_package(libhal-util REQUIRED CONFIG)
find_package(mp-units REQUIRED)

add_executable(foc_driver
    src/main.cpp
    src/peripherals.cpp
    src/example_motor.cpp
    platforms/$ENV{LIBHAL_PLATFORM}.cpp
)

add_executable(adc_demo
    src/adc_main.cpp
)

add_executable(dash_demo
    src/dash_main.cpp
)

add_executable(encoder_demo
    src/encoder_main.cpp
)

if(DEFINED ENV{PICO_SDK_PATH})
    pico_sdk_init()
endif()

foreach(TARGET foc_driver;adc_demo;dash_demo;encoder_demo)
if(DEFINED ENV{PICO_SDK_PATH})
    pico_enable_stdio_usb(${TARGET} 1)
    pico_enable_stdio_uart(${TARGET} 0)
    pico_add_extra_outputs(${TARGET})
    pico_set_binary_type(${TARGET} copy_to_ram)
    target_link_libraries(${TARGET} PRIVATE
        pico_stdlib)
endif()
target_compile_options(${TARGET} PRIVATE -g -Wall -Wextra -flto=auto)
target_link_options(${TARGET} PRIVATE -flto=auto)
target_include_directories(${TARGET} PUBLIC include src)
target_link_libraries(${TARGET} PRIVATE
    libhal::$ENV{LIBHAL_PLATFORM_LIBRARY}
    libhal::util)
endforeach()

target_link_libraries(foc_driver PRIVATE
    mp-units::mp-units)

target_link_libraries(adc_demo PRIVATE
    hardware_adc
    hardware_dma
    hardware_pwm)

target_link_libraries(dash_demo PRIVATE
    hardware_adc
    hardware_dma)


target_link_libraries(encoder_demo PRIVATE
    hardware_i2c
    mp-units::mp-units)