# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(ads1x15 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# turn on all compiler warnings 
add_compile_options(-Wall)

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME}
  examples/single_ended/main.cpp
  #examples/continuous_interrupt_differential/main.cpp
  #examples/single_shot_differential/main.cpp
  #examples/differential/main.cpp
  #examples/comparator/main.cpp

)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

add_library(pico_ads1x15 INTERFACE)

target_sources(pico_ads1x15 INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/ads1x15/ads1x15.cpp
)

target_include_directories(pico_ads1x15 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

# Pull in pico libraries that we need
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_i2c pico_ads1x15 )


# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

