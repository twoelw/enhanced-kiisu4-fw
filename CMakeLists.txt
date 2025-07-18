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
set(CMAKE_PROJECT_NAME KiisuCompanion)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} Core/Src/rw_sh1106.c Core/Inc/main.h Core/Src/rw_sh1106.h Core/Src/rw_hal.c Core/Src/rw_hal.h Core/Inc/board.h Core/Src/rw_i2c_emu.c Core/Src/rw_i2c_emu.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -Wl,-q)

set_source_files_properties(rw_sh1106.h PROPERTIES HEADER_FILE_ONLY TRUE)

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    BYPRODUCTS ${CMAKE_PROJECT_NAME}.bin
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "generate .bin file"
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>.bin"
    VERBATIM
)

add_custom_target(
    flash
    DEPENDS ${CMAKE_PROJECT_NAME}.bin
    COMMAND st-flash --reset write  "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>.bin" 0x08000000
)
