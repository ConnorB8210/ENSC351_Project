cmake_minimum_required(VERSION 3.18)
project(config C)

# Header-only library
add_library(config INTERFACE)

target_include_directories(config
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)