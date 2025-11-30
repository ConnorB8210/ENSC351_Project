cmake_minimum_required(VERSION 3.18)
project(config C)

# Runtime config implementation lives here
add_library(config STATIC
    motor_config_runtime.c
)

target_include_directories(config
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}   # motor_config.h, motor_config_runtime.h
)