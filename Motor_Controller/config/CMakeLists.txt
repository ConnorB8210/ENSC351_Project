cmake_minimum_required(VERSION 3.18)
project(config C)

add_library(config STATIC
    src/motor_config_runtime.c
)

target_include_directories(config
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}          # if motor_config.h lives directly here
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # if you have config/include/
)