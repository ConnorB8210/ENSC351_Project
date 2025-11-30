cmake_minimum_required(VERSION 3.18)
project(algorithms C)

add_library(algorithms STATIC
    pi_controller.c
    filters.c
    bemf_sector.c
)

target_include_directories(algorithms
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

find_library(M_LIB m REQUIRED)

target_link_libraries(algorithms
    PUBLIC
        ${M_LIB}
        pthread
)