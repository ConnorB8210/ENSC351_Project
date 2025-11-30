cmake_minimum_required(VERSION 3.18)
project(algorithms C)

add_library(algorithms STATIC
    bemf_sector.c
    pi_controller.c
    filters.c
)

target_include_directories(algorithms
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/config
)

find_library(M_LIB m REQUIRED)

target_link_libraries(algorithms
    PUBLIC
        ${M_LIB}
        pthread
)