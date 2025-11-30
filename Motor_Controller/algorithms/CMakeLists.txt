cmake_minimum_required(VERSION 3.18)
project(algorithms C)

add_library(algorithms STATIC
    src/bemf_sector.c
    src/filters.c
    src/pi_controller.c
)
target_include_directories(algorithms
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/config          # e.g. Motor_Controller/config/motor_config.h
        ${CMAKE_SOURCE_DIR}/config/include  # if you keep headers in config/include/
        ${CMAKE_SOURCE_DIR}/hal/include     # for bemf.h
)

find_library(M_LIB m REQUIRED)

target_link_libraries(algorithms
    PUBLIC
        ${M_LIB}
        pthread
)