cmake_minimum_required(VERSION 3.18)
project(motor C)

add_library(motor STATIC
    motor_control.c
    hall_commutator.c
    position_estimator.c
    speed_measurement.c
    sensorless_handover.c
    motor_states.c
)

# Include directories (local + shared headers)
target_include_directories(motor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Motor depends on nothing external except pthread + math
find_library(M_LIB m REQUIRED)

target_link_libraries(motor
    PUBLIC
        pthread
        ${M_LIB}
)