cmake_minimum_required(VERSION 3.18)
project(motor C)

add_library(motor STATIC
    motor_control.c
    position_estimator.c
    speed_measurement.c
    hall_commutator.c
    sensorless_handover.c
)

target_include_directories(motor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/config          # motor_config.h
        ${CMAKE_SOURCE_DIR}/hal/include     # PwmMotor_t, HallHandle_t, BemfHandle_t
        ${CMAKE_SOURCE_DIR}/algorithms      # bemf_sector.h etc.
)

target_link_libraries(motor
    PUBLIC
        algorithms
        config
        pthread
)