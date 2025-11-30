cmake_minimum_required(VERSION 3.18)
project(hal C)

add_library(hal STATIC
    src/adc.c
    src/bemf.c
    src/gpio.c
    src/drv8302.c
    src/timer.c
    src/hall.c
    src/pwm.c
    src/pwm_motor.c
)

# --- libgpiod via pkg-config ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# --- Math library (for sqrt, pow, etc.) ---
find_library(M_LIB m REQUIRED)

# --- Include directories ---
target_include_directories(hal
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${LIBGPIOD_INCLUDE_DIRS}
)

# --- Link against libgpiod + pthread + libm ---
target_link_libraries(hal
    PUBLIC
        ${LIBGPIOD_LIBRARIES}
        pthread
        ${M_LIB}
)

# --- Diagnostics ---
message(STATUS "Using libgpiod version: ${LIBGPIOD_VERSION}")
message(STATUS "libgpiod include dirs: ${LIBGPIOD_INCLUDE_DIRS}")
message(STATUS "libgpiod libraries: ${LIBGPIOD_LIBRARIES}")
message(STATUS "Math library: ${M_LIB}")