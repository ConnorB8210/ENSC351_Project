cmake_minimum_required(VERSION 3.18)

project(Motor_Controller C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# --- Toolchain auto-detect ---
if(NOT CMAKE_TOOLCHAIN_FILE AND NOT CMAKE_CROSSCOMPILING)
    set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/aarch64-toolchain.cmake")
    if(EXISTS ${TOOLCHAIN_FILE})
        message(STATUS "Auto-detecting toolchain file: ${TOOLCHAIN_FILE}")
        set(CMAKE_TOOLCHAIN_FILE ${TOOLCHAIN_FILE} CACHE FILEPATH "Toolchain file" FORCE)
    else()
        message(STATUS "No toolchain file found, using default compiler")
    endif()
endif()

# --- Sysroot support ---
if(CMAKE_CROSSCOMPILING AND DEFINED CMAKE_SYSROOT)
    message(STATUS "Cross-compiling with sysroot: ${CMAKE_SYSROOT}")
    set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
    set(ENV{PKG_CONFIG_PATH} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig)
    set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
endif()

# --- Ensure pkg-config is available ---
find_package(PkgConfig REQUIRED)

# --- Add HAL and App subdirectories ---
add_subdirectory(hal)
add_subdirectory(app)

# --- Diagnostics ---
message(STATUS "System name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C compiler: ${CMAKE_C_COMPILER}")