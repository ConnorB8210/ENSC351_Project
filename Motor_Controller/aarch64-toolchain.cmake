# -----------------------------
# aarch64-toolchain.cmake
# -----------------------------
# Toolchain file for cross-compiling to BeagleBone (aarch64)
# Using sysroot at /home/connor/bbb-sysroot
# Host compiler: aarch64-linux-gnu-gcc
# -----------------------------

# Name of target system
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Specify the cross-compilers on the host
set(CMAKE_C_COMPILER   /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Sysroot path (where the BeagleBone root filesystem is)
set(CMAKE_SYSROOT /home/connor/bbb-sysroot)

# Search for libraries and headers inside the sysroot
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Adjust search behavior:
#  - Search first in sysroot for libraries and includes
#  - Do not search host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Optional: tell pkg-config to look in sysroot
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
set(ENV{PKG_CONFIG_PATH} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig)

set(CMAKE_MAKE_PROGRAM /usr/bin/make CACHE FILEPATH "Make program")