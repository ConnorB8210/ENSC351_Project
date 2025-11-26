#tell cmake to use the aarch64 toolchain

#run the build command like so
# cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=./aarch64-toolchain.cmake

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
