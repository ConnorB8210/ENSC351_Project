cmake_minimum_required(VERSION 3.18)
project(Motor_Controller C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Each of these folders has its own CMakeLists.txt
add_subdirectory(config)
add_subdirectory(hal)
add_subdirectory(algorithms)
add_subdirectory(motor)
add_subdirectory(app)