cmake_minimum_required(VERSION 3.18)
project(Motor_Controller C)

set(CMAKE_C_STANDARD 11)

# --- Subdirectories ---
add_subdirectory(config)
add_subdirectory(hal)
add_subdirectory(algorithms)
add_subdirectory(motor)

# --- Executable ---
add_executable(Motor_Controller
    app/src/main.c
    app/src/status_display.c
    app/src/udp_server.c
)

# --- Include dirs for the app ---
target_include_directories(Motor_Controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/app/include   # app headers
    ${CMAKE_SOURCE_DIR}/hal/include          # HAL headers
    ${CMAKE_SOURCE_DIR}/config               # motor_config.h, runtime .h
    ${CMAKE_SOURCE_DIR}/motor/include        # motor_control.h, etc (if you have this)
    ${CMAKE_SOURCE_DIR}/algorithms/include   # bemf_sector.h, etc (if you have this)
)

# --- Link against libs ---
target_link_libraries(Motor_Controller PRIVATE
    hal
    motor
    algorithms
    config
    pthread
)

# --- Compiler warnings / sanitizers ---
target_compile_options(Motor_Controller PRIVATE
    -Wall -Werror -Wpedantic -Wextra -fdiagnostics-color -fsanitize=address -pthread
)
target_link_options(Motor_Controller PRIVATE
    -fsanitize=address -pthread
)

# --- Copy executable to NFS ---
add_custom_command(TARGET Motor_Controller POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:Motor_Controller>
        /home/connor/ENSC351/public/
    COMMENT "Copying Motor_Controller executable to NFS directory"
)