cmake_minimum_required(VERSION 3.18)

project(Motor_Controller_app C)

# --- Executable --- 
add_executable(Motor_Controller
    src/main.c
    src/status_display.c
    src/udp_server.c
)

# --- Include directories for app and HAL ---
target_include_directories(Motor_Controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include      # app headers
    ${CMAKE_SOURCE_DIR}/hal/include          # HAL headers
    ${LIBGPIOD_INCLUDE_DIRS}                 # libgpiod from sysroot
)

# --- Link against HAL library and libgpiod ---
target_link_libraries(Motor_Controller PRIVATE
    hal
    ${LIBGPIOD_LIBRARIES}
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