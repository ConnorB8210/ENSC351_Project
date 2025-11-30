cmake_minimum_required(VERSION 3.18)
project(Motor_Controller C)

# Use C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# --- Subdirectories ---
# Each of these has its own CMakeLists.txt:
#   config/      -> config lib (motor_config_runtime.c, headers)
#   hal/         -> hal lib (adc, bemf, gpio, pwm, etc.)
#   algorithms/  -> algorithms lib (bemf_sector.c, etc.)
#   motor/       -> motor lib (motor_control, speed_measurement, etc.)
#   app/         -> defines the Motor_Controller executable
add_subdirectory(config)
add_subdirectory(hal)
add_subdirectory(algorithms)
add_subdirectory(motor)
add_subdirectory(app)