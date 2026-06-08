cmake_minimum_required(VERSION 3.8)
project(delivery_mission)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
 
include_directories(include)
 
# ── Step 1: Pre-flight check ──────────────────────────────────────────────────
add_executable(preflight_check src/preflight_check.cpp)
ament_target_dependencies(preflight_check rclcpp px4_msgs)
 
install(TARGETS
  preflight_check
  DESTINATION lib/${PROJECT_NAME}
)
 
install(DIRECTORY include/
  DESTINATION include/
)
 
ament_package()
 
