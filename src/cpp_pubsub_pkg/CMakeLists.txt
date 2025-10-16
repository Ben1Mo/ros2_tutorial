cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include custom interfaces
find_package(custom_interfaces REQUIRED)

# Add talker executable and dependencies
add_executable(parrot src/talker.cpp)
ament_target_dependencies(parrot rclcpp std_msgs)

# Add listener executable and dependencies
add_executable(spy src/listener.cpp)
ament_target_dependencies(spy rclcpp std_msgs)


# Add custom_talker executable and dependencies
add_executable(custom_parrot src/talker_custom.cpp)
ament_target_dependencies(custom_parrot rclcpp custom_interfaces)

# Add custom_listener executable and dependencies
add_executable(custom_spy src/listener_custom.cpp)
ament_target_dependencies(custom_spy rclcpp custom_interfaces)

install(TARGETS
  parrot
  spy
  custom_parrot
  custom_spy
  DESTINATION lib/${PROJECT_NAME})

ament_package()
