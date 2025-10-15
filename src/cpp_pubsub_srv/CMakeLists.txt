cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(adv_parrot src/talker_adv.cpp)
ament_target_dependencies(adv_parrot rclcpp custom_interfaces)

add_executable(adv_spy src/listener_adv.cpp)
ament_target_dependencies(adv_spy rclcpp custom_interfaces)

install(TARGETS
    adv_parrot
    adv_spy
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/$(PROJECT_NAME)
)

ament_package()
