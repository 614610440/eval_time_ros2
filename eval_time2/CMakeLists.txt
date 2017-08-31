cmake_minimum_required(VERSION 3.5)

project(eval)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(app
  eval_time2.cpp
)
ament_target_dependencies(app
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV")

install(TARGETS
  app
  DESTINATION bin/${PROJECT_NAME})


ament_package()
