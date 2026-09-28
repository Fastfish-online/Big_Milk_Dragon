cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MVS_SDK_ROOT "" CACHE PATH "Root of the HIKROBOT MVS SDK")
find_path(MVS_INCLUDE_DIR MvCameraControl.h
  HINTS ${MVS_SDK_ROOT} ENV MVS_SDK_ROOT
  PATH_SUFFIXES include)
find_library(MVS_LIBRARY NAMES MvCameraControl
  HINTS ${MVS_SDK_ROOT} ENV MVS_SDK_ROOT
  PATH_SUFFIXES lib lib/64 lib/arm64)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "HIKROBOT MVS SDK was not found. Set -DMVS_SDK_ROOT=/opt/MVS (or MVS_SDK_ROOT).")
endif()

add_executable(hikrobot_camera_node src/hikrobot_camera_node.cpp)
target_include_directories(hikrobot_camera_node PRIVATE ${MVS_INCLUDE_DIR})
target_link_libraries(hikrobot_camera_node ${MVS_LIBRARY})
ament_target_dependencies(hikrobot_camera_node rclcpp rcl_interfaces sensor_msgs std_msgs)

install(TARGETS hikrobot_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
