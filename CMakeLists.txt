cmake_minimum_required(VERSION 3.5)
project(caret_trace)

set(CMAKE_CXX_FLAGS "-fcf-protection=none")


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(fastrtps REQUIRED)
find_package(rmw_fastrtps_shared_cpp REQUIRED)
find_package(LTTngUST REQUIRED)

add_library(caret SHARED
  src/ros_trace_points.cpp
  src/hooked_trace_points.cpp
  src/tracing_controller.cpp
  src/singleton.cpp
  src/tp.c)

ament_target_dependencies(caret
  rclcpp
  rcl
  rmw
  rmw_implementation
  rcpputils
  std_msgs
  rmw_fastrtps_shared_cpp
  LTTngUST
  )

target_link_libraries(caret CycloneDDS::ddsc fastrtps)

add_executable(clock_recorder src/clock_recorder.cpp src/tp.c)
ament_target_dependencies(clock_recorder rclcpp LTTngUST)

install(TARGETS
  clock_recorder
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS caret
  EXPORT export_caret_trace
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
