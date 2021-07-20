cmake_minimum_required(VERSION 3.5)
project(ros2_hook)

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

include_directories(include)
add_library(hook SHARED
  src/hook.cpp
  src/tp.c)
ament_target_dependencies(hook
  rclcpp
  rcl
  rmw
  rmw_implementation
  rcpputils
  std_msgs
  rmw_fastrtps_shared_cpp
  LTTngUST
  )

target_link_libraries(hook CycloneDDS::ddsc fastrtps)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
