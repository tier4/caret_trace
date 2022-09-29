
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

message(ament_cmake, ${ament_cmake_gmock_LIBRARIES})

ament_add_gmock(test_hashable_keys test_hashable_keys.cpp)
target_link_libraries(test_hashable_keys caret)
