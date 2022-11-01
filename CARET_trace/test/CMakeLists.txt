find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(LTTngUST REQUIRED)

message(ament_cmake, ${ament_cmake_gmock_LIBRARIES})

ament_add_gmock(test_hashable_keys test_hashable_keys.cpp)
target_link_libraries(test_hashable_keys caret)

ament_add_gmock(test_context test_context.cpp)
target_link_libraries(test_context caret)

ament_add_gmock(test_data_container test_data_container.cpp common.cpp)
target_link_libraries(test_data_container caret)

ament_add_gmock(test_data_recorder test_data_recorder.cpp common.cpp)
target_link_libraries(test_data_recorder caret)

ament_add_gmock(test_recordable_data test_recordable_data.cpp common.cpp)
target_link_libraries(test_recordable_data caret)
