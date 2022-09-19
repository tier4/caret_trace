
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

message(ament_cmake, ${ament_cmake_gmock_LIBRARIES})

include_directories(include)

ament_add_gmock(test_trace_node test_trace_node.cpp common.cpp)
target_link_libraries(test_trace_node caret_test_core)

ament_add_gmock(test_data_container test_data_container.cpp common.cpp)
target_link_libraries(test_data_container caret_test_core)

ament_add_gmock(test_data_recorder test_data_recorder.cpp common.cpp)
target_link_libraries(test_data_recorder caret_test_core)

ament_add_gmock(test_recordable_data test_recordable_data.cpp common.cpp)
target_link_libraries(test_recordable_data caret_test_core)

ament_add_gmock(test_lttng_session test_lttng_session.cpp common.cpp)
target_link_libraries(test_lttng_session caret_test_core)

ament_add_gmock(test_context test_context.cpp common.cpp)
target_link_libraries(test_context caret_test_core)

ament_add_gmock(test_hashable_keys test_hashable_keys.cpp common.cpp)
target_link_libraries(test_hashable_keys caret_test_core)

ament_add_gmock(test_senario test_scenario.cpp common.cpp)
target_link_libraries(test_senario caret_test_core)
