find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(LTTngUST REQUIRED)

include_directories(include)

ament_add_gmock(test_hashable_keys test_hashable_keys.cpp)
target_link_libraries(test_hashable_keys caret ${LTTNGUST_LIBRARIES})

ament_add_gmock(test_context test_context.cpp)
target_link_libraries(test_context caret ${LTTNGUST_LIBRARIES})

ament_add_gmock(test_scenario test_scenario.cpp common.cpp)
target_link_libraries(test_scenario caret ${LTTNGUST_LIBRARIES})

ament_add_gmock(test_trace_node test_trace_node.cpp common.cpp)
target_link_libraries(test_trace_node caret ${LTTNGUST_LIBRARIES})

ament_add_gmock(test_data_container test_data_container.cpp common.cpp)
target_link_libraries(test_data_container caret ${LTTNGUST_LIBRARIES})

ament_add_gmock(test_data_recorder test_data_recorder.cpp common.cpp)
target_link_libraries(test_data_recorder caret ${LTTNGUST_LIBRARIES})

ament_add_gmock(test_recordable_data test_recordable_data.cpp common.cpp)
target_link_libraries(test_recordable_data caret ${LTTNGUST_LIBRARIES})
