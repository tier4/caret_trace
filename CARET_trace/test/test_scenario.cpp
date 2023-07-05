// Copyright 2021 Research Institute of Systems Planning, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "caret_trace/context.hpp"
#include "caret_trace/data_container.hpp"
#include "caret_trace/lttng_session.hpp"
#include "caret_trace/trace_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test/common.hpp"
#include "test/mock.hpp"

#include "caret_msgs/msg/end.hpp"
#include "caret_msgs/msg/start.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <utility>

using ::testing::_;
using ::testing::MockFunction;
using ::testing::Return;

using rclcpp::Logger;

void add_data(DataContainer & container, int loop)
{
  char * ptr = nullptr;
  for (auto i = 0; i < loop; i++) {
    ptr++;
    container.store_rcl_init(ptr, 0);
  }
}

void record_data(DataContainer & container, int loop)
{
  container.record(loop);
}

TEST(ScenarioTest, TestSingleThread)
{
  auto keys = std::make_shared<DataContainer::RclInit::KeysT>("rcl_init");

  DataContainer container(
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, keys, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr);

  MockFunction<DataContainer::RclInit::FuncT> rcl_init_mock;
  container.assign_rcl_init(rcl_init_mock.AsStdFunction());

  EXPECT_TRUE(container.is_assigned_rcl_init());

  auto loop = 1000;

  char * ptr = nullptr;
  for (auto i = 0; i < loop; i++) {
    ptr++;
    EXPECT_CALL(rcl_init_mock, Call(ptr, 0)).Times(1);
  }

  add_data(container, loop);
  container.record(loop);
}

TEST(ScenarioTest, TestMultiThread)
{
  auto keys = std::make_shared<DataContainer::RclInit::KeysT>("rcl_init");

  DataContainer container(
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, keys, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr);

  MockFunction<DataContainer::RclInit::FuncT> rcl_init_mock;
  container.assign_rcl_init(rcl_init_mock.AsStdFunction());

  EXPECT_TRUE(container.is_assigned_rcl_init());

  auto loop = 1000;

  char * ptr = nullptr;
  for (auto i = 0; i < loop; i++) {
    ptr++;
    EXPECT_CALL(rcl_init_mock, Call(ptr, 0)).WillRepeatedly(Return());
  }

  // NOTE: Ensure recording data. Avoid container.record() before add_data() is called.
  add_data(container, 1);

  std::thread t1([&container, loop]() { add_data(container, loop); });
  std::thread t2([&container, loop]() { container.record(loop); });

  t1.join();
  t2.join();
  auto finished = container.record(loop);

  EXPECT_TRUE(finished);
  EXPECT_EQ(keys->pending_size(), (size_t)0);
  EXPECT_EQ(keys->size(), (size_t)loop);
}

TEST(ScenarioTest, RepetitiveRecordingCase)
{
  init_context();

  auto container = std::make_shared<DataContainer>();

  void * addr_0 = reinterpret_cast<void *>(0);
  void * addr_1 = reinterpret_cast<void *>(1);
  container->store_rcl_init(addr_0, 0);
  container->store_rcl_init(addr_1, 0);

  MockFunction<DataContainer::RclInit::FuncT> func;
  container->assign_rcl_init(func.AsStdFunction());

  auto lttng = std::make_shared<LttngSessionMock>();
  EXPECT_CALL(*lttng, started_session_running()).WillRepeatedly(Return(false));

  rclcpp::NodeOptions options;
  auto node = TraceNode("test", options, lttng, container, Logger::Level::Warn, false);
  EXPECT_EQ(node.get_status(), TRACE_STATUS::WAIT);

  {  // fist recording
    EXPECT_CALL(func, Call(addr_0, 0)).Times(1);
    EXPECT_CALL(func, Call(addr_1, 0)).Times(1);

    auto start_msg = std::make_unique<caret_msgs::msg::Start>();
    start_msg->recording_frequency = 1;  // record nullptr only.
    node.start_callback(std::move(start_msg));
    EXPECT_EQ(node.get_status(), TRACE_STATUS::PREPARE);

    node.timer_callback();  // record rest data.
    EXPECT_EQ(node.get_status(), TRACE_STATUS::RECORD);

    auto end_msg = std::make_unique<caret_msgs::msg::End>();
    node.end_callback(std::move(end_msg));
    EXPECT_EQ(node.get_status(), TRACE_STATUS::WAIT);
  }

  {  // second recording
    EXPECT_CALL(func, Call(addr_0, 0)).Times(1);
    EXPECT_CALL(func, Call(addr_1, 0)).Times(1);

    auto start_msg = std::make_unique<caret_msgs::msg::Start>();
    start_msg->recording_frequency = 1;  // record nullptr only.
    node.start_callback(std::move(start_msg));
    EXPECT_EQ(node.get_status(), TRACE_STATUS::PREPARE);

    node.timer_callback();  // record rest data.
    EXPECT_EQ(node.get_status(), TRACE_STATUS::RECORD);

    auto end_msg = std::make_unique<caret_msgs::msg::End>();
    node.end_callback(std::move(end_msg));
    EXPECT_EQ(node.get_status(), TRACE_STATUS::WAIT);
  }
}

TEST(ScenarioTest, IsRecordingAllowed)
{
  init_context();

  auto container = std::make_shared<DataContainer>();
  Context context;

  void * addr_0 = reinterpret_cast<void *>(0);
  void * addr_1 = reinterpret_cast<void *>(1);
  container->store_rcl_init(addr_0, 0);
  container->store_rcl_init(addr_1, 0);

  auto lttng = std::make_shared<LttngSessionMock>();
  EXPECT_CALL(*lttng, started_session_running()).WillRepeatedly(Return(false));

  rclcpp::NodeOptions options;
  auto node =
    std::make_shared<TraceNode>("test", options, lttng, container, Logger::Level::Warn, false);
  context.assign_node(node);

  EXPECT_EQ(node->get_status(), TRACE_STATUS::WAIT);
  EXPECT_FALSE(context.is_recording_allowed());

  auto start_msg = std::make_unique<caret_msgs::msg::Start>();
  start_msg->recording_frequency = 1;
  node->start_callback(std::move(start_msg));
  EXPECT_EQ(node->get_status(), TRACE_STATUS::PREPARE);
  EXPECT_FALSE(context.is_recording_allowed());

  node->timer_callback();
  EXPECT_EQ(node->get_status(), TRACE_STATUS::RECORD);
  EXPECT_TRUE(context.is_recording_allowed());
}

TEST(ScenarioTest, Record)
{
  init_context();

  auto keys = std::make_shared<DataContainer::RclInit::KeysT>("rcl_init");

  auto container = std::make_shared<DataContainer>(
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, keys, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr);
  Context context;
  bool pending;

  void * addr_1 = reinterpret_cast<void *>(1001);
  void * addr_2 = reinterpret_cast<void *>(1002);
  void * addr_3 = reinterpret_cast<void *>(1003);
  void * addr_4 = reinterpret_cast<void *>(1004);

  // assertion case: rcl_init function is not assigned
  // container->store_rcl_init(nullptr)

  MockFunction<DataContainer::RclInit::FuncT> func;
  EXPECT_CALL(func, Call(addr_1, 0)).Times(1);
  EXPECT_CALL(func, Call(addr_2, 0)).Times(1);
  EXPECT_CALL(func, Call(addr_3, 0)).Times(0);
  EXPECT_CALL(func, Call(addr_4, 0)).Times(0);

  container->assign_rcl_init(func.AsStdFunction());

  // case: node is not initialized
  pending = container->store_rcl_init(addr_1, 0);
  EXPECT_EQ(keys->size(), (size_t)1);
  EXPECT_FALSE(pending);

  auto lttng = std::make_shared<LttngSessionMock>();
  EXPECT_CALL(*lttng, started_session_running()).WillRepeatedly(Return(false));

  rclcpp::NodeOptions options;
  auto node =
    std::make_shared<TraceNode>("test", options, lttng, container, Logger::Level::Warn, false);
  context.assign_node(node);

  // case: WAIT status
  EXPECT_EQ(node->get_status(), TRACE_STATUS::WAIT);
  pending = container->store_rcl_init(addr_2, 0);
  EXPECT_EQ(keys->size(), (size_t)2);
  EXPECT_FALSE(pending);

  // case: PREPARE status
  auto start_msg = std::make_unique<caret_msgs::msg::Start>();
  start_msg->recording_frequency = 1;
  node->start_callback(std::move(start_msg));

  EXPECT_EQ(node->get_status(), TRACE_STATUS::PREPARE);
  pending = container->store_rcl_init(addr_3, 0);
  EXPECT_EQ(keys->size(), (size_t)2);
  EXPECT_EQ(keys->pending_size(), (size_t)1);
  EXPECT_TRUE(pending);  // NOTE: Data in PENDING should be recorded in real time.

  // case: RECORD status
  node->timer_callback();

  EXPECT_EQ(node->get_status(), TRACE_STATUS::RECORD);
  EXPECT_EQ(keys->size(), (size_t)3);

  container->store_rcl_init(addr_4, 0);
  EXPECT_EQ(keys->size(), (size_t)4);
}
