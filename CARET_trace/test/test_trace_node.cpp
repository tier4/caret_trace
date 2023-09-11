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

#include "caret_trace/data_container.hpp"
#include "caret_trace/lttng_session.hpp"
#include "caret_trace/trace_node.hpp"
#include "caret_trace/tracing_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test/common.hpp"
#include "test/mock.hpp"

#include "caret_msgs/msg/end.hpp"
#include "caret_msgs/msg/start.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

using ::testing::_;
using ::testing::Return;

/// @private
class CaretTraceNodeTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    lttng_session_ = std::make_shared<LttngSessionMock>();
    data_container_ = std::make_shared<DataContainerMock>();
    set_data_container_return_value(false);
    set_lttng_session_return_value(false);
    node_ = get_init_node();
  }

  void set_status(TRACE_STATUS status)
  {
    node_ = get_init_node();

    // transition to wait status
    auto end_msg = get_end_msg();
    node_->end_callback(std::move(end_msg));
    if (node_->get_status() == status && status == TRACE_STATUS::WAIT) {
      return;
    }

    // transition to prepare status
    auto start_msg = get_start_msg();
    node_->start_callback(std::move(start_msg));
    if (node_->get_status() == status && status == TRACE_STATUS::PREPARE) {
      return;
    }

    // transition to record status
    set_data_container_return_value(true);
    node_->timer_callback();
  }

  void set_data_container_return_value(bool return_value)
  {
    EXPECT_CALL(*data_container_, record(_)).WillRepeatedly(Return(return_value));
  }

  void set_lttng_session_return_value(bool return_value)
  {
    EXPECT_CALL(*lttng_session_, started_session_running()).WillRepeatedly(Return(return_value));
  }

  std::unique_ptr<caret_msgs::msg::Start> get_start_msg()
  {
    return std::make_unique<caret_msgs::msg::Start>();
  }

  std::unique_ptr<caret_msgs::msg::End> get_end_msg()
  {
    return std::make_unique<caret_msgs::msg::End>();
  }

  std::unique_ptr<TraceNode> node_;

private:
  std::shared_ptr<LttngSessionMock> lttng_session_;
  std::shared_ptr<DataContainerMock> data_container_;

  std::unique_ptr<TraceNode> get_init_node()
  {
    if (node_) {
      node_ = nullptr;
    }
    set_lttng_session_return_value(true);
    rclcpp::NodeOptions options;
    return std::make_unique<TraceNode>(
      "node_name", options, lttng_session_, data_container_, rclcpp::Logger::Level::Warn, false,
      false);
  }
};

TEST(CaretTraceTest, TestStartTransition)
{
  init_context();

  auto data_container = std::make_shared<DataContainerMock>();

  {  // session has already started
    auto lttng = std::make_shared<LttngSessionMock>();
    EXPECT_CALL(*lttng, started_session_running()).WillRepeatedly(Return(true));

    rclcpp::NodeOptions options;
    auto node =
      TraceNode("test", options, lttng, data_container, rclcpp::Logger::Level::Warn, false);
    EXPECT_TRUE(node.get_status() == TRACE_STATUS::RECORD);
  }

  {  // session is not started.
    auto lttng = std::make_shared<LttngSessionMock>();
    EXPECT_CALL(*lttng, started_session_running()).WillRepeatedly(Return(false));

    rclcpp::NodeOptions options;
    auto node = TraceNode("node_name", options, lttng, data_container, rclcpp::Logger::Level::Warn);
    EXPECT_TRUE(node.get_status() == TRACE_STATUS::WAIT);
  }
}

TEST_F(CaretTraceNodeTest, TestWaitTransition)
{
  init_context();

  {  // subscribe start message: WAIT -> PREPARE
    set_status(TRACE_STATUS::WAIT);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::WAIT);

    auto start_msg = get_start_msg();
    node_->start_callback(std::move(start_msg));

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);
  }

  {  // subscribe end message: WAIT -> WAIT
    set_status(TRACE_STATUS::WAIT);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::WAIT);

    auto end_msg = get_end_msg();
    node_->end_callback(std::move(end_msg));

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::WAIT);
  }
}

TEST_F(CaretTraceNodeTest, TestPrepareTransition)
{
  init_context();

  {  // subscribe start message: PREPARE -> PREPARE
    set_status(TRACE_STATUS::PREPARE);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);

    auto start_msg = get_start_msg();
    node_->start_callback(std::move(start_msg));

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);
  }

  {  // subscribe end message: PREPARE -> WAIT
    set_status(TRACE_STATUS::PREPARE);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);

    auto end_msg = get_end_msg();
    node_->end_callback(std::move(end_msg));

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::WAIT);
  }

  {
    set_status(TRACE_STATUS::PREPARE);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);

    set_data_container_return_value(false);
    node_->timer_callback();

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);

    set_data_container_return_value(true);
    node_->timer_callback();

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::RECORD);
  }
}

TEST_F(CaretTraceNodeTest, TestMeasureTransition)
{
  init_context();

  {  // subscribe start message: MEASURE -> PREPARE
    set_status(TRACE_STATUS::RECORD);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::RECORD);

    auto start_msg = get_start_msg();
    node_->start_callback(std::move(start_msg));

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::PREPARE);
  }

  {  // subscribe end message: MEASURE -> WAIT
    set_status(TRACE_STATUS::RECORD);

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::RECORD);

    auto end_msg = get_end_msg();
    node_->end_callback(std::move(end_msg));

    EXPECT_TRUE(node_->get_status() == TRACE_STATUS::WAIT);
  }
}

TEST_F(CaretTraceNodeTest, TestWait)
{
  init_context();

  set_status(TRACE_STATUS::WAIT);
  EXPECT_FALSE(node_->is_recording_allowed());
  EXPECT_FALSE(node_->is_timer_running());
}

TEST_F(CaretTraceNodeTest, TestPrepare)
{
  init_context();

  set_status(TRACE_STATUS::PREPARE);
  EXPECT_FALSE(node_->is_recording_allowed());
  EXPECT_TRUE(node_->is_timer_running());
}

TEST_F(CaretTraceNodeTest, TestMeasure)
{
  init_context();

  set_status(TRACE_STATUS::RECORD);
  EXPECT_TRUE(node_->is_recording_allowed());
  EXPECT_FALSE(node_->is_timer_running());
}
