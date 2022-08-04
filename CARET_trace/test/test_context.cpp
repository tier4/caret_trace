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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <utility>
#include <memory>

#include "caret_trace/data_container.hpp"
#include "caret_trace/trace_node.hpp"
#include "caret_trace/context.hpp"

using ::testing::Return;
using ::testing::_;

class CaretTraceNodeModeMock : public CaretTraceNodeInterface
{
public:
  MOCK_CONST_METHOD0(is_recording_allowed, bool());
  MOCK_CONST_METHOD0(is_timer_running, bool());
  MOCK_METHOD0(get_data_container, DataContainerInterface & ());
  MOCK_CONST_METHOD0(get_status, const TRACE_STATUS & ());
};

TEST(ContextTest, TestGetDataContainer) {
  auto data_container = std::make_shared<DataContainer>();
  Context context(data_container);

  auto & data_container_ = context.get_data_container();
  EXPECT_EQ(&data_container_, data_container.get());

  auto data_container_ptr_ = context.get_data_container_ptr();
  EXPECT_EQ(data_container_ptr_.get(), data_container.get());
}

TEST(ContextTest, TestNodeAssign) {
  auto data_container = std::make_shared<DataContainer>();

  Context context(data_container);
  EXPECT_FALSE(context.is_node_assigned());
  EXPECT_DEATH(context.get_node(), "");

  auto node = std::make_shared<CaretTraceNodeModeMock>();
  context.assign_node(node);
  EXPECT_TRUE(context.is_node_assigned());
  auto & node_ = context.get_node();
  EXPECT_EQ(&node_, node.get());
}

TEST(ContextTest, TestIsRecordingEnabled) {
  auto data_container = std::make_shared<DataContainer>();

  Context context(data_container);
  EXPECT_FALSE(context.is_recording_enabled());

  auto node = std::make_shared<CaretTraceNodeModeMock>();

  context.assign_node(node);
  EXPECT_CALL(*node, is_recording_allowed()).WillRepeatedly(Return(false));
  EXPECT_FALSE(context.is_recording_enabled());

  EXPECT_CALL(*node, is_recording_allowed()).WillRepeatedly(Return(true));
  EXPECT_TRUE(context.is_recording_enabled());
}
