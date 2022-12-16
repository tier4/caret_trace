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
#include "caret_trace/trace_node.hpp"
#include "caret_trace/tracing_controller.hpp"
#include "test/mock.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

using ::testing::_;
using ::testing::Return;

TEST(ContextTest, TestGetController)
{
  auto data_container = std::make_shared<DataContainer>();
  auto controller = std::make_shared<TracingController>();
  Context context(data_container, controller);

  auto & controller_ = context.get_controller();
  EXPECT_EQ(&controller_, controller.get());
}

TEST(ContextTest, TestGetDataContainer)
{
  auto data_container = std::make_shared<DataContainer>();
  auto controller = std::make_shared<TracingController>();
  Context context(data_container, controller);

  auto & data_container_ = context.get_data_container();
  EXPECT_EQ(&data_container_, data_container.get());

  auto data_container_ptr_ = context.get_data_container_ptr();
  EXPECT_EQ(data_container_ptr_.get(), data_container.get());
}

TEST(ContextTest, TestNodeAssign)
{
  auto data_container = std::make_shared<DataContainer>();
  auto controller = std::make_shared<TracingController>();

  Context context(data_container, controller);
  EXPECT_FALSE(context.is_node_assigned());

  auto node = std::make_shared<CaretTraceNodeModeMock>();
  context.assign_node(node);
  EXPECT_TRUE(context.is_node_assigned());
  auto & node_ = context.get_node();
  EXPECT_EQ(&node_, node.get());
}
