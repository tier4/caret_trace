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

#include <utility>
#include <memory>

#include "caret_trace/data_container.hpp"

TEST(DataContainerTest, EmptyCase) {
  DataContainer container;
  bool finished;
  finished = container.record();
  EXPECT_TRUE(finished);
}

TEST(DataContainerTest, DataConsumeCase) {
  DataContainer container;
  container.add_rcl_node_init(nullptr, nullptr, "node_name", "ns_a");
  container.add_rcl_node_init(nullptr, nullptr, "node_name", "ns_b");
  container.add_rcl_init(nullptr);

  EXPECT_FALSE(container.is_assigned_rcl_init());
  EXPECT_FALSE(container.is_assigned_rcl_node_init());

  container.assign_rcl_init([](const void *) {});
  container.assign_rcl_node_init([](const void *, const void *, const char *, const char *) {});

  EXPECT_TRUE(container.is_assigned_rcl_init());
  EXPECT_TRUE(container.is_assigned_rcl_node_init());

  bool finished;
  finished = container.record();
  EXPECT_FALSE(finished);

  finished = container.record();
  EXPECT_FALSE(finished);

  finished = container.record();
  EXPECT_TRUE(finished);
}
