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
#include <string>
#include <vector>

#include "caret_trace/data_container.hpp"

TEST(DataContainerTest, EmptyCase) {
  DataContainer container(
    nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr
  );
  bool finished;
  finished = container.record();
  EXPECT_TRUE(finished);
}

TEST(DataContainerTest, DataConsumeCase) {
  DataContainer container;
  container.store_rcl_node_init(nullptr, nullptr, "node_name", "ns_a");
  container.store_rcl_node_init(nullptr, nullptr, "node_name", "ns_b");
  container.store_rcl_init(nullptr);

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

TEST(DataContainerTest, TracePoints) {
  DataContainer container;
  auto trarce_points = container.trace_points();

  std::vector<std::string> expect({
    "add_callback_group",
    "add_callback_group_static_executor",
    "callback_group_add_client",
    "callback_group_add_service",
    "callback_group_add_subscription",
    "callback_group_add_timer",
    "construct_executor",
    "construct_static_executor",
    "rcl_client_init",
    "rcl_init",
    "rcl_node_init",
    "rcl_publisher_init",
    "rcl_service_init",
    "rcl_subscription_init",
    "rcl_timer_init",
    "rclcpp_callback_register",
    "rclcpp_service_callback_added",
    "rclcpp_subscription_callback_added",
    "rclcpp_subscription_init",
    "rclcpp_timer_callback_added",
    "rclcpp_timer_link_node",
    "rmw_implementation"
  });

  EXPECT_EQ(trarce_points, expect);
}
