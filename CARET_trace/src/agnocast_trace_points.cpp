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

#include "caret_trace/clock.hpp"
#include "caret_trace/context.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/tp.h"
#include "caret_trace/trace_node.hpp"
#include "caret_trace/tracing_controller.hpp"

#include <cstdint>
#include <cstdlib>

// clang-format off

extern "C" {

void ros_trace_agnocast_publisher_init(
  const void * publisher_handle,
  const void * node_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * publisher_handle,
    const void * node_handle,
    const char * topic_name,
    const size_t queue_depth,
    int64_t init_time
  ) {
      if (!controller.is_allowed_publisher_handle(publisher_handle)){
        return;
      }
      tracepoint(TRACEPOINT_PROVIDER, agnocast_publisher_init, publisher_handle, node_handle,
      topic_name, queue_depth, init_time);
    };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();
  controller.add_publisher_handle(node_handle, publisher_handle, topic_name);

  if (!data_container.is_assigned_agnocast_publisher_init()) {
    data_container.assign_agnocast_publisher_init(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_publisher_init(
    publisher_handle,
    node_handle,
    topic_name,
    queue_depth,
    now);

  record(publisher_handle, node_handle, topic_name, queue_depth, now);
}

void ros_trace_agnocast_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * callback,
  const void * callback_group,
  const char * symbol,
  const char * topic_name,
  const size_t queue_depth,
  const uint64_t pid_ciid)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static auto record = [](
    const void * subscription_handle,
    const void * node_handle,
    const void * callback,
    const void * callback_group,
    const char * symbol,
    const char * topic_name,
    const size_t queue_depth,
    const uint64_t pid_ciid,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();

    if (!controller.is_allowed_subscription_handle(subscription_handle)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, agnocast_subscription_init, subscription_handle,
      node_handle, callback, callback_group, symbol, topic_name, queue_depth, pid_ciid, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  controller.add_subscription_handle(node_handle, subscription_handle, topic_name);

  if (!data_container.is_assigned_agnocast_subscription_init()) {
    data_container.assign_agnocast_subscription_init(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_subscription_init(
    subscription_handle, node_handle, callback, callback_group, symbol, topic_name,
    queue_depth, pid_ciid, now);

  record(subscription_handle, node_handle, callback, callback_group, symbol,
    topic_name, queue_depth, pid_ciid, now);
}

void ros_trace_agnocast_construct_executor(
  const void * executor_addr,
  const char * executor_type_name)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * executor_addr,
    const char * executor_type_name,
    int64_t init_time
  ) {
    tracepoint(TRACEPOINT_PROVIDER, agnocast_construct_executor, executor_addr,
      executor_type_name, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  if (!data_container.is_assigned_agnocast_construct_executor()) {
    data_container.assign_agnocast_construct_executor(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_construct_executor(executor_addr, executor_type_name, now);

  record(executor_addr, executor_type_name, now);
}

// clang-format on
}
