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

#define DEFINE_ORIG_FUNC(TP_NAME) TP_NAME = dlsym(RTLD_NEXT, #TP_NAME)

namespace ORIG_FUNC
{
static void * DEFINE_ORIG_FUNC(ros_trace_agnocast_publish);
static void * DEFINE_ORIG_FUNC(ros_trace_agnocast_create_callable);
static void * DEFINE_ORIG_FUNC(ros_trace_agnocast_callable_start);
static void * DEFINE_ORIG_FUNC(ros_trace_agnocast_callable_end);
static void * DEFINE_ORIG_FUNC(ros_trace_agnocast_take);
static void * DEFINE_ORIG_FUNC(ros_trace_agnocast_create_timer_callable);
}  // namespace ORIG_FUNC

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
  const uint64_t pid_callback_info_id)
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
    const uint64_t pid_callback_info_id,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();

    if (!controller.is_allowed_subscription_handle(subscription_handle)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, agnocast_subscription_init, subscription_handle,
      node_handle, callback, callback_group, symbol, topic_name, queue_depth,
      pid_callback_info_id, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  controller.add_subscription_handle(node_handle, subscription_handle, topic_name);
  controller.add_pid_callback_info_id(
    pid_callback_info_id, node_handle, topic_name);

  if (!data_container.is_assigned_agnocast_subscription_init()) {
    data_container.assign_agnocast_subscription_init(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_subscription_init(
    subscription_handle, node_handle, callback, callback_group, symbol, topic_name,
    queue_depth, pid_callback_info_id, now);

  record(subscription_handle, node_handle, callback, callback_group, symbol,
    topic_name, queue_depth, pid_callback_info_id, now);
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

void ros_trace_agnocast_init(
  const void * context_handle)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * context_handle,
    int64_t init_time
  ) {
    tracepoint(TRACEPOINT_PROVIDER, agnocast_init, context_handle, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  if (!data_container.is_assigned_agnocast_init()) {
    data_container.assign_agnocast_init(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_init(context_handle, now);

  record(context_handle, now);
}

void ros_trace_agnocast_node_init(
  const void * node_handle,
  const char * node_name,
  const char * namespace_)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * node_handle,
    const char * node_name,
    const char * namespace_,
    int64_t init_time
  ) {
    if (!controller.is_allowed_node(node_handle)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, agnocast_node_init,
      node_handle, node_name, namespace_, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  controller.add_node(node_handle, node_name);

  if (!data_container.is_assigned_agnocast_node_init()) {
    data_container.assign_agnocast_node_init(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_node_init(node_handle, node_name, namespace_, now);

  record(node_handle, node_name, namespace_, now);
}

void ros_trace_agnocast_timer_init(
  const void * node_handle,
  const uint64_t pid_timer_id,
  const void * callback,
  const void * callback_group,
  const char * symbol,
  int64_t period)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * node_handle,
    const uint64_t pid_timer_id,
    const void * callback,
    const void * callback_group,
    const char * symbol,
    int64_t period,
    int64_t init_time
  ) {
    if (!controller.is_allowed_pid_timer_id(pid_timer_id)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, agnocast_timer_init,
      node_handle, pid_timer_id, callback, callback_group, symbol, period, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  controller.add_pid_timer_id(pid_timer_id, node_handle);

  if (!data_container.is_assigned_agnocast_timer_init()) {
    data_container.assign_agnocast_timer_init(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_timer_init(
    node_handle, pid_timer_id, callback, callback_group, symbol, period, now);

  record(node_handle, pid_timer_id, callback, callback_group, symbol, period, now);
}

void ros_trace_agnocast_add_callback_group(
  const void * executor_addr,
  const void * callback_group_addr,
  const char * group_type_name)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * executor_addr,
    const void * callback_group_addr,
    const char * group_type_name,
    int64_t init_time
  ) {
    tracepoint(TRACEPOINT_PROVIDER, agnocast_add_callback_group,
      executor_addr, callback_group_addr, group_type_name, init_time);
  };

  if (!controller.is_allowed_process()) {
    return;
  }

  auto now = clock.now();

  if (!data_container.is_assigned_agnocast_add_callback_group()) {
    data_container.assign_agnocast_add_callback_group(record);
  }

  check_and_run_trace_node();

  data_container.store_agnocast_add_callback_group(
    executor_addr, callback_group_addr, group_type_name, now);

  record(executor_addr, callback_group_addr, group_type_name, now);
}

void ros_trace_agnocast_publish(
  const void * publisher_handle,
  int64_t entry_id)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *, int64_t);

  if (!controller.is_allowed_process()) {
    return;
  }

  if (controller.is_allowed_publisher_handle(publisher_handle) && context.is_recording_allowed()) {
    ((functionT) orig_func)(publisher_handle, entry_id);
  }
}

void ros_trace_agnocast_create_callable(
  const void * callable_handle,
  int64_t entry_id,
  uint64_t pid_callback_info_id)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *, int64_t, uint64_t);

  if (!controller.is_allowed_process()) {
    return;
  }

  controller.add_agnocast_callable(callable_handle, pid_callback_info_id);

  if (controller.is_allowed_agnocast_callable(callable_handle) && context.is_recording_allowed()) {
    ((functionT) orig_func)(callable_handle, entry_id, pid_callback_info_id);
  }
}

void ros_trace_agnocast_callable_start(
  const void * callable_handle)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *);

  if (!controller.is_allowed_process()) {
    return;
  }

  if (controller.is_allowed_agnocast_callable(callable_handle) && context.is_recording_allowed()) {
    ((functionT) orig_func)(callable_handle);
  }
}

void ros_trace_agnocast_callable_end(
  const void * callable_handle)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *);

  if (!controller.is_allowed_process()) {
    return;
  }

  if (controller.is_allowed_agnocast_callable(callable_handle) && context.is_recording_allowed()) {
    ((functionT) orig_func)(callable_handle);
  }
}

void ros_trace_agnocast_take(
  const void * subscription_handle,
  const void * message,
  int64_t entry_id)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *, const void *, int64_t);

  if (!controller.is_allowed_process()) {
    return;
  }

  if (controller.is_allowed_subscription_handle(subscription_handle)
      && context.is_recording_allowed()) {
    ((functionT) orig_func)(subscription_handle, message, entry_id);
  }
}

void ros_trace_agnocast_create_timer_callable(
  const void * callable,
  const uint64_t pid_timer_id)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *, const uint64_t);

  if (!controller.is_allowed_process()) {
    return;
  }

  controller.add_agnocast_timer_callable(callable, pid_timer_id);

  if (controller.is_allowed_pid_timer_id(pid_timer_id) && context.is_recording_allowed()) {
    ((functionT) orig_func)(callable, pid_timer_id);
  }
}

// clang-format on
}
