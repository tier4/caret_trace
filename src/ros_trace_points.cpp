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

#include <dlfcn.h>

#include <iostream>
#include <memory>
#include <iomanip>
#include <string>

#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/singleton.hpp"

extern "C" {
void ros_trace_rcl_node_init(
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  std::string ns = node_namespace;
  char last_char = ns[ns.length() - 1];
  if (last_char != '/') {
    ns = ns + '/';
  }
  auto node_ns_and_name = ns + node_name;

  controller.add_node(node_handle, node_ns_and_name);

  using functionT = void (*)(const void *, const void *, const char *, const char *);
  ((functionT) orig_func)(node_handle, rmw_handle, node_name, node_namespace);
}

void ros_trace_rcl_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription_handle(node_handle, subscription_handle, topic_name);

  using functionT =
    void (*)(const void *, const void *, const void *, const char *, const size_t);
  ((functionT) orig_func)(
    subscription_handle, node_handle, rmw_subscription_handle, topic_name,
    queue_depth);
}

void ros_trace_rclcpp_subscription_init(
  const void * subscription_handle,
  const void * subscription)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription(subscription_handle, subscription);

  using functionT = void (*)(const void *, const void *);
  ((functionT) orig_func)(subscription_handle, subscription);
}

void ros_trace_rclcpp_subscription_callback_added(
  const void * subscription,
  const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription_callback(subscription, callback);

  using functionT = void (*)(const void *, const void *);
  ((functionT) orig_func)(subscription, callback);
}

void ros_trace_rclcpp_timer_callback_added(const void * timer_handle, const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_timer_callback(timer_handle, callback);

  using functionT = void (*)(const void *, const void *);
  ((functionT) orig_func)(timer_handle, callback);
}

void ros_trace_rclcpp_timer_link_node(const void * timer_handle, const void * node_handle)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_timer_handle(node_handle, timer_handle);

  using functionT = void (*)(const void *, const void *);
  ((functionT) orig_func)(timer_handle, node_handle);
}

void ros_trace_callback_start(const void * callback, bool is_intra_process)
{
  static auto & controller = Singleton<TracingController>::get_instance();

  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, bool);

  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(callback, is_intra_process);
  }
}

void ros_trace_callback_end(const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *);
  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(callback);
  }
}

void ros_trace_dispatch_subscription_callback(
  const void * message,
  const void * callback,
  const uint64_t source_timestamp)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *, const uint64_t);
  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(message, callback, source_timestamp);
  }
}

void ros_trace_dispatch_intra_process_subscription_callback(
  const void * message,
  const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(message, callback);
  }
}
}
