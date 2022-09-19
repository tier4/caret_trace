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
#include <thread>
#include <sys/types.h>
#include <mutex>


#define ros_trace_rclcpp_publish _ros_trace_rclcpp_publish
#define ros_trace_rclcpp_service_callback_added _ros_trace_rclcpp_service_callback_added

#include "rclcpp/rclcpp.hpp"
#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/trace_node.hpp"
#include "caret_trace/context.hpp"

#undef ros_trace_rclcpp_publish
#undef ros_trace_rclcpp_service_callback_added

// #define DEBUG_OUTPUT

std::unique_ptr<std::thread> trace_node_thread;

void run_caret_trace_node()
{
  std::string node_name_base = "caret_trace";
  auto & context = Singleton<Context>::get_instance();
  auto data_container = context.get_data_container_ptr();
  // contextが作られる前にNodeを作ろうとすると失敗する。
  auto trace_node = std::make_shared<TraceNode>(node_name_base, data_container);
  context.assign_node(trace_node);
  auto exec = rclcpp::executors::SingleThreadedExecutor();
  exec.add_node(trace_node);
  exec.spin();
}

extern "C" {
void ros_trace_rcl_node_init(
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static std::mutex mtx;

  {  // TraceNodeの初期化
    std::lock_guard<std::mutex> lock(mtx);
    if (!context.is_node_assigned() && !context.is_node_initializing.load()) {
      context.is_node_initializing.store(true);
      trace_node_thread = std::make_unique<std::thread>(run_caret_trace_node);
      trace_node_thread->detach();
    }
  }

  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, const char *, const char *);

  if (!data_container.is_assigned_rcl_node_init()) {
    data_container.assign_rcl_node_init((functionT) orig_func);
  }

  trace_node_thread = std::make_unique<std::thread>();

  std::string ns = node_namespace;
  char last_char = ns[ns.length() - 1];
  if (last_char != '/') {
    ns = ns + '/';
  }
  auto node_ns_and_name = ns + node_name;

  controller.add_node(node_handle, node_ns_and_name);

  bool pending = data_container.store_rcl_node_init(
    node_handle, rmw_handle, node_name,
    node_namespace);

  bool record = context.is_recording_enabled() || pending;
  if (controller.is_allowed_node(node_handle) && record) {
    ((functionT) orig_func)(node_handle, rmw_handle, node_name, node_namespace);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_node_init," <<
      node_handle << "," <<
      rmw_handle << "," <<
      node_name << "," <<
      node_namespace << std::endl;
#endif
  }
}

void ros_trace_rcl_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription_handle(node_handle, subscription_handle, topic_name);

  using functionT =
    void (*)(const void *, const void *, const void *, const char *, const size_t);

  if (!data_container.is_assigned_rcl_subscription_init()) {
    data_container.assign_rcl_subscription_init((functionT) orig_func);
  }

  bool pending = data_container.store_rcl_subscription_init(
    subscription_handle, node_handle, rmw_subscription_handle, topic_name,
    reinterpret_cast<size_t>(queue_depth));

  bool record = context.is_recording_enabled() || pending;
  if (controller.is_allowed_subscription_handle(subscription_handle) && record) {
    ((functionT) orig_func)(
      subscription_handle, node_handle, rmw_subscription_handle, topic_name,
      queue_depth);
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_subscription_init," <<
      subscription_handle << "," <<
      node_handle << "," <<
      rmw_subscription_handle << "," <<
      topic_name << "," <<
      queue_depth << std::endl;
#endif
  }
}

void ros_trace_rclcpp_subscription_init(
  const void * subscription_handle,
  const void * subscription)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription(subscription_handle, subscription);

  using functionT = void (*)(const void *, const void *);
  if (!data_container.is_assigned_rclcpp_subscription_init()) {
    data_container.assign_rclcpp_subscription_init((functionT) orig_func);
  }

  bool pending = data_container.store_rclcpp_subscription_init(subscription_handle, subscription);
  bool record = context.is_recording_enabled() || pending;

  if (controller.is_allowed_subscription_handle(subscription_handle) && record) {
    ((functionT) orig_func)(subscription_handle, subscription);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_subscription_init," <<
      subscription_handle << "," <<
      subscription << std::endl;
#endif
  }
}

void ros_trace_rclcpp_subscription_callback_added(
  const void * subscription,
  const void * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription_callback(subscription, callback);

  using functionT = void (*)(const void *, const void *);
  if (!data_container.is_assigned_rclcpp_subscription_callback_added()) {
    data_container.assign_rclcpp_subscription_callback_added((functionT)orig_func);
  }

  bool pending = data_container.store_rclcpp_subscription_callback_added(subscription, callback);
  bool record = context.is_recording_enabled() || pending;

  if (controller.is_allowed_callback(callback) && record) {
    ((functionT) orig_func)(subscription, callback);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_subscription_callback_added," <<
      subscription << "," <<
      callback << std::endl;
#endif
  }
}

void ros_trace_rclcpp_timer_callback_added(const void * timer_handle, const void * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_timer_callback(timer_handle, callback);
  using functionT = void (*)(const void *, const void *);

  if (!data_container.is_assigned_rclcpp_timer_callback_added()) {
    data_container.assign_rclcpp_timer_callback_added((functionT) orig_func);
  }

  bool pending = data_container.store_rclcpp_timer_callback_added(timer_handle, callback);
  bool record = context.is_recording_enabled() || pending;
  if (controller.is_allowed_callback(callback) && record) {
    ((functionT) orig_func)(timer_handle, callback);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_timer_callback_added," <<
      timer_handle << "," <<
      callback << std::endl;
#endif
  }
}

void ros_trace_rclcpp_timer_link_node(const void * timer_handle, const void * node_handle)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_timer_handle(node_handle, timer_handle);

  using functionT = void (*)(const void *, const void *);

  if (!data_container.is_assigned_rclcpp_timer_link_node()) {
    data_container.assign_rclcpp_timer_link_node((functionT) orig_func);
  }

  bool pending = data_container.store_rclcpp_timer_link_node(timer_handle, node_handle);
  bool record = context.is_recording_enabled() || pending;
  if (controller.is_allowed_node(node_handle) && record) {
    ((functionT) orig_func)(timer_handle, node_handle);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_timer_link_node," <<
      timer_handle << "," <<
      node_handle << std::endl;
#endif
  }
}

void ros_trace_callback_start(const void * callback, bool is_intra_process)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();

  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, bool);

  if (controller.is_allowed_callback(callback) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(callback, is_intra_process);
#ifdef DEBUG_OUTPUT
    std::cerr << "callback_start," <<
      callback << "," <<
      is_intra_process << std::endl;
#endif
  }
}

void ros_trace_callback_end(const void * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *);
  if (controller.is_allowed_callback(callback) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(callback);

#ifdef DEBUG_OUTPUT
    std::cerr << "callback_end," <<
      callback << std::endl;
#endif
  }
}

void ros_trace_dispatch_subscription_callback(
  const void * message,
  const void * callback,
  const uint64_t source_timestamp,
  const uint64_t message_timestamp)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *, const uint64_t, const uint64_t);
  if (controller.is_allowed_callback(callback) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(message, callback, source_timestamp, message_timestamp);

#ifdef DEBUG_OUTPUT
    std::cerr << "dispatch_subscription_callback," <<
      message << "," <<
      callback << "," <<
      source_timestamp << "," <<
      message_timestamp << std::endl;
#endif
  }
}

void ros_trace_dispatch_intra_process_subscription_callback(
  const void * message,
  const void * callback,
  const uint64_t message_timestamp)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *, const uint64_t);
  if (controller.is_allowed_callback(callback) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(message, callback, message_timestamp);

#ifdef DEBUG_OUTPUT
    std::cerr << "dispatch_intra_process_subscription_callback," <<
      message << "," <<
      callback << "," <<
      message_timestamp << std::endl;
#endif
  }
}

void ros_trace_rclcpp_publish(
  const void * publisher_handle,
  const void * message,
  const uint64_t message_timestamp)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *, const uint64_t);
  if (controller.is_allowed_publisher_handle(publisher_handle) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(publisher_handle, message, message_timestamp);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_publish," <<
      publisher_handle << "," <<
      message << "," <<
      message_timestamp << std::endl;
#endif
  }
}

void ros_trace_rclcpp_intra_publish(
  const void * publisher_handle,
  const void * message,
  const uint64_t message_timestamp)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *, const uint64_t message_timestamp);

  if (controller.is_allowed_publisher_handle(publisher_handle) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(publisher_handle, message, message_timestamp);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_intra_publish," <<
      publisher_handle << "," <<
      message << "," <<
      message_timestamp << std::endl;
#endif
  }
}

void ros_trace_rcl_timer_init(
  const void * timer_handle,
  int64_t period)
{
  static auto & context = Singleton<Context>::get_instance();

  static auto & data_container = context.get_data_container();
  // TODO(hsgwa): Add filtering of timer initialization using node_handle
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, int64_t);

  if (!data_container.is_assigned_rcl_timer_init()) {
    data_container.assign_rcl_timer_init((functionT) orig_func);
  }

  bool pending = data_container.store_rcl_timer_init(timer_handle, period);
  if (context.is_recording_enabled() || pending) {
    ((functionT) orig_func)(timer_handle, period);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_timer_init," <<
      timer_handle << "," <<
      period << std::endl;
#endif
  }
}

void ros_trace_rcl_init(
  const void * context_handle)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  using functionT = void (*)(const void *);
  ((functionT) orig_func)(context_handle);

  if (!data_container.is_assigned_rcl_init()) {
    data_container.assign_rcl_init((functionT)orig_func);
  }
  bool pending = data_container.store_rcl_init(context_handle);
  if (context.is_recording_enabled() || pending) {
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_init," <<
      context_handle << std::endl;
#endif
  }
}


void ros_trace_rcl_publisher_init(
  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  controller.add_publisher_handle(node_handle, publisher_handle, topic_name);

  using functionT = void (*)(const void *, const void *, const void *, const char *, const size_t);
  // TODO(hsgwa): support topic_name filtering
  // It seems to be executed before the topic name and node name are known.

  if (!data_container.is_assigned_rcl_publisher_init()) {
    data_container.assign_rcl_publisher_init((functionT) orig_func);
  }

  bool pending = data_container.store_rcl_publisher_init(
    publisher_handle,
    node_handle,
    rmw_publisher_handle,
    topic_name,
    queue_depth);

  if (context.is_recording_enabled() || pending) {
    ((functionT) orig_func)(
      publisher_handle,
      node_handle,
      rmw_publisher_handle,
      topic_name,
      queue_depth);
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_publisher_init," <<
      publisher_handle << "," <<
      node_handle << "," <<
      rmw_publisher_handle << "," <<
      topic_name << "," <<
      queue_depth << std::endl;
#endif
  }
}

void ros_trace_rcl_publish(
  const void * publisher_handle,
  const void * message)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_publisher_handle(publisher_handle) &&
    context.is_recording_enabled())
  {
    ((functionT) orig_func)(publisher_handle, message);
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_publish," <<
      publisher_handle << "," <<
      message << std::endl;
#endif
  }
}

void ros_trace_rcl_service_init(
  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, const void *, const char *);

  if (!data_container.is_assigned_rcl_service_init()) {
    data_container.assign_rcl_service_init((functionT) orig_func);
  }

  bool pending = data_container.store_rcl_service_init(
    service_handle, node_handle, rmw_service_handle,
    service_name);
  if (context.is_recording_enabled() || pending) {
    ((functionT) orig_func)(service_handle, node_handle, rmw_service_handle, service_name);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_service_init," <<
      service_handle << "," <<
      node_handle << "," <<
      rmw_service_handle << "," <<
      service_name << std::endl;
#endif
  }
}

void ros_trace_rclcpp_service_callback_added(
  const void * service_handle,
  const char * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *);

  if (!data_container.is_assigned_rclcpp_service_callback_added()) {
    data_container.assign_rclcpp_service_callback_added((functionT) orig_func);
  }
  bool pending = data_container.store_rclcpp_service_callback_added(service_handle, callback);

  if (context.is_recording_enabled() || pending) {
    ((functionT) orig_func)(service_handle, callback);

#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_service_callback_added," <<
      service_handle << "," <<
      callback << std::endl;
#endif
  }
}

void ros_trace_rcl_client_init(
  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, const void *, const char *);

  if (!data_container.is_assigned_rcl_client_init()) {
    data_container.assign_rcl_client_init((functionT) orig_func);
  }
  bool pending = data_container.store_rcl_client_init(
    client_handle, node_handle, rmw_client_handle,
    service_name);
  if (context.is_recording_enabled() || pending) {
    ((functionT) orig_func)(client_handle, node_handle, rmw_client_handle, service_name);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_client_init," <<
      client_handle << "," <<
      node_handle << "," <<
      rmw_client_handle << "," <<
      service_name << std::endl;
#endif
  }
}

void ros_trace_rclcpp_callback_register(
  const void * callback,
  const char * symbol)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & data_container = context.get_data_container();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();
  using functionT = void (*)(const void *, const char *);


  if (!data_container.is_assigned_rclcpp_callback_register()) {
    data_container.assign_rclcpp_callback_register((functionT) orig_func);
  }

  bool pending = data_container.store_rclcpp_callback_register(callback, symbol);
  bool record = context.is_recording_enabled() || pending;

  if (controller.is_allowed_callback(callback) && record) {
    ((functionT) orig_func)(callback, symbol);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_callback_register," <<
      callback << "," <<
      symbol << std::endl;
#endif
  }
}

void ros_trace_rcl_lifecycle_state_machine_init(
  const void * node_handle,
  const void * state_machine)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *);

  if (context.is_recording_enabled()) {
    ((functionT) orig_func)(node_handle, state_machine);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_lifecycle_state_machine_init," <<
      node_handle << "," <<
      state_machine << std::endl;
#endif
  }
}

void ros_trace_rcl_lifecycle_transition(
  const void * state_machine,
  const char * start_label,
  const char * goal_label)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const char *, const char *);

  if (context.is_recording_enabled()) {
    ((functionT) orig_func)(state_machine, start_label, goal_label);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_lifecycle_transition," <<
      state_machine << "," <<
      start_label << "," <<
      goal_label << "," << std::endl;
#endif
  }
}

void ros_trace_message_construct(
  const void * original_message,
  const void * constructed_message)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *);
  if (context.is_recording_enabled()) {
    ((functionT) orig_func)(original_message, constructed_message);

#ifdef DEBUG_OUTPUT
    std::cerr << "message_construct," <<
      original_message << "," <<
      constructed_message << std::endl;
#endif
  }
}
}
