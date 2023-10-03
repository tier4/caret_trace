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

// disable tracepoint for declaration conflicting
#define ros_trace_rclcpp_publish ros_trace_rclcpp_publish_disabled
#define ros_trace_rclcpp_service_callback_added ros_trace_rclcpp_service_callback_added_disabled
#define ros_trace_rmw_publisher_init ros_trace_rmw_publisher_init_disabled
#define ros_trace_rmw_subscription_init ros_trace_rmw_subscription_init_disabled

#include "caret_trace/clock.hpp"
#include "caret_trace/context.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/tp.h"
#include "caret_trace/trace_node.hpp"
#include "caret_trace/tracing_controller.hpp"
#include "rcl/context.h"
#include "rclcpp/rclcpp.hpp"

#include <dlfcn.h>
#include <sys/types.h>
#include <time.h>

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#undef ros_trace_rclcpp_publish
#undef ros_trace_rclcpp_service_callback_added
#undef ros_trace_rmw_publisher_init
#undef ros_trace_rmw_subscription_init

// #define DEBUG_OUTPUT

#define DEFINE_ORIG_FUNC(TP_NAME) TP_NAME = dlsym(RTLD_NEXT, #TP_NAME)

std::unique_ptr<std::thread> trace_node_thread;
thread_local bool trace_filter_is_rcl_publish_recorded;

using std::string;
using std::vector;
static bool ignore_rcl_timer_init = false;

static vector<string> string_split(string & str, char delim)
{
  using std::stringstream;
  vector<string> elems;
  stringstream ss(str);
  string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

static bool is_python3_command()
{
  using std::ifstream;
  using std::ios;
  using std::istreambuf_iterator;
  using std::to_string;
  pid_t pid = getpid();
  string fileName = "/proc/" + to_string(pid) + "/cmdline";

  ifstream ifs(fileName, ios::in | ios::binary);
  if (!ifs) {
    return false;
  }
  istreambuf_iterator<char> it_ifs_begin(ifs);
  istreambuf_iterator<char> it_ifs_end{};
  vector<char> input_data(it_ifs_begin, it_ifs_end);
  if (!ifs) {
    ifs.close();
    return false;
  }
  ifs.close();

  vector<string> cmd_line;
  auto itr_begin = input_data.begin();
  auto itr_find = input_data.begin();
  for (;;) {
    itr_find = find(itr_begin, input_data.end(), 0x00);
    if (itr_find == input_data.end()) {
      break;
    }
    string output_string(itr_begin, itr_find);
    cmd_line.push_back(output_string);
    itr_begin = itr_find + 1;
  }

  // Check if the right side of the last '/' in cmdline[0] matches 'python3'.
  //   pattern 1: /usr/bin/python3 /opt/ros/humble/bin/ros2 launch ...
  //   pattern 2: python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket ...
  if (cmd_line.size() < 1) {
    return false;
  }
  if (cmd_line[0].find("python3") == string::npos) {
    return false;
  }
  auto subStr = string_split(cmd_line[0], '/');
  if (subStr[subStr.size() - 1].compare("python3") == 0) {
    return true;
  }
  return false;
}

void run_caret_trace_node()
{
  // When the python implementation node is executed,
  // this thread is executed without obtaining a global interpreter lock.
  // Therefore, this thread is executed asynchronously with the execution of the python node.
  auto & context = Singleton<Context>::get_instance();
  auto clock = context.get_clock();

  auto now = clock.now();
  auto distribution = getenv("ROS_DISTRO");
  tracepoint(TRACEPOINT_PROVIDER, caret_init, now, distribution);

  std::string node_name_base = "caret_trace";
  auto data_container = context.get_data_container_ptr();

  // If you try to create a Node before the context is created, it will fail.
  auto lttng = context.get_lttng_session_ptr();

  // For nodes in the cpp implementation,
  // a default context that has already been initialized is used.
  // For nodes implemented outside of cpp, such as python,
  // an uninitialized default context is used.
  rclcpp::NodeOptions option;

  if (!option.context()->is_valid()) {
    // Initialize the context if it has not been initialized.
    int argc = 1;
    const char * argv[] = {"caret_trace_node"};
    option.context()->init(argc, argv);
  }
  // Set global arguments to false to prevent the node to be generated from being renamed.
  option.use_global_arguments(false);
  auto trace_node = std::make_shared<TraceNode>(node_name_base, option, lttng, data_container);
  RCLCPP_INFO(trace_node->get_logger(), "%s started", trace_node->get_fully_qualified_name());

  ignore_rcl_timer_init = is_python3_command();

  context.assign_node(trace_node);
  auto exec = rclcpp::executors::SingleThreadedExecutor();
  exec.add_node(trace_node);
  exec.spin();
}

void check_and_run_trace_node()
{
  // NOTE:
  // This function should be executed after the context has been initialized.
  // Execute at initialization trace points other than rcl_init.
  // This function is called at the execution of various initialization functions,
  // so it is executed recursively.
  auto & context = Singleton<Context>::get_instance();
  static std::mutex mtx;
  if (context.is_node_initializing.load()) {
    return;
  }

  {  // Initialize TraceNode
    std::lock_guard<std::mutex> lock(mtx);
    if (!context.is_node_assigned() && !context.is_node_initializing.load()) {
      // Only one of the first threads execute this block.
      context.is_node_initializing.store(true);
      trace_node_thread = std::make_unique<std::thread>(run_caret_trace_node);
      trace_node_thread->detach();
    }
  }
}

namespace ORIG_FUNC
{
static void * DEFINE_ORIG_FUNC(ros_trace_callback_end);
static void * DEFINE_ORIG_FUNC(ros_trace_callback_start);
static void * DEFINE_ORIG_FUNC(ros_trace_dispatch_intra_process_subscription_callback);
static void * DEFINE_ORIG_FUNC(ros_trace_message_construct);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_lifecycle_state_machine_init);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_lifecycle_transition);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_node_init);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_publish);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_service_init);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_subscription_init);
static void * DEFINE_ORIG_FUNC(ros_trace_rcl_timer_init);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_callback_register);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_intra_publish);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_publish);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_service_callback_added);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_subscription_callback_added);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_subscription_init);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_timer_callback_added);
static void * DEFINE_ORIG_FUNC(ros_trace_rclcpp_timer_link_node);
}  // namespace ORIG_FUNC

// clang-format off

extern "C" {

// int clock_gettime(clockid_t clk_id, struct timespec *tp)
// {
//   static void * orig_func = dlsym(RTLD_NEXT, __func__);
//   using functionT = int (*)(clockid_t, struct timespec);
//   return ((functionT) orig_func)(clk_id, *tp);
// }

void ros_trace_rcl_node_init(
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](
    const void * node_handle,
    const void * rmw_handle,
    const char * node_name,
    const char * node_namespace,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();

    if (!controller.is_allowed_node(node_handle)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, rcl_node_init, node_handle, rmw_handle,
      node_name, node_namespace, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_node_init," <<
      node_handle << "," <<
      rmw_handle << "," <<
      node_name << "," <<
      node_namespace << std::endl;
#endif
  };

  auto now = clock.now();

  check_and_run_trace_node();

  if (!data_container.is_assigned_rcl_node_init()) {
    data_container.assign_rcl_node_init(record);
  }

  std::string ns = node_namespace;
  char last_char = ns[ns.length() - 1];
  if (last_char != '/') {
    ns = ns + '/';
  }
  auto node_ns_and_name = ns + node_name;

  controller.add_node(node_handle, node_ns_and_name);


  data_container.store_rcl_node_init(node_handle, rmw_handle, node_name, node_namespace, now);

  record(node_handle, rmw_handle, node_name, node_namespace, now);
}

void ros_trace_rcl_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static auto record = [](
    const void * subscription_handle,
    const void * node_handle,
    const void * rmw_subscription_handle,
    const char * topic_name,
    const size_t queue_depth,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();

    if (!controller.is_allowed_subscription_handle(subscription_handle)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, rcl_subscription_init, subscription_handle,
      node_handle, rmw_subscription_handle, topic_name, queue_depth, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_subscription_init," <<
      subscription_handle << "," <<
      node_handle << "," <<
      rmw_subscription_handle << "," <<
      topic_name << "," <<
      queue_depth << std::endl;
#endif
  };

  auto now = clock.now();

  controller.add_subscription_handle(node_handle, subscription_handle, topic_name);
  controller.add_rmw_subscription_handle(node_handle, rmw_subscription_handle, topic_name);

  if (!data_container.is_assigned_rcl_subscription_init()) {
    data_container.assign_rcl_subscription_init(record);
  }

  check_and_run_trace_node();

  data_container.store_rcl_subscription_init(
    subscription_handle, node_handle, rmw_subscription_handle, topic_name,
    reinterpret_cast<size_t>(queue_depth), now);

  record(subscription_handle, node_handle, rmw_subscription_handle, topic_name, queue_depth, now);
}

void ros_trace_rclcpp_subscription_init(
  const void * subscription_handle,
  const void * subscription)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static auto record = [](
    const void * subscription_handle,
    const void * subscription,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();
    if (!controller.is_allowed_subscription_handle(subscription_handle)){
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_subscription_init,
      subscription_handle, subscription, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_subscription_init," <<
      subscription_handle << "," <<
      subscription << std::endl;
#endif
  };

  auto now = clock.now();
  check_and_run_trace_node();

  controller.add_subscription(subscription_handle, subscription);

  if (!data_container.is_assigned_rclcpp_subscription_init()) {
    data_container.assign_rclcpp_subscription_init(record);
  }

  data_container.store_rclcpp_subscription_init(
    subscription_handle, subscription, now);

  record(subscription_handle, subscription, now);
}

void ros_trace_rclcpp_subscription_callback_added(
  const void * subscription,
  const void * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static auto record = [](
    const void * subscription,
    const void * callback,
    int64_t init_time
  ) {
    if (!controller.is_allowed_callback(callback)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_subscription_callback_added,
      subscription, callback, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_subscription_callback_added," <<
      subscription << "," <<
      callback << std::endl;
#endif
  };

  auto now = clock.now();
  check_and_run_trace_node();

  controller.add_subscription_callback(subscription, callback);

  if (!data_container.is_assigned_rclcpp_subscription_callback_added()) {
    data_container.assign_rclcpp_subscription_callback_added(record);
  }

  data_container.store_rclcpp_subscription_callback_added(
    subscription, callback, now);

  record(subscription, callback, now);
}

void ros_trace_rclcpp_timer_callback_added(const void * timer_handle, const void * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static auto record = [](
    const void * timer_handle,
    const void * callback,
    int64_t init_time
  ) {
  if (!controller.is_allowed_callback(callback)) {
    return;
  }
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_timer_callback_added, timer_handle, callback, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_timer_callback_added," <<
      timer_handle << "," <<
      callback << std::endl;
#endif
  };

  auto now = clock.now();
  check_and_run_trace_node();

  controller.add_timer_callback(timer_handle, callback);

  if (!data_container.is_assigned_rclcpp_timer_callback_added()) {
    data_container.assign_rclcpp_timer_callback_added(record);
  }

  data_container.store_rclcpp_timer_callback_added(timer_handle, callback, now);
  record(timer_handle, callback, now);
}

void ros_trace_rclcpp_timer_link_node(const void * timer_handle, const void * node_handle)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();
  static auto record = [](
    const void * timer_handle,
    const void * node_handle,
    int64_t init_time
  ) {
  if (!controller.is_allowed_node(node_handle)) {
    return;
  }
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_timer_link_node, timer_handle, node_handle, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_timer_link_node," <<
      timer_handle << "," <<
      node_handle << std::endl;
#endif
  };

  auto now = clock.now();
  check_and_run_trace_node();

  controller.add_timer_handle(node_handle, timer_handle);

  if (!data_container.is_assigned_rclcpp_timer_link_node()) {
    data_container.assign_rclcpp_timer_link_node(record);
  }

  data_container.store_rclcpp_timer_link_node(timer_handle, node_handle, now);
  record(timer_handle, node_handle, now);
}

void ros_trace_callback_start(const void * callback, bool is_intra_process)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();

  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, bool);

  if (controller.is_allowed_callback(callback) &&
    context.is_recording_allowed())
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
    context.is_recording_allowed())
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
    context.is_recording_allowed())
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
    context.is_recording_allowed())
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
  const void * message)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_publisher_handle(publisher_handle) &&
    context.is_recording_allowed())
  {
    ((functionT) orig_func)(publisher_handle, message);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_publish," <<
      publisher_handle << "," <<
      message << std::endl;
#endif
  }
}

void ros_trace_rclcpp_intra_publish(
  const void * publisher_handle,
  const void * message)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  using functionT = void (*)(const void *, const void *);

  if (controller.is_allowed_publisher_handle(publisher_handle) &&
    context.is_recording_allowed())
  {
    ((functionT) orig_func)(publisher_handle, message);
#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_intra_publish," <<
      publisher_handle << "," <<
      message << std::endl;
#endif
  }
}

void ros_trace_rcl_timer_init(
  const void * timer_handle,
  int64_t period)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  // TODO(hsgwa): Add filtering of timer initialization using node_handle

  if (ignore_rcl_timer_init == true) {
    return;
  }

  static auto record = [](const void * timer_handle, int64_t period, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rcl_timer_init, timer_handle, period, init_time);
  };

  auto now = clock.now();
  if (!data_container.is_assigned_rcl_timer_init()) {
    data_container.assign_rcl_timer_init(record);
  }

  data_container.store_rcl_timer_init(timer_handle, period, now);
  record(timer_handle, period, now);

#ifdef DEBUG_OUTPUT
  std::cerr << "rcl_timer_init," <<
    timer_handle << "," <<
    period << std::endl;
#endif
}

void ros_trace_rcl_init(
  const void * context_handle)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();

  static auto record = [](  const void * context_handle, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rcl_init, context_handle, init_time);
  };

  auto now = clock.now();
  if (!data_container.is_assigned_rcl_init()) {
    data_container.assign_rcl_init(record);
  }

  data_container.store_rcl_init(context_handle, now);
  record(context_handle, now);
#ifdef DEBUG_OUTPUT
  std::cerr << "rcl_init," <<
    context_handle << std::endl;
#endif
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
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  static auto record = [](  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth,
  int64_t init_time
) {
    tracepoint(TRACEPOINT_PROVIDER, rcl_publisher_init, publisher_handle, node_handle,
    rmw_publisher_handle, topic_name, queue_depth, init_time);
  };

  auto now = clock.now();
  controller.add_publisher_handle(node_handle, publisher_handle, topic_name);

  // TODO(hsgwa): support topic_name filtering
  // It seems to be executed before the topic name and node name are known.

  if (!data_container.is_assigned_rcl_publisher_init()) {
    data_container.assign_rcl_publisher_init(record);
  }

  check_and_run_trace_node();

  data_container.store_rcl_publisher_init(
    publisher_handle,
    node_handle,
    rmw_publisher_handle,
    topic_name,
    queue_depth,
    now);

  record(publisher_handle, node_handle, rmw_publisher_handle, topic_name, queue_depth, now);
#ifdef DEBUG_OUTPUT
  std::cerr << "rcl_publisher_init," <<
    publisher_handle << "," <<
    node_handle << "," <<
    rmw_publisher_handle << "," <<
    topic_name << "," <<
    queue_depth << std::endl;
#endif
}

void ros_trace_rcl_publish(
  const void * publisher_handle,
  const void * message)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = context.get_controller();

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_publisher_handle(publisher_handle) && context.is_recording_allowed()) {
    ((functionT) orig_func)(publisher_handle, message);
    trace_filter_is_rcl_publish_recorded = true;
#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_publish," <<
      publisher_handle << "," <<
      message << std::endl;
#endif
  } else {
    trace_filter_is_rcl_publish_recorded = false;
  }
}

void ros_trace_rcl_service_init(
  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name,
  int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rcl_service_init, service_handle,
    node_handle, rmw_service_handle, service_name, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_service_init," <<
      service_handle << "," <<
      node_handle << "," <<
      rmw_service_handle << "," <<
      service_name << std::endl;
#endif
  };

  auto now = clock.now();
  if (!data_container.is_assigned_rcl_service_init()) {
    data_container.assign_rcl_service_init(record);
  }

  check_and_run_trace_node();

  data_container.store_rcl_service_init(
    service_handle, node_handle, rmw_service_handle, service_name, now);
  record(service_handle, node_handle, rmw_service_handle, service_name, now);
}

void ros_trace_rclcpp_service_callback_added(
  const void * service_handle,
  const char * callback)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](const void * service_handle, const char * callback, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_service_callback_added,
      service_handle, callback, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_service_callback_added," <<
      service_handle << "," <<
      callback << std::endl;
#endif
  };
  auto now = clock.now();

  check_and_run_trace_node();

  if (!data_container.is_assigned_rclcpp_service_callback_added()) {
    data_container.assign_rclcpp_service_callback_added(record);
  }
  data_container.store_rclcpp_service_callback_added(service_handle, callback, now);

  record(service_handle, callback, now);
}

void ros_trace_rcl_client_init(
  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name,
  int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rcl_client_init, client_handle, node_handle,
      rmw_client_handle, service_name, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_client_init," <<
      client_handle << "," <<
      node_handle << "," <<
      rmw_client_handle << "," <<
      service_name << std::endl;
#endif
  };
  auto now = clock.now();

  if (!data_container.is_assigned_rcl_client_init()) {
    data_container.assign_rcl_client_init(record);
  }

  check_and_run_trace_node();

  data_container.store_rcl_client_init(
    client_handle, node_handle, rmw_client_handle,
    service_name, now);
  record(client_handle, node_handle, rmw_client_handle, service_name, now);
}

void ros_trace_rclcpp_callback_register(
  const void * callback,
  const char * symbol)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();

  static auto record = [](
    const void * callback,
    const char * symbol,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();
    if (!controller.is_allowed_callback(callback)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_callback_register, callback, symbol, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_callback_register," <<
      callback << "," <<
      symbol << std::endl;
#endif
  };
  auto now = clock.now();

  check_and_run_trace_node();

  if (!data_container.is_assigned_rclcpp_callback_register()) {
    data_container.assign_rclcpp_callback_register(record);
  }

  data_container.store_rclcpp_callback_register(callback, symbol, now);

  record(callback, symbol, now);
}

// For ros2 distributions after iron.
bool ros_trace_enabled_rclcpp_callback_register()
{
  return true;
}

// For ros2 distributions after iron.
void ros_trace_do_rclcpp_callback_register(
  const void * callback,
  const char * symbol)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();

  static auto record = [](
    const void * callback,
    const char * symbol,
    int64_t init_time
  ) {
    static auto & context = Singleton<Context>::get_instance();
    static auto & controller = context.get_controller();
    if (!controller.is_allowed_callback(callback)) {
      return;
    }
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_callback_register, callback, symbol, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "rclcpp_callback_register," <<
      callback << "," <<
      symbol << std::endl;
#endif
  };
  auto now = clock.now();

  check_and_run_trace_node();

  if (!data_container.is_assigned_rclcpp_callback_register()) {
    data_container.assign_rclcpp_callback_register(record);
  }

  data_container.store_rclcpp_callback_register(callback, symbol, now);

  record(callback, symbol, now);
}

void ros_trace_rcl_lifecycle_state_machine_init(
  const void * node_handle,
  const void * state_machine)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto record = [](
    const void * node_handle,
    const void * state_machine,
    int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rcl_lifecycle_state_machine_init,
      node_handle, state_machine, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "rcl_lifecycle_state_machine_init," <<
      node_handle << "," <<
      state_machine << std::endl;
#endif
  };
  auto now = clock.now();

  check_and_run_trace_node();

  record(node_handle, state_machine, now);
}

void ros_trace_rcl_lifecycle_transition(
  const void * state_machine,
  const char * start_label,
  const char * goal_label)
{
  static auto & context = Singleton<Context>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const char *, const char *);

  if (context.is_recording_allowed()) {
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
  if (context.is_recording_allowed()) {
    ((functionT) orig_func)(original_message, constructed_message);

#ifdef DEBUG_OUTPUT
    std::cerr << "message_construct," <<
      original_message << "," <<
      constructed_message << std::endl;
#endif
  }
}

void ros_trace_rclcpp_executor_execute(
  const void * handle
)
{
  (void) handle;
// #ifdef DEBUG_OUTPUT
//   std::cerr << "rclcpp_executor_execute," << handle << std::endl;
// #endif
}

void ros_trace_rclcpp_executor_wait_for_work(
  const int64_t timeout
)
{
  (void) timeout;
// #ifdef DEBUG_OUTPUT
//   std::cerr << "rclcpp_executor_wait_for_work," << timeout << std::endl;
// #endif
}

void ros_trace_rclcpp_executor_get_next_ready()
{
// #ifdef DEBUG_OUTPUT
//   std::cerr << "rclcpp_executor_get_next_ready," << std::endl;
// #endif
}

void ros_trace_rcl_take(
  const void * message
)
{
  (void) message;
// #ifdef DEBUG_OUTPUT
//   std::cerr << "rcl_take," << message << std::endl;
// #endif
}

void ros_trace_rclcpp_take(
  const void * message
)
{
  (void) message;
}

void ros_trace_rmw_take(
  const void * rmw_subscription_handle,
  const void * message,
  int64_t source_timestamp,
  const bool taken
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, int64_t, const bool);
  if (controller.is_allowed_rmw_subscription_handle(rmw_subscription_handle) &&
    context.is_recording_allowed())
  {
    ((functionT) orig_func)(rmw_subscription_handle, message,
      source_timestamp, taken);

#ifdef DEBUG_OUTPUT
  std::cerr << "rmw_take," <<
    rmw_subscription_handle << "," <<
    message << "," <<
    source_timestamp << "," <<
    taken << "," << std::endl;
#endif
  }
}

void ros_trace_rmw_publish(
  const void * message
)
{
  if (trace_filter_is_rcl_publish_recorded) {
    tracepoint(TRACEPOINT_PROVIDER, dds_write, message);
#ifdef DEBUG_OUTPUT
    std::cerr << "rmw_publish," <<
      message << "," << std::endl;
#endif
  }
}

void ros_trace_rmw_publisher_init(
  const void * rmw_publisher_handle,
  const uint8_t gid
)
{
  (void) rmw_publisher_handle;
  (void) gid;
// #ifdef DEBUG_OUTPUT
//   std::cerr << "rmw_publisher_init," <<
//     rmw_publisher_handle << "," <<
//     gid << "," << std::endl;
// #endif
}

void ros_trace_rmw_subscription_init(
  const void * rmw_subscription_handle,
  const uint8_t gid
)
{
  (void) rmw_subscription_handle;
  (void) gid;
// #ifdef DEBUG_OUTPUT
//   std::cerr << "rmw_subscription_init," <<
//     rmw_subscription_handle << "," <<
//     gid << "," << std::endl;
// #endif
}

void ros_trace_rclcpp_buffer_to_ipb(
    const void * buffer,
    const void * ipb
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  controller.add_buffer(buffer, ipb);
  static auto record = [](
    const void * buffer,
    const void * ipb,
    int64_t init_timestamp
  ){
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_buffer_to_ipb,
      buffer, ipb, init_timestamp);

#ifdef DEBUG_OUTPUT
  std::cerr << "rclcpp_buffer_to_ipb," <<
    buffer << "," <<
    ipb << std::endl;
#endif
  };

  auto now = clock.now();

  if (!data_container.is_assigned_rclcpp_buffer_to_ipb()){
    data_container.assign_rclcpp_buffer_to_ipb(record);
  }

  check_and_run_trace_node();

  data_container.store_rclcpp_buffer_to_ipb(buffer, ipb, now);
  record(buffer, ipb, now);
}

void ros_trace_rclcpp_ipb_to_subscription(
    const void * ipb,
    const void * subscription
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto & controller = context.get_controller();

  controller.add_ipb(ipb, subscription);
  static auto record = [](
    const void * ipb,
    const void * subscription,
    int64_t init_timestamp
  ){
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_ipb_to_subscription,
      ipb, subscription, init_timestamp);

#ifdef DEBUG_OUTPUT
  std::cerr << "rclcpp_ipb_to_subscription," <<
    ipb << "," <<
    subscription << std::endl;
#endif
  };

  auto now = clock.now();

  if (!data_container.is_assigned_rclcpp_ipb_to_subscription()){
    data_container.assign_rclcpp_ipb_to_subscription(record);
  }
  check_and_run_trace_node();

  data_container.store_rclcpp_ipb_to_subscription(ipb, subscription, now);
  record(ipb, subscription, now);
}

void ros_trace_rclcpp_construct_ring_buffer(
    const void * buffer,
    const uint64_t capacity
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();

  static auto record = [](
    const void * buffer,
    uint64_t capacity,
    int64_t init_timestamp
  ){
    tracepoint(TRACEPOINT_PROVIDER, rclcpp_construct_ring_buffer,
      buffer, capacity, init_timestamp);

#ifdef DEBUG_OUTPUT
  std::cerr << "rclcpp_construct_ring_buffer," <<
    buffer << "," <<
    capacity << std::endl;
#endif
  };

  auto now = clock.now();

  if (!data_container.is_assigned_rclcpp_construct_ring_buffer()) {
    data_container.assign_rclcpp_construct_ring_buffer(record);
  }

  check_and_run_trace_node();

  data_container.store_rclcpp_construct_ring_buffer(
    buffer, capacity, now);
  record(buffer, capacity, now);
}

void ros_trace_rclcpp_ring_buffer_enqueue(
    const void * buffer,
    const uint64_t index,
    const uint64_t size,
    const bool overwritten
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const uint64_t, const uint64_t, bool);
  if (controller.is_allowed_buffer(buffer) &&
    context.is_recording_allowed())
  {
    ((functionT) orig_func)(buffer, index, size, overwritten);

#ifdef DEBUG_OUTPUT
  std::cerr << "rclcpp_ring_buffer_enqueue," <<
    buffer << "," <<
    index << "," <<
    size << "," <<
    overwritten << std::endl;
#endif
  }
}

void ros_trace_rclcpp_ring_buffer_dequeue(
    const void * buffer,
    const uint64_t index,
    const uint64_t size
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const uint64_t, const uint64_t);
  if (controller.is_allowed_buffer(buffer) &&
    context.is_recording_allowed())
  {
  ((functionT) orig_func)(buffer, index, size);

#ifdef DEBUG_OUTPUT
  std::cerr << "rclcpp_ring_buffer_dequeue," <<
    buffer << "," <<
    index << "," <<
    size << "," << std::endl;
#endif
  }
}

void ros_trace_rclcpp_ring_buffer_clear(
    const void * buffer
)
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & controller = context.get_controller();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  if (controller.is_allowed_buffer(buffer) &&
    context.is_recording_allowed())
  {
    using functionT = void (*)(const void *);
    ((functionT) orig_func)(buffer);

#ifdef DEBUG_OUTPUT
  std::cerr << "rclcpp_ring_buffer_clear," <<
    buffer << "," << std::endl;
#endif
  }
}

// clang-format on
}
