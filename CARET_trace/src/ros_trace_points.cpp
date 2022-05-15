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
#include <mutex>

#include "caret_trace/tp.h"
#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/clock.hpp"
#include "caret_trace/debug.hpp"
#include "caret_trace/thread_local.hpp"


std::mutex debug_mutex;
// #define DEBUG_OUTPUT

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

  if (controller.is_allowed_node(node_handle)) {
    ((functionT) orig_func)(node_handle, rmw_handle, node_name, node_namespace);

#ifdef DEBUG_OUTPUT
    debug.print(
      "rcl_node_init",
      node_handle,
      rmw_handle,
      node_name,
      node_namespace
    );
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
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription_handle(node_handle, subscription_handle, topic_name);

  using functionT =
    void (*)(const void *, const void *, const void *, const char *, const size_t);

  if (controller.is_allowed_subscription_handle(subscription_handle)) {
    ((functionT) orig_func)(
      subscription_handle, node_handle, rmw_subscription_handle, topic_name,
      queue_depth);
#ifdef DEBUG_OUTPUT
    debug.print(
      "rcl_subscription_init",
      subscription_handle,
      node_handle,
      rmw_subscription_handle,
      topic_name,
      queue_depth
    );
#endif
  }
}

void ros_trace_rclcpp_subscription_init(
  const void * subscription_handle,
  const void * subscription)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription(subscription_handle, subscription);

  set_subscription_handle(subscription_handle);

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_subscription_handle(subscription_handle)) {
    ((functionT) orig_func)(subscription_handle, subscription);
#ifdef DEBUG_OUTPUT
    debug.print(
      "rclcpp_subscription_init",
      subscription_handle,
      subscription
    );
#endif
  }
}

void ros_trace_rclcpp_subscription_callback_added(
  const void * subscription,
  const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_subscription_callback(subscription, callback);
  unset_subscription_handle();

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(subscription, callback);
#ifdef DEBUG_OUTPUT
    debug.print(
      "rclcpp_subscription_callback_added",
      subscription,
      callback
    );
#endif
  }
}

void ros_trace_rclcpp_timer_callback_added(const void * timer_handle, const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_timer_callback(timer_handle, callback);

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(timer_handle, callback);
#ifdef DEBUG_OUTPUT
    debug.print(
      "rclcpp_timer_callback_added",
      timer_handle,
      callback
    );
#endif
  }
}

void ros_trace_rclcpp_timer_link_node(const void * timer_handle, const void * node_handle)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  controller.add_timer_handle(node_handle, timer_handle);

  using functionT = void (*)(const void *, const void *);
  if (controller.is_allowed_node(node_handle)) {
    ((functionT) orig_func)(timer_handle, node_handle);
#ifdef DEBUG_OUTPUT
    debug.print(
      "rclcpp_timer_link_node",
      timer_handle,
      node_handle
    );
#endif
  }
}


void ros_trace_callback_start(const void * callback, bool is_intra_process)
{
  static auto & controller = Singleton<TracingController>::get_instance();


  set_callback(callback);
  if (controller.is_allowed_callback(callback)) {

    set_callback_start(trace_clock_read64_monotonic());
    set_is_intra_process(is_intra_process);
#ifdef DEBUG_OUTPUT
    debug.print(
      "callback_start",
      callback,
      is_intra_process
    );
#endif
  }
}

void ros_trace_callback_end(const void * callback)
{
  static auto & controller = Singleton<TracingController>::get_instance();
  // static void * orig_func = dlsym(RTLD_NEXT, __func__);

  unset_callback();
  auto callback_end = trace_clock_read64_monotonic();

  if (controller.is_allowed_callback(callback)) {
    if (get_is_intra_process()) {
      tracepoint(
        TRACEPOINT_PROVIDER,
        intra_callback_duration,
        callback,
        get_callback_start(),
        callback_end,
        get_message_stamp()
      );
    } else {
      tracepoint(
        TRACEPOINT_PROVIDER,
        inter_callback_duration,
        callback,
        get_callback_start(),
        callback_end,
        get_source_stamp(),
        get_message_stamp()
      );
    }

    set_callback_start(0);
    set_source_stamp(0);
    set_message_stamp(0);
    set_is_intra_process(false);

#ifdef DEBUG_OUTPUT
    debug.print(
      "callback_end",
      callback
    );
#endif
  }
}

void ros_trace_dispatch_subscription_callback(
  const void * message,
  const void * callback,
  const uint64_t source_timestamp,
  const uint64_t message_timestamp)
{
  (void) message;
  static auto & controller = Singleton<TracingController>::get_instance();

  if (controller.is_allowed_callback(callback)) {
    set_source_stamp(source_timestamp);
    set_message_stamp(message_timestamp);

#ifdef DEBUG_OUTPUT
    debug.print(
      "dispatch_subscription_callback",
      std::to_string(source_timestamp),
      std::to_string(message_timestamp)
    );
#endif
  }
}

void ros_trace_dispatch_intra_process_subscription_callback(
  const void * message,
  const void * callback,
  const uint64_t message_timestamp)
{
  (void) message;
  static auto & controller = Singleton<TracingController>::get_instance();

  if (controller.is_allowed_callback(callback)) {
    set_message_stamp(message_timestamp);

#ifdef DEBUG_OUTPUT
    debug.print(
      "dispatch_intra_process_subscription_callback",
      message,
      callback,
      message_timestamp
    );
#endif
  }
}


std::unordered_map<void *, std::unordered_set<void *>> recorded_publisher;


void ros_trace_rclcpp_publish(
  const void * publisher_handle,
  const void * message,
  const uint64_t message_timestamp)
{
  (void) message;
  (void) message_timestamp;

  static auto & controller = Singleton<TracingController>::get_instance();
  set_publisher_handle(publisher_handle);

  if (controller.is_allowed_publisher_handle(publisher_handle)) {
    set_rclcpp_publish(trace_clock_read64_monotonic());
    set_message_stamp(message_timestamp);

#ifdef DEBUG_OUTPUT
    auto cb = get_callback();

    auto publisher_handle_ = const_cast<void *>(publisher_handle);
    auto & cb_map = recorded_publisher.find(publisher_handle_)->second;
    if (cb_map.count(cb) == 0 && cb != 0) {
      debug.print(
        "bind_callback_and_publisher",
        cb,
        publisher_handle
      );
      cb_map.insert(cb);
    }
    debug.print(
      "rclcpp_publish",
      publisher_handle,
      message,
      message_timestamp
    );
#endif
  }
}

void ros_trace_rclcpp_intra_publish(
  const void * publisher_handle,
  const void * message,
  const uint64_t message_timestamp)
{
  (void) message;
  (void) message_timestamp;
  static auto & controller = Singleton<TracingController>::get_instance();

  set_publisher_handle(publisher_handle);

  if (controller.is_allowed_publisher_handle(publisher_handle)) {
    tracepoint(
      TRACEPOINT_PROVIDER,
      rclcpp_intra_publish,
      publisher_handle,
      message_timestamp
    );
#ifdef DEBUG_OUTPUT
    debug.print(
      "bind_callback_and_publisher",
      get_callback(),
      publisher_handle
    );
    debug.print(
      "rclcpp_intra_publish",
      publisher_handle,
      message,
      message_timestamp
    );
#endif
  }
}

#ifdef DEBUG_OUTPUT
void ros_trace_rcl_timer_init(
  const void * timer_handle,
  int64_t period)
{
  // TODO(hsgwa): Add filtering of timer initialization using node_handle
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, int64_t);
  ((functionT) orig_func)(timer_handle, period);

  debug.print(
    "rcl_timer_init",
    timer_handle,
    period
  );
}
#endif

void ros_trace_rcl_init(
  const void * context_handle)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *);
  ((functionT) orig_func)(context_handle);

  tracepoint(TRACEPOINT_PROVIDER, rcl_init_caret);

#ifdef DEBUG_OUTPUT
  debug.print(
    "rcl_init", context_handle, LIB_CARET_VERSION
  );
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
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  static auto & controller = Singleton<TracingController>::get_instance();

  controller.add_publisher_handle(node_handle, publisher_handle, topic_name);

  auto publisher_handle_ = const_cast<void *>(publisher_handle);
  recorded_publisher.insert(
    std::make_pair(publisher_handle_, std::unordered_set<void *>())
  );

  using functionT = void (*)(const void *, const void *, const void *, const char *, const size_t);
  // TODO(hsgwa): support topic_name filtering
  // It seems to be executed before the topic name and node name are known.

  ((functionT) orig_func)(
    publisher_handle,
    node_handle,
    rmw_publisher_handle,
    topic_name,
    queue_depth);
#ifdef DEBUG_OUTPUT
  debug.print(
    "rcl_publisher_init",
    publisher_handle,
    node_handle,
    rmw_publisher_handle,
    topic_name,
    queue_depth
  );
#endif
}

void ros_trace_rcl_publish(
  const void * publisher_handle,
  const void * message)
{
  (void) message;
  static auto & controller = Singleton<TracingController>::get_instance();

  if (controller.is_allowed_publisher_handle(publisher_handle)) {
    set_rcl_publish(trace_clock_read64_monotonic());
#ifdef DEBUG_OUTPUT
    debug.print(
      "rcl_publish",
      publisher_handle,
      message
    );
#endif
  }
}

void ros_trace_ring_buffer_enqueue(
  const void * buffer,
  const void * message,
  const size_t size,
  const bool is_full
)
{
  (void) buffer;
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  const auto & publisher_handle = get_publisher_handle();
  static auto & controller = Singleton<TracingController>::get_instance();
  using functionT = void (*)(const void *, const void *, size_t, bool);

  if (controller.is_allowed_publisher_handle(publisher_handle)) {
    ((functionT) orig_func)(buffer, message, size, is_full);
#ifdef DEBUG_OUTPUT
  debug.print(
    "ring_buffer_enqueue",
    buffer,
    message,
    size,
    is_full
  );
#endif
  }
}

#ifdef DEBUG_OUTPUT
void ros_trace_ring_buffer_dequeue(
  const void * buffer,
  const void * message,
  const size_t size
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, size_t);
  ((functionT) orig_func)(buffer, message, size);

  debug.print(
    "ring_buffer_dequeue",
    buffer,
    message,
    size
  );
}
#endif

#ifdef DEBUG_OUTPUT
void ros_trace_ring_buffer_clear(
  const void * buffer
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *);
  ((functionT) orig_func)(buffer);

  debug.print(
    "ring_buffer_clear",
    buffer
  );
}
#endif

#ifdef DEBUG_OUTPUT
void ros_trace_rcl_service_init(
  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, const void *, const char *);
  ((functionT) orig_func)(service_handle, node_handle, rmw_service_handle, service_name);

  std::lock_guard<std::mutex> lock(debug_mutex);
  debug.print(
    "rcl_service_init",
    service_handle,
    node_handle,
    rmw_service_handle,
    service_name);
}
#endif

#ifdef DEBUG_OUTPUT
void ros_trace_rclcpp_service_callback_added(
  const void * service_handle,
  const char * callback)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *);
  ((functionT) orig_func)(service_handle, callback);

  debug.print(
    "rclcpp_service_callback_added",
    service_handle,
    callback
  );
}
#endif

#ifdef DEBUG_OUTPUT
void ros_trace_rcl_client_init(
  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *, const void *, const char *);
  ((functionT) orig_func)(client_handle, node_handle, rmw_client_handle, service_name);

  debug.print("rcl_client_init", client_handle, node_handle, rmw_client_handle, service_name);
}
#endif

void ros_trace_rclcpp_callback_register(
  const void * callback,
  const char * symbol)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & controller = Singleton<TracingController>::get_instance();
  using functionT = void (*)(const void *, const char *);
  if (controller.is_allowed_callback(callback)) {
    ((functionT) orig_func)(callback, symbol);
#ifdef DEBUG_OUTPUT
    debug.print(
      "rclcpp_callback_register",
      callback,
      symbol
    );
#endif
  }
}

#ifdef DEBUG_OUTPUT
void ros_trace_rcl_lifecycle_state_machine_init(
  const void * node_handle,
  const void * state_machine)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const void *);
  ((functionT) orig_func)(node_handle, state_machine);

  debug.print(
    "rcl_lifecycle_state_machine_init",
    node_handle,
    state_machine);
}
#endif

#ifdef DEBUG_OUTPUT
void ros_trace_rcl_lifecycle_transition(
  const void * state_machine,
  const char * start_label,
  const char * goal_label)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(const void *, const char *, const char *);
  ((functionT) orig_func)(state_machine, start_label, goal_label);

  debug.print("rcl_lifecycle_transition", state_machine, start_label, goal_label);
}
#endif

#ifdef DEBUG_OUTPUT
void ros_trace_message_construct(
  const void * original_message,
  const void * constructed_message)
{


  debug.print("message_construct", original_message, constructed_message);
}
#endif
}
