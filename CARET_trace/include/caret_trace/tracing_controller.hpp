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

#ifndef CARET_TRACE__TRACING_CONTROLLER_HPP_

#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

/// @brief Classes that implement trace filtering functions such as CARET_SELECT_NODES.
/// @details Reads the environment variables CARET_SELECT_NODES,
/// CARET_IGNORE_NODES, CARET_SELECT_TOPICS, and CARET_IGNORE_TOPICS.
/// Tracepoints related to the specified node name or topic name are filtered
/// so that they are not recorded by LTTng.
/// If both SELECT and IGNORE are specified, SELECT takes priority.
/// @remark Tracepoints for which node names cannot be obtained,
/// such as tracepoints without a DDS layer, are currently not filtered.
class TracingController
{
public:
  /// @brief Construct an instance.
  explicit TracingController(bool use_log = true);

  /// @brief Register node name for rcl_node_init tracepoint hook.
  /// @param node_handle Address of the node handle.
  /// @param node_name Node name.
  void add_node(const void * node_handle, std::string node_name);

  /// @brief Register topic name for rcl_subscription_init hook.
  /// @param node_handle Address of the node handle.
  /// @param subscription_handle Address of the subscription handle.
  /// @param topic_name topic name.
  void add_subscription_handle(
    const void * node_handle, const void * subscription_handle, std::string topic_name);

  /// @brief Register topic name for rcl_subscription_init hook.
  /// @param node_handle Address of the node handle.
  /// @param rmw_subscription_handle Address of the rmw_subscription handle.
  /// @param topic_name topic name.
  void add_rmw_subscription_handle(
    const void * node_handle, const void * rmw_subscription_handle, std::string topic_name);

  /// @brief Register binding information for rclcpp_subscription_init tracepoint hook.
  /// @param subscription_handle Address of the subscription handle.
  /// @param subscription Address of subscription instance.
  void add_subscription(const void * subscription_handle, const void * subscription);

  /// @brief Register binding information for rclcpp_subscription_callback_added tracepoint hook.
  /// @param subscription Address of subscription instance.
  /// @param callback Address of callback instance.
  void add_subscription_callback(const void * subscription, const void * callback);

  /// @brief Register binding information for rclcpp_timer_link_node tracepoint hook.
  /// @param node_handle Address of the node handle.
  /// @param timer_handle Address of the timer handle.
  void add_timer_handle(const void * node_handle, const void * timer_handle);

  /// @brief Register topic name for ros_trace_rcl_publisher_init
  /// @param node_handle  Address of the node handle.
  /// @param publisher_handle  Address of the publisher handle.
  /// @param topic_name Topic name.
  void add_publisher_handle(
    const void * node_handle, const void * publisher_handle, std::string topic_name);

  /// @brief Register binding information for rclcpp_timer_callback_added tracepoint hook.
  /// @param timer_handle Address of the timer handle.
  /// @param callback Address of callback instance.
  void add_timer_callback(const void * timer_handle, const void * callback);

  /// @brief Register binding information for rclcpp_buffer_to_ipb tracepoint.
  /// @param buffer  Address of the buffer.
  /// @param ipb  Address of the IntraProcessBuffer.
  void add_buffer(const void * buffer, const void * ipb);

  /// @brief Register binding information for rclcpp_ipb_to_subscription tracepoint.
  /// @param ipb  Address of the IntraProcessBuffer.
  /// @param subscription  Address of the subscription instance.
  void add_ipb(const void * ipb, const void * subscription);

  /// @brief Check if trace point is a enabled callback
  /// @param callback
  /// @param callback Address of callback instance.
  /// @return True if the callback is enabled, false otherwise.
  bool is_allowed_callback(const void * callback);

  /// @brief Check if trace point is a enabled node
  /// @param node_handle  Address of the node handle.
  /// @return True if the node is enabled, false otherwise.
  bool is_allowed_node(const void * node_handle);

  /// @brief Check if trace point is a enabled publisher
  /// @param publisher_handle  Address of the publisher handle.
  /// @return True if the publisher is enabled, false otherwise.
  bool is_allowed_publisher_handle(const void * publisher_handle);

  /// @brief Check if trace point is a enabled subscription
  /// @param subscription_handle Address of the subscription handle.
  /// @return True if the subscription is enabled, false otherwise.
  bool is_allowed_subscription_handle(const void * subscription_handle);

  /// @brief Check if trace point is a enabled subscription
  /// @param rmw_subscription_handle Address of the rmw_subscription handle.
  /// @return True if the rmw_subscription is enabled, false otherwise.
  bool is_allowed_rmw_subscription_handle(const void * rmw_subscription_handle);

  /// @brief Check if trace point is a enabled subscription
  /// @param buffer Address of the intra-process buffer.
  /// @return True if the buffer is enabled, false otherwise.
  bool is_allowed_buffer(const void * buffer);

private:
  void debug(std::string message) const;
  void info(std::string message) const;

  std::shared_timed_mutex mutex_;
  std::string to_node_name(const void * callback);
  std::string to_topic_name(const void * callback);

  const std::unordered_set<std::string> selected_node_names_;
  const std::unordered_set<std::string> ignored_node_names_;
  const std::unordered_set<std::string> selected_topic_names_;
  const std::unordered_set<std::string> ignored_topic_names_;

  const bool select_enabled_;
  const bool ignore_enabled_;

  const bool use_log_;  // for test

  std::unordered_map<const void *, const void *> subscription_handle_to_node_handles_;
  std::unordered_map<const void *, std::string> subscription_handle_to_topic_names_;
  std::unordered_map<const void *, const void *> subscription_to_subscription_handles_;
  std::unordered_map<const void *, const void *> callback_to_subscriptions_;

  std::unordered_map<const void *, const void *> rmw_subscription_handle_to_node_handles_;
  std::unordered_map<const void *, std::string> rmw_subscription_handle_to_topic_names_;
  std::unordered_map<const void *, bool> allowed_rmw_subscription_handles_;

  std::unordered_map<const void *, std::string> node_handle_to_node_names_;
  std::unordered_map<const void *, const void *> callback_to_timer_handles_;
  std::unordered_map<const void *, const void *> timer_handle_to_node_handles_;
  std::unordered_map<const void *, bool> allowed_callbacks_;

  std::unordered_map<const void *, const void *> publisher_handle_to_node_handles_;
  std::unordered_map<const void *, std::string> publisher_handle_to_topic_names_;
  std::unordered_map<const void *, bool> allowed_publishers_;

  std::unordered_map<const void *, const void *> buffer_to_ipbs_;
  std::unordered_map<const void *, const void *> ipb_to_subscriptions_;
  std::unordered_map<const void *, bool> allowed_buffers_;
};

#endif  // CARET_TRACE__TRACING_CONTROLLER_HPP_
#define CARET_TRACE__TRACING_CONTROLLER_HPP_
