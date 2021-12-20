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
#include <unordered_set>
#include <unordered_map>
#include <string>

class TracingController
{
public:
  TracingController();
  void add_node(const void * node_handle, std::string node_name);

  void add_subscription_handle(
    const void * node_handle, const void * subscription_handle, std::string topic_name);

  void add_subscription(
    const void * subscription_handle, const void * subscription);

  void add_subscription_callback(
    const void * subscription, const void * callback);

  void add_timer_handle(
    const void * node_handle, const void * timer_handle);

  void add_publisher_handle(
    const void * node_handle, const void * publisher_handle, std::string topic_name);

  void add_timer_callback(
    const void * timer_handle, const void * callback);

  bool is_allowed_callback(const void * callback);

  bool is_allowed_node(const void * node_handle);

  bool is_allowed_publisher_handle(const void * publisher_handle);

  bool is_allowed_subscription_handle(const void * subscription_handle);

private:
  std::shared_timed_mutex mutex_;
  std::string to_node_name(const void * callback);
  std::string to_topic_name(const void * callback);

  const std::unordered_set<std::string> selected_node_names_;
  const std::unordered_set<std::string> ignored_node_names_;
  const std::unordered_set<std::string> selected_topic_names_;
  const std::unordered_set<std::string> ignored_topic_names_;

  const bool select_enabled_;
  const bool ignore_enabled_;

  std::unordered_map<const void *, const void *> subscription_handle_to_node_handles_;
  std::unordered_map<const void *, std::string> subscription_handle_to_topic_names_;
  std::unordered_map<const void *, const void *> subscription_to_subscription_handles_;
  std::unordered_map<const void *, const void *> callback_to_subscriptions_;

  std::unordered_map<const void *, std::string> node_handle_to_node_names_;
  std::unordered_map<const void *, const void *> callback_to_timer_handles_;
  std::unordered_map<const void *, const void *> timer_handle_to_node_handles_;
  std::unordered_map<const void *, bool> allowed_callbacks_;

  std::unordered_map<const void *, const void *> publisher_handle_to_node_handles_;
  std::unordered_map<const void *, std::string> publisher_handle_to_topic_names_;
  std::unordered_map<const void *, bool> allowed_publishers_;
};

#endif  // CARET_TRACE__TRACING_CONTROLLER_HPP_
#define CARET_TRACE__TRACING_CONTROLLER_HPP_
