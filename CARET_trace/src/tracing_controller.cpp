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

#include <shared_mutex>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <iomanip>
#include <string>
#include <regex>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rcpputils/get_env.hpp"
#include "caret_trace/tracing_controller.hpp"

#define SELECT_NODES_ENV_NAME "CARET_SELECT_NODES"
#define IGNORE_NODES_ENV_NAME "CARET_IGNORE_NODES"
#define SELECT_TOPICS_ENV_NAME "CARET_SELECT_TOPICS"
#define IGNORE_TOPICS_ENV_NAME "CARET_IGNORE_TOPICS"

bool partial_match(std::unordered_set<std::string> set, std::string target_name)
{
  for (auto & condition : set) {
    try {
      if (condition == "*") {
        return true;
      }
      std::regex re(condition.c_str());
      if (std::regex_search(target_name, re)) {
        return true;
      }
    } catch (std::regex_error & e) {
      // print out error message during initialization phase
    }
  }
  return false;
}

std::unordered_set<std::string> split(std::string str, char del)
{
  std::unordered_set<std::string> result;
  std::string::size_type first = 0;
  std::string::size_type last;

  while (first < str.size()) {
    last = str.find_first_of(del, first);
    std::string subStr(str, first, last - first);
    result.insert(subStr);
    if (last == std::string::npos) {
      break;
    }
    first = last + 1;
  }
  return result;
}

std::string get_env_var(std::string env_var_name)
{
  std::string env_var;
  try {
    env_var = rcpputils::get_env_var(env_var_name.c_str());
  } catch (const std::exception & e) {
    std::cerr << "Failed to get " + env_var_name + " variable" << std::endl;
  }

  if (env_var.empty()) {
    env_var = "";
  }

  return env_var;
}

std::unordered_set<std::string> get_env_vars(std::string env_var_name)
{
  std::string env_var = get_env_var(env_var_name);
  return split(env_var, ':');
}

bool is_condition_valid(std::string condition)
{
  std::string dummy_str = "foo_bar";
  try {
    std::regex re(condition.c_str());
    std::regex_search(dummy_str, re);
    return true;
  } catch (std::regex_error & e) {
    return false;
  }
}

void check_condition_set(std::unordered_set<std::string> conditions)
{
  for (auto & condition : conditions) {
    if (!is_condition_valid(condition)) {
      std::string msg = "Failed to load regular expression \"" + condition + "\". Skip filtering.";
      RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
    }
  }
}

TracingController::TracingController()
: selected_node_names_(get_env_vars(SELECT_NODES_ENV_NAME)),
  ignored_node_names_(get_env_vars(IGNORE_NODES_ENV_NAME)),
  selected_topic_names_(get_env_vars(SELECT_TOPICS_ENV_NAME)),
  ignored_topic_names_(get_env_vars(IGNORE_TOPICS_ENV_NAME)),
  select_enabled_(selected_topic_names_.size() > 0 || selected_node_names_.size() > 0),
  ignore_enabled_(ignored_topic_names_.size() > 0 || ignored_node_names_.size() > 0)
{
  if (select_enabled_ || ignore_enabled_) {
    std::string msg = "trace filtering is enabled.";
    RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
  }
  if (select_enabled_ && ignore_enabled_) {
    std::string msg = "both select and ignore are set. select is used.";
    RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
  }
  if (select_enabled_) {
    if (selected_node_names_.size() > 0) {
      std::string msg = SELECT_NODES_ENV_NAME + std::string(": ") + get_env_var(
        SELECT_NODES_ENV_NAME);
      RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
    }
    if (selected_topic_names_.size() > 0) {
      std::string msg = SELECT_TOPICS_ENV_NAME + std::string(": ") + get_env_var(
        SELECT_TOPICS_ENV_NAME);
      RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
    }
  } else if (ignore_enabled_) {
    if (ignored_node_names_.size() > 0) {
      std::string msg = IGNORE_NODES_ENV_NAME + std::string(": ") + get_env_var(
        IGNORE_NODES_ENV_NAME);
      RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
    }
    if (ignored_topic_names_.size() > 0) {
      std::string msg = IGNORE_TOPICS_ENV_NAME + std::string(": ") + get_env_var(
        IGNORE_TOPICS_ENV_NAME);
      RCLCPP_INFO(rclcpp::get_logger("caret"), msg.c_str());
    }
  }

  check_condition_set(selected_node_names_);
  check_condition_set(ignored_node_names_);
  check_condition_set(selected_topic_names_);
  check_condition_set(ignored_topic_names_);
}

bool TracingController::is_allowed_callback(const void * callback)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_callbacks_.find(callback);
    if (is_allowed_it != allowed_callbacks_.end() ) {
      return is_allowed_it->second;
    }
  }

  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    auto node_name = to_node_name(callback);
    auto topic_name = to_topic_name(callback);

    if (select_enabled_) {
      auto is_selected_topic = partial_match(selected_topic_names_, topic_name);
      auto is_selected_node = partial_match(selected_node_names_, node_name);

      if (selected_topic_names_.size() > 0 && is_selected_topic) {
        allowed_callbacks_.insert(std::make_pair(callback, true));
        return true;
      }
      if (selected_node_names_.size() > 0 && is_selected_node) {
        allowed_callbacks_.insert(std::make_pair(callback, true));
        return true;
      }
      if (selected_node_names_.size() == 0 && topic_name == "") {  // allow timer callback
        allowed_callbacks_.insert(std::make_pair(callback, true));
        return true;
      }

      allowed_callbacks_.insert(std::make_pair(callback, false));
      return false;
    }
    if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (ignored_node_names_.size() > 0 && is_ignored_node) {
        allowed_callbacks_.insert(std::make_pair(callback, false));
        return false;
      }
      if (ignored_topic_names_.size() > 0 && is_ignored_topic) {
        allowed_callbacks_.insert(std::make_pair(callback, false));
        return false;
      }
      allowed_callbacks_.insert(std::make_pair(callback, true));
      return true;
    }
    allowed_callbacks_.insert(std::make_pair(callback, true));
    return true;
  }
}

bool TracingController::is_allowed_node(const void * node_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  auto node_name = node_handle_to_node_names_[node_handle];
  if (select_enabled_ && selected_node_names_.size() > 0) {
    auto is_selected_node = partial_match(selected_node_names_, node_name);
    return is_selected_node;
  } else if (ignore_enabled_ && ignored_node_names_.size() > 0) {
    auto is_ignored_node = partial_match(ignored_node_names_, node_name);
    return !is_ignored_node;
  }
  return true;
}

bool TracingController::is_allowed_subscription_handle(const void * subscription_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  auto node_handle = subscription_handle_to_node_handles_[subscription_handle];
  auto node_name = node_handle_to_node_names_[node_handle];
  auto topic_name = subscription_handle_to_topic_names_[subscription_handle];

  if (select_enabled_) {
    auto is_selected_node = partial_match(selected_node_names_, node_name);
    auto is_selected_topic = partial_match(selected_topic_names_, topic_name);

    if (is_selected_node && selected_node_names_.size() > 0) {
      return true;
    }
    if (is_selected_topic && selected_topic_names_.size() > 0) {
      return true;
    }
    return false;
  } else if (ignore_enabled_) {
    auto is_ignored_node = partial_match(ignored_node_names_, node_name);
    auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

    if (is_ignored_node && ignored_node_names_.size() > 0) {
      return false;
    }
    if (is_ignored_topic && ignored_topic_names_.size() > 0) {
      return false;
    }
    return true;
  }
  return true;
}

bool TracingController::is_allowed_publisher_handle(const void * publisher_handle)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_publishers_.find(publisher_handle);
    if (is_allowed_it != allowed_publishers_.end() ) {
      return is_allowed_it->second;
    }
  }
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    auto node_handle = publisher_handle_to_node_handles_[publisher_handle];
    auto node_name = node_handle_to_node_names_[node_handle];
    auto topic_name = publisher_handle_to_topic_names_[publisher_handle];

    if (select_enabled_) {
      auto is_selected_node = partial_match(selected_node_names_, node_name);
      auto is_selected_topic = partial_match(selected_topic_names_, topic_name);

      if (is_selected_node && selected_node_names_.size() > 0) {
        allowed_publishers_.insert(std::make_pair(publisher_handle, true));
        return true;
      }
      if (is_selected_topic && selected_topic_names_.size() > 0) {
        allowed_publishers_.insert(std::make_pair(publisher_handle, true));
        return true;
      }

      allowed_publishers_.insert(std::make_pair(publisher_handle, false));
      return false;
    } else if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (is_ignored_node && ignored_node_names_.size() > 0) {
        allowed_publishers_.insert(std::make_pair(publisher_handle, false));
        return false;
      }
      if (is_ignored_topic && ignored_topic_names_.size() > 0) {
        allowed_publishers_.insert(std::make_pair(publisher_handle, false));
        return false;
      }
      allowed_publishers_.insert(std::make_pair(publisher_handle, true));
      return true;
    }
    allowed_publishers_.insert(std::make_pair(publisher_handle, true));
    return true;
  }
}

std::string TracingController::to_node_name(const void * callback)
{
  do {
    auto timer_handle_it = callback_to_timer_handles_.find(callback);
    if (timer_handle_it == callback_to_timer_handles_.end()) {
      break;
    }
    auto node_handle_it = timer_handle_to_node_handles_.find(timer_handle_it->second);
    if (node_handle_it == timer_handle_to_node_handles_.end()) {
      break;
    }
    auto node_name_it = node_handle_to_node_names_.find(node_handle_it->second);
    if (node_name_it == node_handle_to_node_names_.end()) {
      break;
    }
    auto node_name = node_name_it->second;

    return node_name;
  } while (false);

  do {
    auto subscription_it = callback_to_subscriptions_.find(callback);
    if (subscription_it == callback_to_subscriptions_.end()) {
      break;
    }

    auto subscription_handle_it =
      subscription_to_subscription_handles_.find(subscription_it->second);
    if (subscription_handle_it == subscription_to_subscription_handles_.end()) {
      break;
    }

    auto node_handle_it =
      subscription_handle_to_node_handles_.find(subscription_handle_it->second);
    if (node_handle_it == subscription_handle_to_node_handles_.end()) {
      break;
    }

    auto node_name_it = node_handle_to_node_names_.find(node_handle_it->second);
    if (node_name_it == node_handle_to_node_names_.end()) {
      break;
    }

    auto node_name = node_name_it->second;
    return node_name;
  } while (false);

  return "";
}

std::string TracingController::to_topic_name(const void * callback)
{
  do {
    auto subscription_it = callback_to_subscriptions_.find(callback);
    if (subscription_it == callback_to_subscriptions_.end()) {
      break;
    }

    auto subscription_handle_it =
      subscription_to_subscription_handles_.find(subscription_it->second);
    if (subscription_handle_it == subscription_to_subscription_handles_.end()) {
      break;
    }
    auto topic_name_it = subscription_handle_to_topic_names_.find(subscription_handle_it->second);
    if (topic_name_it == subscription_handle_to_topic_names_.end()) {
      break;
    }
    auto topic_name = topic_name_it->second;
    return topic_name;
  } while (false);

  return "";
}

void TracingController::add_node(const void * node_handle, std::string node_name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  node_handle_to_node_names_.insert(std::make_pair(node_handle, node_name));
}

void TracingController::add_subscription_handle(
  const void * node_handle, const void * subscription_handle, std::string topic_name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  subscription_handle_to_node_handles_.insert(std::make_pair(subscription_handle, node_handle));
  subscription_handle_to_topic_names_.insert(std::make_pair(subscription_handle, topic_name));
}

void TracingController::add_subscription(
  const void * subscription_handle, const void * subscription)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  subscription_to_subscription_handles_.insert(std::make_pair(subscription, subscription_handle));
}

void TracingController::add_subscription_callback(
  const void * subscription, const void * callback)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  callback_to_subscriptions_.insert(std::make_pair(callback, subscription));
}

void TracingController::add_timer_handle(
  const void * node_handle, const void * timer_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  timer_handle_to_node_handles_.insert(std::make_pair(node_handle, timer_handle));
}

void TracingController::add_timer_callback(
  const void * timer_handle, const void * callback)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  callback_to_timer_handles_.insert(std::make_pair(callback, timer_handle));
}

void TracingController::add_publisher_handle(
  const void * node_handle,
  const void * publisher_handle,
  std::string topic_name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  publisher_handle_to_node_handles_.insert(std::make_pair(publisher_handle, node_handle));
  publisher_handle_to_topic_names_.insert(std::make_pair(publisher_handle, topic_name));
}
