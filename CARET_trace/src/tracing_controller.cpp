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

#include "caret_trace/tracing_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/get_env.hpp"

#include <dlfcn.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "caret_trace/DEBUG.hpp"

#define SELECT_NODES_ENV_NAME "CARET_SELECT_NODES"
#define IGNORE_NODES_ENV_NAME "CARET_IGNORE_NODES"
#define SELECT_TOPICS_ENV_NAME "CARET_SELECT_TOPICS"
#define IGNORE_TOPICS_ENV_NAME "CARET_IGNORE_TOPICS"
#define IGNORE_PROCESSES_ENV_NAME "CARET_IGNORE_PROCESSES"

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

void check_condition_set(std::unordered_set<std::string> conditions, bool use_log)
{
  for (auto & condition : conditions) {
    if (use_log && !is_condition_valid(condition)) {
      std::string message =
        "Failed to load regular expression \"" + condition + "\". Skip filtering.";
      RCLCPP_INFO(rclcpp::get_logger("caret"), message.c_str());
    }
  }
}

TracingController::TracingController(bool use_log)
: selected_node_names_(get_env_vars(SELECT_NODES_ENV_NAME)),
  ignored_node_names_(get_env_vars(IGNORE_NODES_ENV_NAME)),
  selected_topic_names_(get_env_vars(SELECT_TOPICS_ENV_NAME)),
  ignored_topic_names_(get_env_vars(IGNORE_TOPICS_ENV_NAME)),
  ignored_process_names_(get_env_vars(IGNORE_PROCESSES_ENV_NAME)),
  select_enabled_(selected_topic_names_.size() > 0 || selected_node_names_.size() > 0),
  ignore_enabled_(ignored_topic_names_.size() > 0 || ignored_node_names_.size() > 0),
  use_log_(use_log)
{
  if (select_enabled_ || ignore_enabled_) {
    info("trace filtering is enabled.");
  }
  if (select_enabled_ && ignore_enabled_) {
    info("both select and ignore are set. select is used.");
  }
  if (select_enabled_) {
    if (selected_node_names_.size() > 0) {
      info(SELECT_NODES_ENV_NAME + std::string(": ") + get_env_var(SELECT_NODES_ENV_NAME));
    }
    if (selected_topic_names_.size() > 0) {
      info(SELECT_TOPICS_ENV_NAME + std::string(": ") + get_env_var(SELECT_TOPICS_ENV_NAME));
    }
  } else if (ignore_enabled_) {
    if (ignored_node_names_.size() > 0) {
      info(IGNORE_NODES_ENV_NAME + std::string(": ") + get_env_var(IGNORE_NODES_ENV_NAME));
    }
    if (ignored_topic_names_.size() > 0) {
      info(IGNORE_TOPICS_ENV_NAME + std::string(": ") + get_env_var(IGNORE_TOPICS_ENV_NAME));
    }
  }

  if (ignored_process_names_.size() > 0) {
    info(IGNORE_PROCESSES_ENV_NAME + std::string(": ") + get_env_var(IGNORE_PROCESSES_ENV_NAME));
  }

  is_ignored_process_ =
    ignored_process_names_.size() > 0 &&
    partial_match(ignored_process_names_, std::string(program_invocation_short_name));

  check_condition_set(selected_node_names_, use_log);
  check_condition_set(ignored_node_names_, use_log);
  check_condition_set(selected_topic_names_, use_log);
  check_condition_set(ignored_topic_names_, use_log);
  check_condition_set(ignored_process_names_, use_log);
}

void TracingController::debug(std::string message) const
{
  if (use_log_) {
    RCLCPP_DEBUG(rclcpp::get_logger("caret"), message.c_str());
  }
}
void TracingController::info(std::string message) const
{
  if (use_log_) {
    RCLCPP_INFO(rclcpp::get_logger("caret"), message.c_str());
  }
}

bool TracingController::is_allowed_callback(const void * callback)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_callbacks_.find(callback);
    if (is_allowed_it != allowed_callbacks_.end()) {
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
        allowed_callbacks_[callback] = true;
        return true;
      }
      if (selected_node_names_.size() > 0 && is_selected_node) {
        allowed_callbacks_[callback] = true;
        return true;
      }
      if (selected_node_names_.size() == 0 && topic_name == "") {  // allow timer callback
        allowed_callbacks_[callback] = true;
        return true;
      }

      allowed_callbacks_[callback] = false;
      return false;
    }
    if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (ignored_node_names_.size() > 0 && is_ignored_node) {
        allowed_callbacks_[callback] = false;
        return false;
      }
      if (ignored_topic_names_.size() > 0 && is_ignored_topic) {
        allowed_callbacks_[callback] = false;
        return false;
      }
      allowed_callbacks_[callback] = true;
      return true;
    }
    allowed_callbacks_[callback] = true;
    return true;
  }
}

bool TracingController::is_allowed_node(const void * node_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  if (allowed_node_handle_.count(node_handle) > 0) {
    auto allowed = allowed_node_handle_[node_handle];
    auto ss = (allowed)? "--- ALLOW ": "--- NOT ALLOW ";
    std::cout << ss << node_handle << std::endl;
    return allowed;
  }
  if (node_handle_to_node_names_.count(node_handle) > 0) {
    auto node_name = node_handle_to_node_names_[node_handle];
    std::cout << "--- NODE " << node_handle << ": " << node_name << std::endl;

    if (select_enabled_ && selected_node_names_.size() > 0) {
      auto is_selected_node = partial_match(selected_node_names_, node_name);
      allowed_node_handle_[node_handle] = is_selected_node;
      return is_selected_node;
    } else if (ignore_enabled_ && ignored_node_names_.size() > 0) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      allowed_node_handle_[node_handle] = !is_ignored_node;
      return !is_ignored_node;
    }
    allowed_node_handle_[node_handle] = true;
    return true;
  }
  std::cout << "--- UNREG NODE " << node_handle << std::endl;
  return true;
}

bool TracingController::is_allowed_subscription_handle(const void * subscription_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  if (subscription_handle_to_node_handles_.count(subscription_handle) <= 0) {
    return true;
  }
  auto node_handle = subscription_handle_to_node_handles_[subscription_handle];
  if (node_handle_to_node_names_.count(node_handle) <= 0) {
    return true;
  }
  auto node_name = node_handle_to_node_names_[node_handle];
  if (subscription_handle_to_topic_names_.count(subscription_handle) <= 0) {
    return true;
  }
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

bool TracingController::is_allowed_rmw_subscription_handle(const void * rmw_subscription_handle)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_rmw_subscription_handles_.find(rmw_subscription_handle);
    if (is_allowed_it != allowed_rmw_subscription_handles_.end()) {
      return is_allowed_it->second;
    }
  }

  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if (rmw_subscription_handle_to_node_handles_.count(rmw_subscription_handle) <= 0) {
      return true;
    }
    auto node_handle = rmw_subscription_handle_to_node_handles_[rmw_subscription_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto node_name = node_handle_to_node_names_[node_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto topic_name = rmw_subscription_handle_to_topic_names_[rmw_subscription_handle];

    if (select_enabled_) {
      auto is_selected_topic = partial_match(selected_topic_names_, topic_name);
      auto is_selected_node = partial_match(selected_node_names_, node_name);

      if (selected_topic_names_.size() > 0 && is_selected_topic) {
        allowed_rmw_subscription_handles_[rmw_subscription_handle] = true;
        return true;
      }
      if (selected_node_names_.size() > 0 && is_selected_node) {
        allowed_rmw_subscription_handles_[rmw_subscription_handle] = true;
        return true;
      }

      allowed_rmw_subscription_handles_[rmw_subscription_handle] = false;
      return false;
    }
    if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (ignored_node_names_.size() > 0 && is_ignored_node) {
        allowed_rmw_subscription_handles_[rmw_subscription_handle] = false;
        return false;
      }
      if (ignored_topic_names_.size() > 0 && is_ignored_topic) {
        allowed_rmw_subscription_handles_[rmw_subscription_handle] = false;
        return false;
      }
      allowed_rmw_subscription_handles_[rmw_subscription_handle] = true;
      return true;
    }
    allowed_rmw_subscription_handles_[rmw_subscription_handle] = true;
    return true;
  }
}

bool TracingController::is_allowed_publisher_handle(const void * publisher_handle)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_publishers_.find(publisher_handle);
    if (is_allowed_it != allowed_publishers_.end()) {
      return is_allowed_it->second;
    }
  }
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if (publisher_handle_to_node_handles_.count(publisher_handle) <= 0) {
      return true;
    }
    auto node_handle = publisher_handle_to_node_handles_[publisher_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto node_name = node_handle_to_node_names_[node_handle];
    if (publisher_handle_to_topic_names_.count(publisher_handle) <= 0) {
      return true;
    }
    auto topic_name = publisher_handle_to_topic_names_[publisher_handle];

    if (select_enabled_) {
      auto is_selected_node = partial_match(selected_node_names_, node_name);
      auto is_selected_topic = partial_match(selected_topic_names_, topic_name);

      if (is_selected_node && selected_node_names_.size() > 0) {
        allowed_publishers_[publisher_handle] = true;
        return true;
      }
      if (is_selected_topic && selected_topic_names_.size() > 0) {
        allowed_publishers_[publisher_handle] = true;
        return true;
      }

      allowed_publishers_[publisher_handle] = false;
      return false;
    } else if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (is_ignored_node && ignored_node_names_.size() > 0) {
        allowed_publishers_[publisher_handle] = false;
        return false;
      }
      if (is_ignored_topic && ignored_topic_names_.size() > 0) {
        allowed_publishers_[publisher_handle] = false;
        return false;
      }
      allowed_publishers_[publisher_handle] = true;
      return true;
    }
    allowed_publishers_[publisher_handle] = true;
    return true;
  }
}

bool TracingController::is_allowed_buffer(const void * buffer)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_buffers_.find(buffer);
    if (is_allowed_it != allowed_buffers_.end()) {
      return is_allowed_it->second;
    }
  }

  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if (buffer_to_ipbs_.count(buffer) <= 0) {
      return true;
    }
    auto ipb = buffer_to_ipbs_[buffer];
    if (ipb_to_subscriptions_.count(ipb) <= 0) {
      return true;
    }
    auto subscription = ipb_to_subscriptions_[ipb];
    if (subscription_to_subscription_handles_.count(ipb) <= 0) {
      return true;
    }
    auto subscription_handle = subscription_to_subscription_handles_[subscription];
    if (subscription_handle_to_node_handles_.count(subscription_handle) <= 0) {
      return true;
    }
    auto node_handle = subscription_handle_to_node_handles_[subscription_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto node_name = node_handle_to_node_names_[node_handle];
    if (subscription_handle_to_topic_names_.count(subscription_handle) <= 0) {
      return true;
    }
    auto topic_name = subscription_handle_to_topic_names_[subscription_handle];

    if (select_enabled_) {
      auto is_selected_topic = partial_match(selected_topic_names_, topic_name);
      auto is_selected_node = partial_match(selected_node_names_, node_name);

      if (selected_topic_names_.size() > 0 && is_selected_topic) {
        allowed_buffers_[buffer] = true;
        return true;
      }
      if (selected_node_names_.size() > 0 && is_selected_node) {
        allowed_buffers_[buffer] = true;
        return true;
      }

      allowed_buffers_[buffer] = false;
      return false;
    }
    if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (ignored_node_names_.size() > 0 && is_ignored_node) {
        allowed_buffers_[buffer] = false;
        return false;
      }
      if (ignored_topic_names_.size() > 0 && is_ignored_topic) {
        allowed_buffers_[buffer] = false;
        return false;
      }
      allowed_buffers_[buffer] = true;
      return true;
    }
    allowed_buffers_[buffer] = true;
    return true;
  }
}

bool TracingController::is_allowed_process()
{
  return !is_ignored_process_;
}

bool TracingController::is_allowed_timer_handle(const void * timer_handle)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  if (allowed_timer_handle_.count(timer_handle) > 0) {
    auto allowed = allowed_timer_handle_[timer_handle];
    return allowed;
  }
  if (timer_handle_to_node_handles_.count(timer_handle) > 0) {
    auto node_handle = timer_handle_to_node_handles_[timer_handle];
    if (node_handle_to_node_names_.count(node_handle) > 0) {
      auto node_name = node_handle_to_node_names_[node_handle];
      if (select_enabled_) {
        auto is_selected_node = partial_match(selected_node_names_, node_name);
        if (is_selected_node && selected_node_names_.size() > 0) {
          allowed_timer_handle_[timer_handle] = true;
          return true;
        }
      }
      if (ignore_enabled_) {
        auto is_ignored_node = partial_match(ignored_node_names_, node_name);
        if (is_ignored_node && ignored_node_names_.size() > 0) {
          allowed_timer_handle_[timer_handle] = false;
          return false;
        }
      }
      allowed_node_handle_[node_handle] = true;
    }
  }
  D(timer_handle)
  return true;
}

bool TracingController::is_allowed_state_machine(const void * state_machine)
{
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  auto node_handle = state_machine_to_node_handles_[state_machine];
  return is_allowed_node(node_handle);
}

bool TracingController::is_allowed_ipb(const void * ipb)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_ipbs.find(ipb);
    if (is_allowed_it != allowed_ipbs.end()) {
      return is_allowed_it->second;
    }
  }

  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if (ipb_to_subscriptions_.count(ipb) <= 0) {
      return true;
    }
    auto subscription = ipb_to_subscriptions_[ipb];
    if (ipb_to_subscriptions_.count(subscription) <= 0) {
      return true;
    }
    auto subscription_handle = subscription_to_subscription_handles_[subscription];
    if (subscription_handle_to_node_handles_.count(subscription_handle) <= 0) {
      return true;
    }
    auto node_handle = subscription_handle_to_node_handles_[subscription_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto node_name = node_handle_to_node_names_[node_handle];
    if (subscription_handle_to_topic_names_.count(subscription_handle) <= 0) {
      return true;
    }
    auto topic_name = subscription_handle_to_topic_names_[subscription_handle];

    if (select_enabled_) {
      auto is_selected_topic = partial_match(selected_topic_names_, topic_name);
      auto is_selected_node = partial_match(selected_node_names_, node_name);

      if (selected_topic_names_.size() > 0 && is_selected_topic) {
        allowed_ipbs[ipb] = true;
        return true;
      }
      if (selected_node_names_.size() > 0 && is_selected_node) {
        allowed_ipbs[ipb] = true;
        return true;
      }

      allowed_ipbs[ipb] = false;
      return false;
    }
    if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);
      auto is_ignored_topic = partial_match(ignored_topic_names_, topic_name);

      if (ignored_node_names_.size() > 0 && is_ignored_node) {
        allowed_ipbs[ipb] = false;
        return false;
      }
      if (ignored_topic_names_.size() > 0 && is_ignored_topic) {
        allowed_ipbs[ipb] = false;
        return false;
      }
      allowed_ipbs[ipb] = true;
      return true;
    }
    allowed_ipbs[ipb] = true;
    return true;
  }
}

bool TracingController::is_allowed_service_handle(const void * service_handle)
{
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_service_handle_.find(service_handle);
    if (is_allowed_it != allowed_service_handle_.end()) {
      return is_allowed_it->second;
    }
  }

  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if (service_handle_to_node_handles_.count(service_handle) <= 0) {
      return true;
    }
    auto node_handle = service_handle_to_node_handles_[service_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto node_name = node_handle_to_node_names_[node_handle];

    if (select_enabled_) {
      auto is_selected_node = partial_match(selected_node_names_, node_name);

      if (is_selected_node && selected_node_names_.size() > 0) {
        allowed_service_handle_[service_handle] = true;
        return true;
      }

      allowed_service_handle_[service_handle] = false;
      return false;
    } else if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);

      if (is_ignored_node && ignored_node_names_.size() > 0) {
        allowed_service_handle_[service_handle] = false;
        return false;
      }
      allowed_service_handle_[service_handle] = true;
      return true;
    }
    allowed_service_handle_[service_handle] = true;
    return true;
  }
}

bool TracingController::is_allowed_client_handle(const void * client_handle) {
  std::unordered_map<const void *, bool>::iterator is_allowed_it;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    is_allowed_it = allowed_client_handle_.find(client_handle);
    if (is_allowed_it != allowed_client_handle_.end()) {
      return is_allowed_it->second;
    }
  }
  
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    if (client_handle_to_node_handles_.count(client_handle) <= 0) {
      return true;
    }
    auto node_handle = client_handle_to_node_handles_[client_handle];
    if (node_handle_to_node_names_.count(node_handle) <= 0) {
      return true;
    }
    auto node_name = node_handle_to_node_names_[node_handle];

    if (select_enabled_) {
      auto is_selected_node = partial_match(selected_node_names_, node_name);

      if (is_selected_node && selected_node_names_.size() > 0) {
        allowed_client_handle_[client_handle] = true;
        return true;
      }

      allowed_client_handle_[client_handle] = false;
      return false;
    } else if (ignore_enabled_) {
      auto is_ignored_node = partial_match(ignored_node_names_, node_name);

      if (is_ignored_node && ignored_node_names_.size() > 0) {
        allowed_client_handle_[client_handle] = false;
        return false;
      }
      allowed_client_handle_[client_handle] = true;
      return true;
    }
    allowed_client_handle_[client_handle] = true;
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

    auto node_handle_it = subscription_handle_to_node_handles_.find(subscription_handle_it->second);
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

  node_handle_to_node_names_[node_handle] = node_name;
}

void TracingController::add_subscription_handle(
  const void * node_handle, const void * subscription_handle, std::string topic_name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  subscription_handle_to_node_handles_[subscription_handle] = node_handle;
  subscription_handle_to_topic_names_[subscription_handle] = topic_name;
}

void TracingController::add_rmw_subscription_handle(
  const void * node_handle, const void * rmw_subscription_handle, std::string topic_name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  rmw_subscription_handle_to_node_handles_[rmw_subscription_handle] = node_handle;
  rmw_subscription_handle_to_topic_names_[rmw_subscription_handle] = topic_name;
}

void TracingController::add_subscription(
  const void * subscription_handle, const void * subscription)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  subscription_to_subscription_handles_[subscription] = subscription_handle;
}

void TracingController::add_subscription_callback(const void * subscription, const void * callback)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  callback_to_subscriptions_[callback] = subscription;
}

void TracingController::add_timer_handle(const void * timer_handle, const void * node_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  timer_handle_to_node_handles_[timer_handle] = node_handle;
}

void TracingController::add_timer_callback(const void * timer_handle, const void * callback)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);

  callback_to_timer_handles_[callback] = timer_handle;
}

void TracingController::add_publisher_handle(
  const void * node_handle, const void * publisher_handle, std::string topic_name)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  publisher_handle_to_node_handles_[publisher_handle] = node_handle;
  publisher_handle_to_topic_names_[publisher_handle] = topic_name;
}

void TracingController::add_buffer(const void * buffer, const void * ipb)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  buffer_to_ipbs_[buffer] = ipb;
}

void TracingController::add_ipb(const void * ipb, const void * subscription)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  ipb_to_subscriptions_[ipb] = subscription;
}

void TracingController::add_state_machine(const void * state_machine, const void * node_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  state_machine_to_node_handles_[state_machine] = node_handle;
}

void TracingController::add_service_handle(const void * service_handle, const void * node_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  service_handle_to_node_handles_[service_handle] = node_handle;
}

void TracingController::add_client_handle(const void * client_handle, const void * node_handle)
{
  std::lock_guard<std::shared_timed_mutex> lock(mutex_);
  client_handle_to_node_handles_[client_handle] = node_handle;
}

std::string TracingController::get_node_name(const std::string type, const void * key) {
    if (type == "NH") {
      if (node_handle_to_node_names_.count(key) > 0) {
        return node_handle_to_node_names_[key];
      }
    } else if (type == "SH") {
      if (service_handle_to_node_handles_.count(key) > 0) {
        auto nh = service_handle_to_node_handles_[key];
        return node_handle_to_node_names_[nh];
      }
    } else if (type == "CH") {
      if (client_handle_to_node_handles_.count(key) > 0) {
        auto nh = client_handle_to_node_handles_[key];
        return node_handle_to_node_names_[nh];
      }
    } else if (type == "CB") {
      if (callback_to_timer_handles_.count(key) > 0) {
        auto th = callback_to_timer_handles_[key];
        if (timer_handle_to_node_handles_.count(key) > 0) {
          auto nh = timer_handle_to_node_handles_[th];
          return node_handle_to_node_names_[nh];
        }
      } else if (callback_to_subscriptions_.count(key)) {
        auto sub = callback_to_subscriptions_[key];
        if (subscription_to_subscription_handles_.count(sub)) {
          auto subh = subscription_to_subscription_handles_[sub];
          if (subscription_handle_to_node_handles_.count(subh)) {
            auto nh = subscription_handle_to_node_handles_[subh];
            return node_handle_to_node_names_[nh];
          }
        }
      }
    } else if (type == "SM") {
      if (buffer_to_ipbs_.count(key) > 0) {
        auto ipb = buffer_to_ipbs_[key];
        if (ipb_to_subscriptions_.count(ipb)) {
          auto sub = ipb_to_subscriptions_[ipb];
          if (subscription_to_subscription_handles_.count(sub)) {
            auto subh = subscription_to_subscription_handles_[sub];
            if (subscription_handle_to_node_handles_.count(subh)) {
              auto nh = subscription_handle_to_node_handles_[subh];
              return node_handle_to_node_names_[nh];
            }
          }
        }
      }
    } else if (type == "IPB") {
      if (ipb_to_subscriptions_.count(key)) {
        auto sub = ipb_to_subscriptions_[key];
        if (subscription_to_subscription_handles_.count(sub)) {
          auto subh = subscription_to_subscription_handles_[sub];
          if (subscription_handle_to_node_handles_.count(subh)) {
            auto nh = subscription_handle_to_node_handles_[subh];
            return node_handle_to_node_names_[nh];
          }
        }
      }
    }
    return std::string("- NOTHING -");
  }