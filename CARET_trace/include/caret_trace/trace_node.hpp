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

#ifndef CARET_TRACE__TRACE_NODE_HPP_

#include "caret_trace/data_container.hpp"
#include "caret_trace/lttng_session.hpp"
#include "rclcpp/rclcpp.hpp"

#include "caret_msgs/msg/end.hpp"
#include "caret_msgs/msg/start.hpp"
#include "caret_msgs/msg/status.hpp"

#include <memory>
#include <string>
#include <utility>

enum class TRACE_STATUS {
  UNINITIALIZED,
  WAIT,
  PREPARE,
  RECORD,
};

/// @brief Interface class for recording operation node.
class TraceNodeInterface
{
public:
  virtual ~TraceNodeInterface() {}

  /// @brief Check whther current status allows recording.
  /// @return True if recording is allowed, false otherwise.
  virtual bool is_recording_allowed() const = 0;

  /// @brief Check whther current status allows recording. For initialization trace points.
  /// @return True if recording is allowed, false otherwise.
  virtual bool is_recording_allowed_init() const = 0;

  /// @brief Check if recording timer is running.
  /// @return True if timer is running, false otherwise.
  virtual bool is_timer_running() const = 0;

  /// @brief Get DataContainer instance.
  /// @return DataContainer instance.
  virtual DataContainerInterface & get_data_container() = 0;

  /// @brief Get current recording status.
  /// @return recording status.
  virtual const TRACE_STATUS & get_status() const = 0;
};

/// @brief Implementation class for recording operation node.
class TraceNode : public rclcpp::Node, public TraceNodeInterface
{
public:
  /// @brief Construct an instance.
  /// @param node_name_base Base node name. The node name is [node_name_base]_[pid].
  /// @param lttng_session Instance of lttng session
  /// @param data_container Instance of data container.
  /// @param level Log level.
  /// @param use_log Flag to toggle log use.
  /// @param execute_timer_on_run If True, timer_callback is executed after start_callback.
  TraceNode(
    std::string node_name_base, rclcpp::NodeOptions options,
    std::shared_ptr<LttngSession> lttng_session,
    std::shared_ptr<DataContainerInterface> data_container,
    rclcpp::Logger::Level level = rclcpp::Logger::Level::Info, bool use_log = false,
    bool execute_timer_on_run = true);

  ~TraceNode();

  /// @brief Check whther current status allows recording.
  /// @return True if recording is allowed, false otherwise.
  bool is_recording_allowed() const override;

  /// @brief Check whther current status allows recording. For initialization trace points.
  /// @return True if recording is allowed, false otherwise.
  bool is_recording_allowed_init() const override;

  /// @brief Check if recording timer is running.
  /// @return True if timer is running, false otherwise.
  bool is_timer_running() const override;

  /// @brief Timer callback for recording.
  void timer_callback();

  /// @brief Subscription callback for start message.
  /// @param msg start message.
  void start_callback(caret_msgs::msg::Start::UniquePtr msg);

  /// @brief Subscription callback for end message
  /// @param msg end message
  void end_callback(caret_msgs::msg::End::UniquePtr msg);

  /// @brief Get data container instance.
  /// @return data container.
  DataContainerInterface & get_data_container() override;

  /// @brief Get status.
  /// @return Current recording status.
  const TRACE_STATUS & get_status() const override;

private:
  TRACE_STATUS status_;
  uint64_t record_block_size_;
  const bool use_log_;  // for test

  static std::string get_unique_node_name(std::string base_name);
  void run_timer();
  void stop_timer();
  void set_log_level(rclcpp::Logger::Level level);
  void publish_status(TRACE_STATUS status) const;

  void debug(std::string message) const;
  void info(std::string message) const;

  rclcpp::Subscription<caret_msgs::msg::Start>::SharedPtr start_sub_;
  rclcpp::Subscription<caret_msgs::msg::End>::SharedPtr end_sub_;
  rclcpp::Publisher<caret_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DataContainerInterface> data_container_;
  bool execute_timer_on_run_;

  mutable std::shared_mutex mutex_;
};

#endif  // CARET_TRACE__TRACE_NODE_HPP_
#define CARET_TRACE__TRACE_NODE_HPP_
