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

class TraceNodeInterface
{
public:
  virtual ~TraceNodeInterface() {}
  virtual bool is_recording_allowed() const = 0;
  virtual bool is_timer_running() const = 0;
  virtual DataContainerInterface & get_data_container() = 0;
  virtual const TRACE_STATUS & get_status() const = 0;
};

class TraceNode : public rclcpp::Node, public TraceNodeInterface
{
public:
  TraceNode(std::string node_name_base, std::shared_ptr<DataContainer> data_container);

  // for test
  TraceNode(
    std::string node_names, std::shared_ptr<LttngSession> lttng_session,
    std::shared_ptr<DataContainerInterface> data_container,
    rclcpp::Logger::Level level = rclcpp::Logger::Level::Info, bool use_log = false);

  ~TraceNode();

  bool is_recording_allowed() const override;
  bool is_timer_running() const override;
  void timer_callback();
  void start_callback(caret_msgs::msg::Start::UniquePtr msg);
  void end_callback(caret_msgs::msg::End::UniquePtr msg);

  DataContainerInterface & get_data_container() override;

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
};

#endif  // CARET_TRACE__TRACE_NODE_HPP_
#define CARET_TRACE__TRACE_NODE_HPP_
