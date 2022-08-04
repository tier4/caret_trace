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

#ifndef CARET_TRACE__TRQACE_NODE_HPP_

#include <utility>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "caret_msgs/msg/start.hpp"
#include "caret_msgs/msg/end.hpp"
#include "caret_trace/lttng_session.hpp"
#include "caret_trace/data_container.hpp"

enum class TRACE_STATUS
{
  INIT_MEASURE,
  WAIT,
  PREPARE,
  MEASURE,
};


class CaretTraceNodeInterface
{
public:
  virtual ~CaretTraceNodeInterface() {}
  virtual bool is_recording_allowed() const = 0;
  virtual bool is_timer_running() const = 0;
  virtual DataContainerInterface & get_data_container() = 0;
  virtual const TRACE_STATUS & get_status() const = 0;
};


class CaretTraceNode : public rclcpp::Node, public CaretTraceNodeInterface
{
public:
  CaretTraceNode(std::string node_name, std::shared_ptr<DataContainer> data_container);

  // for test
  CaretTraceNode(
    std::string node_names,
    std::shared_ptr<LttngSession> lttng_session,
    std::shared_ptr<DataContainerInterface> data_container
  );

  ~CaretTraceNode();

  bool is_recording_allowed() const override;
  bool is_timer_running() const override;
  void timer_callback();
  void start_callback(caret_msgs::msg::Start::UniquePtr msg);
  void end_callback(caret_msgs::msg::End::UniquePtr msg);

  DataContainerInterface & get_data_container() override;

  const TRACE_STATUS & get_status() const override;

private:
  TRACE_STATUS status_;
  // rclcpp::Logger logger_;
  void run_timer();
  void stop_timer();

  rclcpp::Subscription<caret_msgs::msg::Start>::SharedPtr start_sub_;
  rclcpp::Subscription<caret_msgs::msg::End>::SharedPtr end_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DataContainerInterface> data_container_;
};

#endif  // CARET_TRACE__TRACE_NODE_HPP_
#define CARET_TRACE__TRQACE_NODE_HPP_
