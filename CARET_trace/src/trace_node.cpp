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

#include <string>
#include <memory>
#include <chrono>

#include "caret_trace/trace_node.hpp"
#include "caret_trace/lttng_session.hpp"
#include "caret_msgs/msg/end.hpp"
#include "caret_msgs/msg/start.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

CaretTraceNode::CaretTraceNode(
  std::string node_name,
  std::shared_ptr<LttngSession> lttng_session,
  std::shared_ptr<DataContainerInterface> data_container)
: rclcpp::Node(node_name, rclcpp::NodeOptions()),
  status_(TRACE_STATUS::INIT_MEASURE),
  data_container_(data_container)
{
  if (!lttng_session->is_session_running()) {
    status_ = TRACE_STATUS::WAIT;
    RCLCPP_INFO(get_logger(), "Transitioned to wait status.");
  } else {
    RCLCPP_INFO(get_logger(), "Transitioned to init measure status.");
  }

  start_sub_ = create_subscription<caret_msgs::msg::Start>(
    "/caret/start_record", 10,
    std::bind(&CaretTraceNode::start_callback, this, _1));

  end_sub_ = create_subscription<caret_msgs::msg::End>(
    "/caret/end_record", 10,
    std::bind(&CaretTraceNode::end_callback, this, _1)
  );

  timer_ = create_wall_timer(1s, std::bind(&CaretTraceNode::timer_callback, this));
  timer_->cancel();
  RCLCPP_INFO(get_logger(), "CARET Trace Node initialized.");
}


CaretTraceNode::CaretTraceNode(std::string node_name, std::shared_ptr<DataContainer> data_container)
: CaretTraceNode(
    node_name,
    std::make_shared<LttngSessionImpl>(),
    data_container
)
{
}

CaretTraceNode::~CaretTraceNode()
{
}

DataContainerInterface & CaretTraceNode::get_data_container()
{
  return *data_container_;
}

void CaretTraceNode::run_timer()
{
  RCLCPP_DEBUG(get_logger(), "CARET Trace Node started timer.");
  timer_->reset();
}

void CaretTraceNode::stop_timer()
{
  RCLCPP_DEBUG(get_logger(), "CARET Trace Node stopped timer.");
  timer_->cancel();
}

bool CaretTraceNode::is_recording_allowed() const
{
  return status_ == TRACE_STATUS::INIT_MEASURE ||
         status_ == TRACE_STATUS::MEASURE;
}


const TRACE_STATUS & CaretTraceNode::get_status() const
{
  return status_;
}

void CaretTraceNode::start_callback(caret_msgs::msg::Start::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "CARET Trace Node received start message.");

  bool is_wait_to_prepare_transition = status_ == TRACE_STATUS::WAIT;
  status_ = TRACE_STATUS::PREPARE;
  RCLCPP_INFO(get_logger(), "Transitioned to prepare status.");

  if (is_wait_to_prepare_transition) {
    run_timer();
  }
}

bool CaretTraceNode::is_timer_running() const
{
  return !timer_->is_canceled();
}

void CaretTraceNode::timer_callback()
{
  RCLCPP_DEBUG(get_logger(), "CARET Trace Node fired timer callback.");

  auto record_finished = data_container_->record(1);
  if (record_finished) {
    status_ = TRACE_STATUS::MEASURE;
    RCLCPP_INFO(get_logger(), "Transitioned to measure status.");

    stop_timer();
  }
}

void CaretTraceNode::end_callback(caret_msgs::msg::End::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "CARET Trace Node receivred end message.");

  status_ = TRACE_STATUS::WAIT;
  RCLCPP_INFO(get_logger(), "Transitioned to wait status.");

  stop_timer();
}
