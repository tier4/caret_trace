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
#include <unistd.h>
#include <utility>

#include "caret_trace/trace_node.hpp"
#include "caret_trace/lttng_session.hpp"
#include "caret_msgs/msg/end.hpp"
#include "caret_msgs/msg/start.hpp"
#include "caret_msgs/msg/status.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

TraceNode::TraceNode(
  std::string node_name_base,
  std::shared_ptr<LttngSession> lttng_session,
  std::shared_ptr<DataContainerInterface> data_container,
  rclcpp::Logger::Level level)
: rclcpp::Node(TraceNode::get_unique_node_name(node_name_base), rclcpp::NodeOptions()),
  status_(TRACE_STATUS::UNINITIALIZED),
  record_block_size_(100),
  data_container_(data_container)
{
  set_log_level(level);

  start_sub_ = create_subscription<caret_msgs::msg::Start>(
    "/caret/start_record", 10,
    std::bind(&TraceNode::start_callback, this, _1));

  end_sub_ = create_subscription<caret_msgs::msg::End>(
    "/caret/end_record", 10,
    std::bind(&TraceNode::end_callback, this, _1)
  );

  status_pub_ = create_publisher<caret_msgs::msg::Status>(
    "/caret/status", 10
  );

  timer_ = create_wall_timer(100ms, std::bind(&TraceNode::timer_callback, this));
  timer_->cancel();

  if (!lttng_session->is_session_running()) {
    status_ = TRACE_STATUS::WAIT;
    RCLCPP_INFO(get_logger(), "No active LTTng session exists.");
    RCLCPP_DEBUG(get_logger(), "Transitioned to WAIT status.");
  } else {
    status_ = TRACE_STATUS::RECORD;
    RCLCPP_INFO(get_logger(), "Active LTTng session exists.");
    RCLCPP_DEBUG(get_logger(), "Transitioned to RECORD status.");
  }

  assert(status_ != TRACE_STATUS::UNINITIALIZED);

  publish_status(status_);

  RCLCPP_DEBUG(get_logger(), "Initialized.");
}


TraceNode::TraceNode(std::string node_name, std::shared_ptr<DataContainer> data_container)
: TraceNode(
    node_name,
    std::make_shared<LttngSessionImpl>(),
    data_container
)
{
}

TraceNode::~TraceNode()
{
}

std::string TraceNode::get_unique_node_name(std::string base_name)
{
  auto pid = getpid();
  char * pid_str = nullptr;
  if (asprintf(&pid_str, "%jd", (intmax_t) pid) != -1) {
    base_name += "_" + std::string(pid_str);
    free(pid_str);
  }
  return base_name;
}

DataContainerInterface & TraceNode::get_data_container()
{
  return *data_container_;
}

void TraceNode::run_timer()
{
  RCLCPP_DEBUG(get_logger(), "Started recording timer .");
  timer_->reset();
  timer_->execute_callback();
}

void TraceNode::set_log_level(rclcpp::Logger::Level level)
{
  get_logger().set_level(level);
}

void TraceNode::stop_timer()
{
  RCLCPP_DEBUG(get_logger(), "Stopped recording timer.");
  timer_->cancel();
}

bool TraceNode::is_recording_allowed() const
{
  return status_ == TRACE_STATUS::RECORD;
}


const TRACE_STATUS & TraceNode::get_status() const
{
  return status_;
}

void TraceNode::publish_status(TRACE_STATUS status) const
{
  auto msg = std::make_unique<caret_msgs::msg::Status>();
  msg->node_name = get_name();
  msg->status = static_cast<int>(status);
  status_pub_->publish(std::move(msg));
}

void TraceNode::start_callback(caret_msgs::msg::Start::UniquePtr msg)
{
  (void) msg;

  RCLCPP_DEBUG(get_logger(), "Received start message.");

  bool is_wait_to_prepare_transition = status_ == TRACE_STATUS::WAIT;
  status_ = TRACE_STATUS::PREPARE;

  publish_status(status_);
  RCLCPP_DEBUG(get_logger(), "Transitioned to PREPARE status.");

  data_container_->reset();

  if (is_wait_to_prepare_transition) {
    record_block_size_ = msg->recording_frequency / 10;  // 100ms timer: 10Hz
    if (record_block_size_ <= 0) {
      record_block_size_ = 1;
    }
    run_timer();
  }
}

bool TraceNode::is_timer_running() const
{
  return !timer_->is_canceled();
}

void TraceNode::timer_callback()
{
  auto record_finished = data_container_->record(record_block_size_);
  if (record_finished) {
    status_ = TRACE_STATUS::RECORD;

    publish_status(status_);
    RCLCPP_DEBUG(get_logger(), "Transitioned to RECORD status.");

    stop_timer();
  }
}

void TraceNode::end_callback(caret_msgs::msg::End::UniquePtr msg)
{
  (void) msg;

  status_ = TRACE_STATUS::WAIT;

  publish_status(status_);
  RCLCPP_DEBUG(get_logger(), "Transitioned to WAIT status.");

  stop_timer();
}
