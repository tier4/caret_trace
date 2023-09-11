// Copyright 2022 Research Institute of Systems Planning, Inc.
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

#include "caret_trace/context.hpp"

#include "caret_trace/clock.hpp"
#include "caret_trace/data_container.hpp"
#include "caret_trace/lttng_session.hpp"
#include "caret_trace/trace_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>

Context::Context()
: Context(std::make_shared<DataContainer>(), std::make_shared<TracingController>())
{
}

Context::Context(
  std::shared_ptr<DataContainer> data_container, std::shared_ptr<TracingController> controller)
: is_node_initializing(false),
  data_container_(data_container),
  controller_(controller),
  node_(nullptr),
  lttng_(std::make_shared<LttngSessionImpl>()),
  clock_(std::make_shared<Clock>())
{
}

TracingController & Context::get_controller()
{
  assert(controller_ != nullptr);
  return *controller_;
}

std::shared_ptr<DataContainer> Context::get_data_container_ptr()
{
  return data_container_;
}

std::shared_ptr<LttngSession> Context::get_lttng_session_ptr()
{
  return lttng_;
}

DataContainer & Context::get_data_container()
{
  return *data_container_;
}

TraceNodeInterface & Context::get_node()
{
  assert(is_node_assigned());
  return *node_;
}

Clock & Context::get_clock()
{
  assert(clock_ != nullptr);
  return *clock_;
}

bool Context::is_recording_allowed() const
{
  if (is_node_assigned()) {
    return node_->is_recording_allowed();
  }

  // NOTE: Prohibited to suppress DISCARDED.
  return false;
}

void Context::assign_node(std::shared_ptr<TraceNodeInterface> node)
{
  node_ = node;
}

bool Context::is_node_assigned() const
{
  return static_cast<bool>(node_);
}
