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
// limitations under the License.#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>

#include "caret_trace/context.hpp"
#include "caret_trace/trace_node.hpp"


Context::Context()
: Context(std::make_shared<DataContainer>())
{
}


Context::Context(std::shared_ptr<DataContainer> data_container)
: data_container_(data_container)
{
}

std::shared_ptr<DataContainer> Context::get_data_container_ptr()
{
  return data_container_;
}

DataContainer & Context::get_data_container()
{
  return *data_container_;
}


CaretTraceNodeInterface & Context::get_node()
{
  assert(is_node_assigned());
  return *node_;
}

bool Context::is_recording_enabled() const
{
  if (is_node_assigned()) {
    return node_->is_recording_allowed();
  }
  return false;
}

void Context::assign_node(std::shared_ptr<CaretTraceNodeInterface> node)
{
  node_ = node;
}

bool Context::is_node_assigned() const
{
  return static_cast<bool>(node_);
}
