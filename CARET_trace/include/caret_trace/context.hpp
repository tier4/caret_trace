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

#ifndef CARET_TRACE__CONTEXT_HPP_

#include <memory>
#include <atomic>

#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/trace_node.hpp"


class Context final
{
public:
  Context();
  Context(std::shared_ptr<DataContainer> data_container, std::shared_ptr<TracingController>);

  Context(const Context &) = delete;

  std::shared_ptr<DataContainer> get_data_container_ptr();
  DataContainer & get_data_container();
  TraceNodeInterface & get_node();
  TracingController & get_controller();
  void assign_node(std::shared_ptr<TraceNodeInterface> node);
  bool is_node_assigned() const;
  bool is_recording_enabled() const;
  std::atomic<bool> is_node_initializing;

private:
  std::shared_ptr<DataContainer> data_container_;
  std::shared_ptr<TraceNodeInterface> node_;
  std::shared_ptr<TracingController> controller_;
};

#endif  // CARET_TRACE__CONTEXT_HPP_
#define CARET_TRACE__CONTEXT_HPP_
