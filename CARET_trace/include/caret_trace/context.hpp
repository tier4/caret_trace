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

#include "caret_trace/clock.hpp"
#include "caret_trace/data_container.hpp"
#include "caret_trace/trace_node.hpp"
#include "caret_trace/tracing_controller.hpp"

#include <atomic>
#include <memory>

/// @brief A class for singletons that manage shared instances.
class Context final
{
public:
  /// @brief Construct an instance.
  Context();

  /// @brief Construct an instance.
  /// @param data_container data container.
  /// @param controller controller.
  Context(
    std::shared_ptr<DataContainer> data_container, std::shared_ptr<TracingController> controller);

  Context(const Context &) = delete;

  /// @brief Get TracingController instance.
  /// @return TracingController instance
  TracingController & get_controller();

  /// @brief Get pointer for DataContainer instance.
  /// @return DataContainer pointer
  std::shared_ptr<DataContainer> get_data_container_ptr();

  /// @brief Get pointer for LttngSession instance.
  /// @return LttngSession pointer.
  std::shared_ptr<LttngSession> get_lttng_session_ptr();

  /// @brief Get DataContainer instance.
  /// @return DataContainer instance
  DataContainer & get_data_container();

  /// @brief Get TraceNodeInterface instance.
  /// @return TraceNodeInterface instance.
  TraceNodeInterface & get_node();

  /// @brief Get Clock instance.
  /// @return clock instance.
  Clock & get_clock();

  /// @brief Assign node instance.
  /// @param node Node instance to be assigned.
  void assign_node(std::shared_ptr<TraceNodeInterface> node);

  /// @brief Check whether node is assigned.
  /// @return True is node is assigned, false otherwise.
  bool is_node_assigned() const;

  /// @brief Check whether current status enables recording.
  /// @return True is recording is enabled, false otherwise.
  bool is_recording_allowed() const;

  /// @brief Initializing flag for TraceNode.
  std::atomic<bool> is_node_initializing;

private:
  std::shared_ptr<DataContainer> data_container_;
  std::shared_ptr<TracingController> controller_;
  std::shared_ptr<TraceNodeInterface> node_;
  std::shared_ptr<LttngSession> lttng_;
  std::shared_ptr<Clock> clock_;
};

#endif  // CARET_TRACE__CONTEXT_HPP_
#define CARET_TRACE__CONTEXT_HPP_
