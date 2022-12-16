
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

#include "caret_trace/container_traits.hpp"
#include "caret_trace/recordable_data.hpp"

#include <functional>
#include <initializer_list>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#ifndef CARET_TRACE__DATA_RECORDER_HPP_

/// @brief Class that manages the recording of all trace data.
class DataRecorder
{
public:
  /// @brief Construct an instance.
  DataRecorder();

  /// @brief Construct an instance.
  /// @param data Data to be record controlled.
  explicit DataRecorder(std::initializer_list<std::shared_ptr<RecordableDataInterface>> data);

  /// @brief Construct an instance.
  /// @param data Data to be record controlled.
  explicit DataRecorder(std::vector<std::shared_ptr<RecordableDataInterface>> data);

  /// @brief Check whether recording has finished.
  /// @return True if recording has finished, false otherwise.
  bool finished() const;

  /// @brief Check whether recording is ongoing.
  /// @return True if recording is ongoing, false otherwise.
  bool is_recording() const;

  /// @brief Get data size.
  /// @return Total number of recorded data.
  size_t size() const;

  /// @brief Get trace point names.
  /// @return trace point names.
  std::vector<std::string> trace_points() const;

  /// @brief Move to start state.
  void start();

  /// @brief Record next data.
  void record_next_one();

  /// @brief Reset recording status.
  void reset();

private:
  RecordableDataInterface & get_iterating();
  RecordableDataInterface & get_next_iter();
  void next_();

  std::vector<std::shared_ptr<RecordableDataInterface>> iterables_;
};

#endif  // CARET_TRACE__DATA_RECORDER_HPP_
#define CARET_TRACE__DATA_RECORDER_HPP_
