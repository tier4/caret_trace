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

#include "caret_trace/data_recorder.hpp"

#include "caret_trace/recordable_data.hpp"
#include "caret_trace/singleton.hpp"

#include <cassert>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

DataRecorder::DataRecorder()
{
}

DataRecorder::DataRecorder(std::initializer_list<std::shared_ptr<RecordableDataInterface>> data)
: DataRecorder(std::vector<std::shared_ptr<RecordableDataInterface>>(data))
{
}

DataRecorder::DataRecorder(std::vector<std::shared_ptr<RecordableDataInterface>> data)
{
  for (auto recordable : data) {
    iterables_.emplace_back(recordable);
  }
}

void DataRecorder::record_next_one()
{
  auto & iter = get_iterating();
  iter.record_next_one();
}

bool DataRecorder::is_recording() const
{
  for (auto & it : iterables_) {
    if (it->is_recording()) {
      return true;
    }
  }
  return false;
}

void DataRecorder::reset()
{
  for (auto & it : iterables_) {
    it->reset();
  }
}

std::vector<std::string> DataRecorder::trace_points() const
{
  std::vector<std::string> trace_points_;
  for (auto it : iterables_) {
    trace_points_.emplace_back(it->trace_point());
  }
  return trace_points_;
}

bool DataRecorder::finished() const
{
  for (auto & it : iterables_) {
    if (!it->finished()) {
      return false;
    }
  }
  return true;
}

size_t DataRecorder::size() const
{
  size_t size = 0;
  for (auto & it : iterables_) {
    size += it->size();
  }
  return size;
}

void DataRecorder::start()
{
  for (auto & it : iterables_) {
    it->start();
  }
}

RecordableDataInterface & DataRecorder::get_iterating()
{
  for (auto & it : iterables_) {
    if (it->finished()) {
      continue;
    }
    return *it;
  }
  return Singleton<DummyRecordableKeysSet>::get_instance();
}

RecordableDataInterface & DataRecorder::get_next_iter()
{
  for (auto & it : iterables_) {
    if (!it->finished()) {
      return *it;
    }
  }
  return Singleton<DummyRecordableKeysSet>::get_instance();
}
