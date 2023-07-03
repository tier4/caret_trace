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

#ifndef CARET_TRACE__RECORDABLE_DATA_HPP_

#include "caret_trace/keys_set.hpp"
#include "caret_trace/singleton.hpp"

#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_set>

/// @brief Interface class for RecordableData.
/// @details New data is stored pending during recording.
class RecordableDataInterface
{
public:
  virtual ~RecordableDataInterface() {}

  /// @brief Move on to recording state.
  virtual void start() = 0;

  /// @brief Check whether recording is finished.
  /// @return True if recording is finished, false otherwise.
  virtual bool finished() const = 0;

  /// @brief Check whether recording is ongoing.
  /// @return True if recording is ongoing, false otherwise.
  virtual bool is_recording() const = 0;

  /// @brief Record next data.
  virtual void record_next_one() = 0;

  /// @brief Reset recording status.
  virtual void reset() = 0;

  /// @brief Get size.
  /// @return data size.
  virtual size_t size() const = 0;

  /// @brief Get pending data size.
  /// @return pending data size.
  virtual size_t pending_size() const = 0;

  /// @brief Get trace point name.
  /// @return trace point name.
  virtual const std::string & trace_point() const = 0;
};

/// @brief Dummy class for RecordableData. This class is used instead of None.
class DummyRecordableKeysSet : public RecordableDataInterface
{
public:
  DummyRecordableKeysSet() : trace_point_("dummy_recordable_keys_set") {}

  bool finished() const override { return true; }

  void start() override {}

  bool is_recording() const override { return true; }

  void record_next_one() override {}

  void reset() override {}

  const std::string & trace_point() const override { return trace_point_; }

  size_t size() const override { return 0; }

  size_t pending_size() const override { return 0; }

private:
  const std::string trace_point_;
};

/// @brief Data container class with sequential recording API.
/// @tparam ...Args Trace point data types.
template <typename... Args>
class RecordableData : public RecordableDataInterface
{
private:
  using KeyT = HashableKeys<Args...>;

public:
  using FuncT = void(Args...);
  using StdFuncT = std::function<void(Args...)>;

  /// @brief Construct an instance.
  /// @param trace_point Trace point name for this instance.
  explicit RecordableData(std::string trace_point)
  : it_(nullptr), func_(nullptr), is_iterating_(false), trace_point_(trace_point)
  {
  }

  /// @brief Construct an instance.
  /// @param trace_point Trace point name for this instance.
  explicit RecordableData(char * trace_point) : RecordableData(std::string(trace_point)) {}

  ~RecordableData() override {}

  /// @brief Assign recording function.
  /// @param function recording function.
  void assign(StdFuncT function)
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    func_ = function;
  }

  void start() override
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    reset_();

    it_ = set_.begin();
    is_iterating_ = true;  // transition to PREPARE state
    is_end_iterator__ = is_end_iterator_();
  }

  /// @brief Store new data.
  /// @param data to store.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  bool store(Args... args)
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    if (is_iterating_) {
      // PREPARE state
      pending_set_.insert(args...);
      return true;
    } else {
      // OTHER state
      set_.insert(args...);
      return false;
    }
  }

  void reset() override
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    reset_();
  }

  /// @brief Check whether recording function is assigned.
  /// @return True if recording function is assigned, false otherwise.
  bool is_assigned() const
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return static_cast<bool>(func_);
  }

  size_t size() const override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return set_.size();
  }

  size_t pending_size() const override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return pending_set_.size();
  }

  bool finished() const override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);  // read lock

    return is_end_iterator__;
  }

  bool is_recording() const override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return is_iterating_;
  }

  void record_next_one() override
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    record_one_();
    next_();
    try_merge_pending_data();
  }

  /// @brief Det iterating data.
  /// @return Current iterating data.
  const KeyT & get() const
  {
    assert(!finished());

    std::shared_lock<std::shared_mutex> lock(mutex_);
    return *it_;
  }

  const std::string & trace_point() const override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return trace_point_;
  }

private:
  using SetT = KeysSet<Args...>;
  using ConstIteratorT = typename KeysSet<Args...>::ConstIteratorT;

  void merge_pending_keys()
  {
    for (auto & key : pending_set_) {
      set_.insert(key);
    }
    pending_set_.clear();
  }

  /// @brief merge_pending_data when iterator reached to the end.
  /// @return true : Succeed to merge.
  /// @return false : No data were merged.
  bool try_merge_pending_data()
  {
    if (is_end_iterator__) {
      merge_pending_keys();
      is_iterating_ = false;  // transition to OTHER state
      return true;
    }
    return false;
  }

  void record_one_() const
  {
    if (is_end_iterator_()) {
      return;
    }

    constexpr std::size_t arg_size = sizeof...(Args);

    auto & keys = *it_;
    if (!func_) {
      return;
    }

    if constexpr (arg_size == 1) {
      const auto & first = keys.first();
      func_(first);                        // func_1_ is not assigned.
    } else if constexpr (arg_size == 2) {  // NOLINT
      // NOLINT is necessary for old cpplint.
      // see: https://github.com/cpplint/cpplint/issues/135
      const auto & first = keys.first();
      const auto & second = keys.second();
      func_(first, second);
    } else if constexpr (arg_size == 3) {  // NOLINT
      const auto & first = keys.first();
      const auto & second = keys.second();
      const auto & third = keys.third();
      func_(first, second, third);
    } else if constexpr (arg_size == 4) {  // NOLINT
      const auto & first = keys.first();
      const auto & second = keys.second();
      const auto & third = keys.third();
      const auto & fourth = keys.fourth();
      func_(first, second, third, fourth);
    } else if constexpr (arg_size == 5) {  // NOLINT
      const auto & first = keys.first();
      const auto & second = keys.second();
      const auto & third = keys.third();
      const auto & fourth = keys.fourth();
      const auto & fifth = keys.fifth();
      func_(first, second, third, fourth, fifth);
    } else {
      const auto & first = keys.first();
      const auto & second = keys.second();
      const auto & third = keys.third();
      const auto & fourth = keys.fourth();
      const auto & fifth = keys.fifth();
      const auto & sixth = keys.sixth();
      func_(first, second, third, fourth, fifth, sixth);
    }
  }

  void next_()
  {
    if (!is_end_iterator_()) {
      ++it_;
    }
    is_end_iterator__ = is_end_iterator_();
  }

  void reset_()
  {
    is_iterating_ = false;  // transition to OTHER state
    merge_pending_keys();
    is_end_iterator__ = false;
  }

  inline bool is_end_iterator_() const
  {
    auto is_end = it_ == set_.end();
    return is_end;
  }

  ConstIteratorT it_;
  SetT pending_set_;
  SetT set_;
  StdFuncT func_;
  bool is_iterating_;
  // True if the iterator has reached to the end, otherwise False. False except during iterating.
  bool is_end_iterator__;
  const std::string trace_point_;

  mutable std::shared_mutex mutex_;
};

#endif  // CARET_TRACE__RECORDABLE_DATA_HPP_
#define CARET_TRACE__RECORDABLE_DATA_HPP_
