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

#ifndef CARET_TRACE__RECORDABLE_KEYS_SET_HPP_

#include <iostream>
#include <functional>
#include <unordered_set>
#include <type_traits>
#include <iterator>
#include <cassert>
#include <string>
#include <shared_mutex>

#include "caret_trace/keys_set.hpp"
#include "caret_trace/singleton.hpp"


class RecordableDataInterface
{
public:
  virtual ~RecordableDataInterface() {}
  virtual void start() = 0;
  virtual bool finished() const = 0;
  virtual bool is_recording() const = 0;
  virtual void record_next_one() = 0;
  virtual void reset() = 0;
  virtual size_t size() const = 0;
  virtual size_t pending_size() const = 0;
  virtual const std::string & trace_point() const = 0;
};


//  Dummy class instead of None.
class DummyRecordableKeysSet : public RecordableDataInterface
{
public:
  DummyRecordableKeysSet()
  : trace_point_("dummy_recordable_keys_set")
  {
  }

  bool finished() const override
  {
    return true;
  }

  void start() override
  {
  }

  bool is_recording() const override
  {
    return true;
  }

  void record_next_one() override
  {
  }

  void reset() override
  {
  }

  const std::string & trace_point() const override
  {
    return trace_point_;
  }

  size_t size() const override
  {
    return 0;
  }

  size_t pending_size() const override
  {
    return 0;
  }

private:
  const std::string trace_point_;
};

template<typename ... Args>
class RecordableData : public RecordableDataInterface
{
private:
  using KeyT = HashableKeys<Args...>;

public:
  using FuncT = void (Args ...);
  using StdFuncT = std::function<void (Args...)>;

  explicit RecordableData(std::string trace_point)
  : it_(nullptr), func_(nullptr), is_iterating_(false), trace_point_(trace_point)
  {
  }

  explicit RecordableData(char * trace_point_)
  : RecordableData(std::string(trace_point_))
  {
  }

  ~RecordableData() override
  {
  }

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
    is_iterating_ = true;
    is_end__ = is_end_iterator_();
  }

  bool store(Args... args)
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    if (is_iterating_) {
      pending_set_.insert(args ...);
      return true;
    } else {
      set_.insert(args ...);
      return false;
    }
  }

  void reset() override
  {
    std::lock_guard<std::shared_mutex> lock(mutex_);

    reset_();
  }

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
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return is_end__;
  }

  bool is_recording() const override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return is_iterating_;
  }

  void record_next_one() override
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    record_one_();
    next_();
    try_merge_pending_data();
  }

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

  /**
   * @brief merge_pending_data when iterator reached to the end.
   *
   * @return true : Succeed to merge.
   * @return false : No data were merged.
   */
  bool try_merge_pending_data()
  {
    if (is_end__) {
      merge_pending_keys();
      is_iterating_ = false;
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
      func_(first);  // func_1_ is not assined.
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
    } else {
      const auto & first = keys.first();
      const auto & second = keys.second();
      const auto & third = keys.third();
      const auto & fourth = keys.fourth();
      const auto & fifth = keys.fifth();
      func_(first, second, third, fourth, fifth);
    }
  }

  void next_()
  {
    if (!is_end_iterator_()) {
      ++it_;
    }
    is_end__ = is_end_iterator_();
  }

  void reset_()
  {
    is_iterating_ = false;
    merge_pending_keys();
    is_end__ = false;
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
  bool is_end__;
  const std::string trace_point_;

  mutable std::shared_mutex mutex_;
};

#endif  // CARET_TRACE__RECORDABLE_KEYS_SET_HPP_
#define CARET_TRACE__RECORDABLE_KEYS_SET_HPP_
