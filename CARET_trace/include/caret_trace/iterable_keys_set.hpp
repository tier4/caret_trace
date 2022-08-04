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

#ifndef CARET_TRACE__ITERABLE_KEYS_SET_HPP_

#include <iostream>
#include <functional>
#include <unordered_set>
#include <type_traits>
#include <iterator>
#include <cassert>

#include "caret_trace/keys_set.hpp"
#include "caret_trace/function_traits.hpp"
#include "caret_trace/singleton.hpp"


class IterableKeysSetBase
{
public:
  virtual ~IterableKeysSetBase() {}
  virtual void begin() = 0;
  virtual void next() = 0;
  virtual bool is_end() = 0;
  virtual bool is_iterating() = 0;
  virtual void record_once() = 0;
  virtual void cancel() = 0;
  virtual size_t size() = 0;
};


//  Dummy class instead of None.
class DummyIterableKeysSet : public IterableKeysSetBase
{
public:
  void begin() {}
  void next() {}
  bool is_end()
  {
    return true;
  }
  bool is_iterating()
  {
    return true;
  }
  void record_once()
  {
  }
  void cancel()
  {
  }
  size_t size()
  {
    return 0;
  }
};

template<
  typename T1,
  typename T2 = std::false_type,
  typename T3 = std::false_type,
  typename T4 = std::false_type
>
class IterableKeysSet : public IterableKeysSetBase
{
private:
  using KeyT = HashableKeys<T1, T2, T3, T4>;
  using FuncT_1 = void (T1);
  using FuncT_2 = void (T1, T2);
  using FuncT_3 = void (T1, T2, T3);
  using FuncT_4 = void (T1, T2, T3, T4);

public:
  IterableKeysSet()
  : iterating_(false)
  {
  }

  ~IterableKeysSet()
  {
  }

  void assign(std::function<FuncT_1> record_function)
  {
    static_assert(std::is_same_v<T2, std::false_type>, "Invalid function arguments");
    static_assert(std::is_same_v<T3, std::false_type>, "Invalid function arguments");
    static_assert(std::is_same_v<T4, std::false_type>, "Invalid function arguments");

    func_1_ = record_function;
  }
  void assign(std::function<FuncT_2> record_function)
  {
    static_assert(!std::is_same_v<T2, std::false_type>, "Invalid function arguments");
    static_assert(std::is_same_v<T3, std::false_type>, "Invalid function arguments");
    static_assert(std::is_same_v<T4, std::false_type>, "Invalid function arguments");

    func_2_ = record_function;
  }
  void assign(std::function<FuncT_3> record_function)
  {
    static_assert(!std::is_same_v<T2, std::false_type>, "Invalid function arguments");
    static_assert(!std::is_same_v<T3, std::false_type>, "Invalid function arguments");
    static_assert(std::is_same_v<T4, std::false_type>, "Invalid function arguments");

    func_3_ = record_function;
  }
  void assign(std::function<FuncT_4> record_function)
  {
    static_assert(!std::is_same_v<T2, std::false_type>, "Invalid function arguments");
    static_assert(!std::is_same_v<T3, std::false_type>, "Invalid function arguments");
    static_assert(!std::is_same_v<T4, std::false_type>, "Invalid function arguments");

    func_4_ = record_function;
  }

  bool is_assigned() const
  {
    if constexpr (std::is_same_v<T2, std::false_type>) {
      return static_cast<bool>(func_1_);
    } else if constexpr (std::is_same_v<T3, std::false_type>) {
      return static_cast<bool>(func_2_);
    } else if constexpr (std::is_same_v<T4, std::false_type>) {
      return static_cast<bool>(func_3_);
    } else {
      return static_cast<bool>(func_4_);
    }
  }

  size_t size()
  {
    return set_.size();
  }

  void begin()
  {
    it_ = set_.begin();
    iterating_ = true;
  }

  void next()
  {
    if (!is_end()) {
      ++it_;
      if (is_end()) {
        for (auto & key : pending_set_) {
          set_.insert(key);
        }
      }
    }
  }

  bool is_end()
  {
    return is_end_(it_);
  }

  bool is_iterating()
  {
    return iterating_;
  }

  void record_once()
  {
    if (is_end()) {
      return;
    }

    if constexpr (std::is_same_v<T2, std::false_type>) {
      auto & keys = *it_;
      auto & first = keys.first();
      assert(func_1_);
      func_1_(first);  // func_1_ is not assined.
    } else if constexpr (std::is_same_v<T3, std::false_type>) {  // NOLINT
      // NOLINT is necessary for old cpplint.
      // see: https://github.com/cpplint/cpplint/issues/135
      auto & keys = *it_;
      auto & first = keys.first();
      auto & second = keys.second();
      assert(func_2_);  // func_2_ is not assined.
      func_2_(first, second);
    } else if constexpr (std::is_same_v<T4, std::false_type>) {  // NOLINT
      auto & keys = *it_;
      auto & first = keys.first();
      auto & second = keys.second();
      auto & third = keys.third();
      assert(func_3_);  // func_3_ is not assined.
      func_3_(first, second, third);
    } else {
      auto & keys = *it_;
      auto & first = keys.first();
      auto & second = keys.second();
      auto & third = keys.third();
      auto & fourth = keys.fourth();
      assert(func_4_);  // func_4_ is not assined.
      func_4_(first, second, third, fourth);
    }
  }

  void cancel()
  {
  }

  const KeyT & get()
  {
    return *it_;
  }

  void insert(T1 key1, T2 key2, T3 key3, T4 key4)
  {
    set_.insert(key1, key2, key3, key4);
  }
  void insert(T1 key1, T2 key2, T3 key3)
  {
    set_.insert(key1, key2, key3);
  }
  void insert(T1 key1, T2 key2)
  {
    set_.insert(key1, key2);
  }
  void insert(T1 key1)
  {
    if (iterating_) {
      pending_set_.insert(key1);
    } else {
      set_.insert(key1);
    }
  }

private:
  using SetT = KeysSet<T1, T2, T3, T4>;
  using IteratorT = typename KeysSet<T1, T2, T3, T4>::IteratorT;

  void merge_pending_keys()
  {
  }
  inline bool is_end_(IteratorT & lhs)
  {
    // Check whether begin function have been called beforehand.
    assert(iterating_);
    auto is_end = lhs == set_.end();
    return is_end;
  }

  SetT set_;
  SetT pending_set_;
  IteratorT it_;
  bool iterating_;

  std::function<FuncT_1> func_1_;
  std::function<FuncT_2> func_2_;
  std::function<FuncT_3> func_3_;
  std::function<FuncT_4> func_4_;
};

#endif  // CARET_TRACE__ITERABLE_KEYS_SET_HPP_
#define CARET_TRACE__ITERABLE_KEYS_SET_HPP_
