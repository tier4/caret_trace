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

#ifndef CARET_TRACE__KEYS_SET_HPP_

#include <iostream>
#include <functional>
#include <unordered_set>
#include <type_traits>
#include <iterator>
#include <cassert>

#include "caret_trace/function_traits.hpp"
#include "caret_trace/singleton.hpp"

template<
  typename T1,
  typename T2 = std::false_type,
  typename T3 = std::false_type,
  typename T4 = std::false_type
>
class HashableKeys
{
public:
  explicit HashableKeys(T1 key1)
  : key1_(key1)
  {}

  HashableKeys(T1 key1, T2 key2)
  : key1_(key1), key2_(key2)
  {}

  HashableKeys(T1 key1, T2 key2, T3 key3)
  : key1_(key1), key2_(key2), key3_(key3)
  {}

  HashableKeys(T1 key1, T2 key2, T3 key3, T4 key4)
  : key1_(key1), key2_(key2), key3_(key3), key4_(key4)
  {}

  size_t hash() const
  {
    size_t res = 17;
    if constexpr (std::true_type::value) {
      res = res * 31 + std::hash<T1>()(key1_);
    }
    if constexpr (!std::is_same_v<std::false_type, T2>) {
      res = res * 31 + std::hash<T2>()(key2_);
    }
    if constexpr (!std::is_same_v<std::false_type, T3>) {
      res = res * 31 + std::hash<T3>()(key3_);
    }
    if constexpr (!std::is_same_v<std::false_type, T4>) {
      res = res * 31 + std::hash<T4>()(key4_);
    }

    return res;
  }
  bool equal_to(const HashableKeys<T1, T2, T3, T4> & keys) const
  {
    if constexpr (std::true_type::value) {
      return key1_ == keys.key1_;
    } else if constexpr (!std::is_same_v<std::false_type, T2>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_;
    } else if constexpr (!std::is_same_v<std::false_type, T3>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_ && key3_ == keys.keys3_;
    } else if constexpr (!std::is_same_v<std::false_type, T4>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_ && key3_ == keys.keys3_ &&
             key4_ == keys.key4_;
    }

    return false;
  }

  const T1 & first() const
  {
    return key1_;
  }

  const T2 & second() const
  {
    static_assert(!std::is_same_v<T2, std::false_type>, "Invalid access.");
    return key2_;
  }

  const T3 & third() const
  {
    static_assert(!std::is_same_v<T3, std::false_type>, "Invalid access.");
    return key3_;
  }

  const T4 & fourth() const
  {
    static_assert(!std::is_same_v<T4, std::false_type>, "Invalid access.");
    return key4_;
  }

private:
  T1 key1_;
  T2 key2_;
  T3 key3_;
  T4 key4_;
};

namespace std
{
template<typename T1, typename T2, typename T3, typename T4>
struct hash<HashableKeys<T1, T2, T3, T4>>
{
  size_t operator()(const HashableKeys<T1, T2, T3, T4> & t) const
  {
    return t.hash();
  }
};

template<typename T1, typename T2, typename T3, typename T4>
struct equal_to<HashableKeys<T1, T2, T3, T4>>
{
  size_t operator()(
    const HashableKeys<T1, T2, T3, T4> & t,
    const HashableKeys<T1, T2, T3, T4> & t_) const
  {
    return t.equal_to(t_);
  }
};
}  // namespace std


template<typename T1, typename T2 = std::false_type, typename T3 = std::false_type,
  typename T4 = std::false_type>
class KeysSet
{
public:
  using SetT = std::unordered_set<HashableKeys<T1, T2, T3, T4>>;
  using IteratorT = typename SetT::iterator;

  void insert(T1 key1, T2 key2, T3 key3, T4 key4)
  {
    HashableKeys<T1, T2, T3, T4> keys(key1, key2, key3, key4);
    keys_.insert(keys);
  }
  void insert(T1 key1, T2 key2, T3 key3)
  {
    HashableKeys<T1, T2, T3> keys(key1, key2, key3);
    keys_.insert(keys);
  }
  void insert(HashableKeys<T1, T2, T3, T4> keys)
  {
    keys_.insert(keys);
  }
  void insert(T1 key1, T2 key2)
  {
    HashableKeys<T1, T2> keys(key1, key2);
    keys_.insert(keys);
  }
  void insert(T1 key1)
  {
    HashableKeys<T1> keys(key1);
    keys_.insert(keys);
  }
  bool has(T1 key1, T2 key2, T3 key3, T4 key4)
  {
    HashableKeys<T1, T2, T3, T4> keys(key1, key2, key3, key4);
    return keys_.count(keys) > 0;
  }
  bool has(T1 key1, T2 key2, T3 key3)
  {
    HashableKeys<T1, T2, T3> keys(key1, key2, key3);
    return keys_.count(keys) > 0;
  }
  bool has(T1 key1, T2 key2)
  {
    HashableKeys<T1, T2> keys(key1, key2);
    return keys_.count(keys) > 0;
  }
  bool has(T1 key1)
  {
    HashableKeys<T1> keys(key1);
    return keys_.count(keys) > 0;
  }

  IteratorT begin()
  {
    return keys_.begin();
  }

  IteratorT end()
  {
    return keys_.end();
  }

  size_t size()
  {
    return keys_.size();
  }

private:
  SetT keys_;
};


#endif  // CARET_TRACE__KEYS_SET_HPP_
#define CARET_TRACE__KEYS_SET_HPP_
