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

#include <functional>
#include <iostream>
#include <iterator>
#include <set>
#include <string>
#include <type_traits>

/// @private
template <bool Cond, typename Then, typename Else>
struct if_
{
  using type = Then;
};

/// @private
template <typename Then, typename Else>
struct if_<false, Then, Else>
{
  using type = Else;
};

// clang-format off

/// @brief Tuple-like class that has hash() function.
/// @tparam T1 First argument type.
/// @tparam T2 Second argument type.
/// @tparam T3 Third argument type.
/// @tparam T4 Fourth argument type.
/// @tparam T5 Fifth argument type.
/// @tparam T6 Sixth argument type.
template<
  typename T1,
  typename T2 = std::false_type,
  typename T3 = std::false_type,
  typename T4 = std::false_type,
  typename T5 = std::false_type,
  typename T6 = std::false_type
>
// clang-format on
class HashableKeys
{
private:
  using IsT1String = std::is_same<const char *, T1>;
  using IsT2String = std::is_same<const char *, T2>;
  using IsT3String = std::is_same<const char *, T3>;
  using IsT4String = std::is_same<const char *, T4>;
  using IsT5String = std::is_same<const char *, T5>;
  using IsT6String = std::is_same<const char *, T6>;

  // Store string literal as std::string
  using T1_ = typename if_<IsT1String::value, std::string, T1>::type;
  using T2_ = typename if_<IsT2String::value, std::string, T2>::type;
  using T3_ = typename if_<IsT3String::value, std::string, T3>::type;
  using T4_ = typename if_<IsT4String::value, std::string, T4>::type;
  using T5_ = typename if_<IsT5String::value, std::string, T5>::type;
  using T6_ = typename if_<IsT6String::value, std::string, T6>::type;

public:
  /// @brief Construct an instance.
  /// @param key1 first argument.
  explicit HashableKeys(T1 key1) : key1_(key1) {}

  /// @brief Construct an instance.
  /// @param key1 first argument.
  /// @param key2 second argument.
  HashableKeys(T1 key1, T2 key2) : key1_(key1), key2_(key2) {}

  /// @brief Construct an instance.
  /// @param key1 first argument.
  /// @param key2 second argument.
  /// @param key3 third argument.
  HashableKeys(T1 key1, T2 key2, T3 key3) : key1_(key1), key2_(key2), key3_(key3) {}

  /// @brief Construct an instance.
  /// @param key1 first argument.
  /// @param key2 second argument.
  /// @param key3 third argument.
  /// @param key4 fourth argument.
  HashableKeys(T1 key1, T2 key2, T3 key3, T4 key4)
  : key1_(key1), key2_(key2), key3_(key3), key4_(key4)
  {
  }

  /// @brief Construct an instance.
  /// @param key1 first argument.
  /// @param key2 second argument.
  /// @param key3 third argument.
  /// @param key4 fourth argument.
  /// @param key5 fifth argument.
  HashableKeys(T1 key1, T2 key2, T3 key3, T4 key4, T5 key5)
  : key1_(key1), key2_(key2), key3_(key3), key4_(key4), key5_(key5)
  {
  }

  /// @brief Construct an instance.
  /// @param key1 first argument.
  /// @param key2 second argument.
  /// @param key3 third argument.
  /// @param key4 fourth argument.
  /// @param key5 fifth argument.
  /// @param key6 sixth argument.
  HashableKeys(T1 key1, T2 key2, T3 key3, T4 key4, T5 key5, T6 key6)
  : key1_(key1), key2_(key2), key3_(key3), key4_(key4), key5_(key5), key6_(key6)
  {
  }

  /// @brief Calculate hash.
  /// @return hash value.
  /// @note For hash code implementations, see:
  /// https://www.baeldung.com/java-hashcode#standard-hashcode-implementations
  size_t hash() const
  {
    size_t res = 17;
    if constexpr (std::true_type::value) {
      res = res * 31 + std::hash<T1_>()(key1_);
    }
    if constexpr (!std::is_same<std::false_type, T2>::value) {
      res = res * 31 + std::hash<T2_>()(key2_);
    }
    if constexpr (!std::is_same<std::false_type, T3>::value) {
      res = res * 31 + std::hash<T3_>()(key3_);
    }
    if constexpr (!std::is_same_v<std::false_type, T4>) {
      res = res * 31 + std::hash<T4_>()(key4_);
    }
    if constexpr (!std::is_same_v<std::false_type, T5>) {
      res = res * 31 + std::hash<T5_>()(key5_);
    }
    if constexpr (!std::is_same_v<std::false_type, T6>) {
      res = res * 31 + std::hash<T6_>()(key6_);
    }

    return res;
  }

  /// @brief Check for equivalence.
  /// @param keys Comparison target.
  /// @return True if equal, false otherwise.
  bool equal_to(const HashableKeys<T1, T2, T3, T4, T5, T6> & keys) const
  {
    if constexpr (!std::is_same_v<std::false_type, T6>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_ && key3_ == keys.key3_ &&
             key4_ == keys.key4_ && key5_ == keys.key5_ && key6_ == keys.key6_;
    } else if constexpr (!std::is_same_v<std::false_type, T5>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_ && key3_ == keys.key3_ &&
             key4_ == keys.key4_ && key5_ == keys.key5_;
    } else if constexpr (!std::is_same_v<std::false_type, T4>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_ && key3_ == keys.key3_ &&
             key4_ == keys.key4_;
    } else if constexpr (!std::is_same_v<std::false_type, T3>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_ && key3_ == keys.key3_;
    } else if constexpr (!std::is_same_v<std::false_type, T2>) {
      return key1_ == keys.key1_ && key2_ == keys.key2_;
    } else {
      return key1_ == keys.key1_;
    }
  }

  /// @brief Compare with other keys.
  /// @param rhs Comparison target.
  /// @return True if rhs is greater, false otherwise.
  bool operator<(const HashableKeys<T1, T2, T3, T4, T5, T6> & rhs) const
  {
    if (first() < rhs.first()) {
      return true;
    }

    if constexpr (!std::is_same_v<std::false_type, T2>) {
      if (second() < rhs.second()) {
        return true;
      }
    }

    if constexpr (!std::is_same_v<std::false_type, T3>) {
      if (third() < rhs.third()) {
        return true;
      }
    }

    if constexpr (!std::is_same_v<std::false_type, T4>) {
      if (fourth() < rhs.fourth()) {
        return true;
      }
    }

    if constexpr (!std::is_same_v<std::false_type, T5>) {
      if (fifth() < rhs.fifth()) {
        return true;
      }
    }

    if constexpr (!std::is_same_v<std::false_type, T6>) {
      if (sixth() < rhs.sixth()) {
        return true;
      }
    }

    return false;
  }

  /// @brief Get first argument.
  /// @return First argument.
  T1 first() const
  {
    if constexpr (IsT1String::value) {
      return key1_.c_str();
    } else {
      return key1_;
    }
  }

  /// @brief Get second argument.
  /// @return Second argument.
  T2 second() const
  {
    static_assert(!std::is_same_v<T2, std::false_type>, "Invalid access.");

    if constexpr (IsT2String::value) {
      return key2_.c_str();
    } else {
      return key2_;
    }
  }

  /// @brief Get third argument.
  /// @return Third argument.
  T3 third() const
  {
    static_assert(!std::is_same_v<T3, std::false_type>, "Invalid access.");

    if constexpr (IsT3String::value) {
      return key3_.c_str();
    } else {
      return key3_;
    }
  }

  /// @brief Get fourth argument.
  /// @return Fourth argument.
  T4 fourth() const
  {
    static_assert(!std::is_same_v<T4, std::false_type>, "Invalid access.");

    if constexpr (IsT4String::value) {
      return key4_.c_str();
    } else {
      return key4_;
    }
  }

  /// @brief Get fifth argument.
  /// @return Fifth argument.
  T5 fifth() const
  {
    static_assert(!std::is_same_v<T5, std::false_type>, "Invalid access.");

    if constexpr (IsT5String::value) {
      return key5_.c_str();
    } else {
      return key5_;
    }
  }

  /// @brief Get sixth argument.
  /// @return Sixth argument.
  T6 sixth() const
  {
    static_assert(!std::is_same_v<T6, std::false_type>, "Invalid access.");

    if constexpr (IsT6String::value) {
      return key6_.c_str();
    } else {
      return key6_;
    }
  }

private:
  T1_ key1_;
  T2_ key2_;
  T3_ key3_;
  T4_ key4_;
  T5_ key5_;
  T6_ key6_;
};

namespace std
{
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
struct hash<HashableKeys<T1, T2, T3, T4, T5, T6>>
{
  size_t operator()(const HashableKeys<T1, T2, T3, T4, T5, T6> & t) const { return t.hash(); }
};

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
struct equal_to<HashableKeys<T1, T2, T3, T4, T5, T6>>
{
  size_t operator()(
    const HashableKeys<T1, T2, T3, T4, T5, T6> & t,
    const HashableKeys<T1, T2, T3, T4, T5, T6> & t_) const
  {
    return t.equal_to(t_);
  }
};
}  // namespace std

/// @brief A set class for HashableKeys class.
/// @tparam T1 First argument type.
/// @tparam T2 Second argument type.
/// @tparam T3 Third argument type.
/// @tparam T4 Third argument type.
/// @tparam T5 Third argument type.
/// @tparam T6 Third argument type.
// clang-format off
template <
  typename T1,
  typename T2 = std::false_type,
  typename T3 = std::false_type,
  typename T4 = std::false_type,
  typename T5 = std::false_type,
  typename T6 = std::false_type
>
// clang-format on
class KeysSet
{
public:
  //  Use set to loop through iterators in order of addition.
  using SetT = std::set<HashableKeys<T1, T2, T3, T4, T5, T6>>;
  using IteratorT = typename SetT::iterator;
  using ConstIteratorT = typename SetT::const_iterator;

  /// @brief Insert new keys.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  /// @param key5 Fifth argument.
  /// @param key6 Fifth argument.
  void insert(T1 key1, T2 key2, T3 key3, T4 key4, T5 key5, T6 key6)
  {
    HashableKeys<T1, T2, T3, T4, T5, T6> keys(key1, key2, key3, key4, key5, key6);
    keys_.insert(keys);
  }

  /// @brief Insert new keys.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  /// @param key5 Fifth argument.
  void insert(T1 key1, T2 key2, T3 key3, T4 key4, T5 key5)
  {
    HashableKeys<T1, T2, T3, T4, T5> keys(key1, key2, key3, key4, key5);
    keys_.insert(keys);
  }

  /// @brief Insert new keys.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  void insert(T1 key1, T2 key2, T3 key3, T4 key4)
  {
    HashableKeys<T1, T2, T3, T4> keys(key1, key2, key3, key4);
    keys_.insert(keys);
  }

  /// @brief Insert new keys.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  void insert(T1 key1, T2 key2, T3 key3)
  {
    HashableKeys<T1, T2, T3> keys(key1, key2, key3);
    keys_.insert(keys);
  }

  /// @brief Insert new keys.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  void insert(T1 key1, T2 key2)
  {
    HashableKeys<T1, T2> keys(key1, key2);
    keys_.insert(keys);
  }

  /// @brief Insert new keys.
  /// @param key1 First argument.
  void insert(T1 key1)
  {
    HashableKeys<T1> keys(key1);
    keys_.insert(keys);
  }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  /// @param key5 Fifth argument.
  /// @return True if it contains, false otherwise.
  void insert(HashableKeys<T1, T2, T3, T4, T5, T6> keys) { keys_.insert(keys); }

  /// @brief Clear set.
  void clear() { keys_.clear(); }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  /// @param key5 Fifth argument.
  /// @param key6 Fifth argument.
  /// @return True if it contains, false otherwise.
  bool has(T1 key1, T2 key2, T3 key3, T4 key4, T5 key5, T6 key6) const
  {
    HashableKeys<T1, T2, T3, T4, T5, T6> keys(key1, key2, key3, key4, key5, key6);
    return keys_.count(keys) > 0;
  }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  /// @param key5 Fifth argument.
  /// @return True if it contains, false otherwise.
  bool has(T1 key1, T2 key2, T3 key3, T4 key4, T5 key5) const
  {
    HashableKeys<T1, T2, T3, T4, T5> keys(key1, key2, key3, key4, key5);
    return keys_.count(keys) > 0;
  }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @param key4 Fourth argument.
  /// @return True if it contains, false otherwise.
  bool has(T1 key1, T2 key2, T3 key3, T4 key4) const
  {
    HashableKeys<T1, T2, T3, T4> keys(key1, key2, key3, key4);
    return keys_.count(keys) > 0;
  }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @param key3 Third argument.
  /// @return True if it contains, false otherwise.
  bool has(T1 key1, T2 key2, T3 key3) const
  {
    HashableKeys<T1, T2, T3> keys(key1, key2, key3);
    return keys_.count(keys) > 0;
  }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @param key2 Second argument.
  /// @return True if it contains, false otherwise.
  bool has(T1 key1, T2 key2) const
  {
    HashableKeys<T1, T2> keys(key1, key2);
    return keys_.count(keys) > 0;
  }

  /// @brief Confirm content.
  /// @param key1 First argument.
  /// @return True if it contains, false otherwise.

  bool has(T1 key1) const
  {
    HashableKeys<T1> keys(key1);
    return keys_.count(keys) > 0;
  }

  /// @brief Get const iterator
  /// @return Iterator referring to the first element.
  ConstIteratorT begin() const { return keys_.cbegin(); }

  /// @brief Get const iterator
  /// @return Iterator referring to the last element.
  ConstIteratorT end() const { return keys_.cend(); }

  /// @brief Get size.
  /// @return Element number.
  size_t size() const { return keys_.size(); }

private:
  SetT keys_;
};

#endif  // CARET_TRACE__KEYS_SET_HPP_
#define CARET_TRACE__KEYS_SET_HPP_
