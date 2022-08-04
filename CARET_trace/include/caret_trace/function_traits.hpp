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

#include <functional>
#include <iostream>
#include <tuple>

#ifndef CARET_TRACE__FUNCTION_TRAITS_HPP_

template<typename T>
struct function_traits;

template<typename R, typename ... Args>
struct function_traits<std::function<R(Args...)>>
{
  static const size_t nargs = sizeof...(Args);

  typedef R result_type;

  template<size_t i>
  struct arg
  {
    typedef typename std::tuple_element<i, std::tuple<Args...>>::type type;
  };
};

template<typename R, typename ... Args>
struct function_traits<R(Args...)>
{
  static const size_t nargs = sizeof...(Args);

  typedef R result_type;

  template<size_t i>
  struct arg
  {
    typedef typename std::tuple_element<i, std::tuple<Args...>>::type type;
  };
};

#endif  // CARET_TRACE__FUNCTION_TRAITS_HPP_
#define CARET_TRACE__FUNCTION_TRAITS_HPP_
