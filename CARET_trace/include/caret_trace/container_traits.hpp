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

#include "caret_trace/recordable_data.hpp"

#ifndef CARET_TRACE__CONTAINER_TRAITS_HPP_

/// @brief Type traits for container.
/// @tparam ...Args Container types.
template <typename... Args>
class ContainerTraits
{
public:
  /// @brief Type for RecordableData
  using KeysT = RecordableData<Args...>;

  /// @brief Type for record function.
  using FuncT = typename KeysT::FuncT;

  /// @brief Type for record std::function.
  using StdFuncT = typename KeysT::StdFuncT;
};

#endif  // CARET_TRACE__CONTAINER_TRAITS_HPP_
#define CARET_TRACE__CONTAINER_TRAITS_HPP_
