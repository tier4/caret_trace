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

#ifndef CARET_TRACE__LTTNG_SESSION_HPP_

#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_set>

/// @brief Interface class for manipulating Lttng sessions.
class LttngSession
{
public:
  virtual ~LttngSession() {}

  /// @brief Check whether session is up and running.
  /// @return True if running, false otherwise.
  virtual bool is_session_running() const = 0;

  /// @brief Check whether the session was up at startup.
  /// @return True if running, false otherwise.
  virtual bool started_session_running() const = 0;
};

/// @brief Implementation of LttngSession.
class LttngSessionImpl : public LttngSession
{
public:
  LttngSessionImpl() : started_session_running_(is_session_running()) {}

  ~LttngSessionImpl() {}

  bool is_session_running() const;
  bool started_session_running() const;

private:
  mutable std::mutex mtx_;
  const bool started_session_running_;
};

#endif  // CARET_TRACE__LTTNG_SESSION_HPP_
#define CARET_TRACE__LTTNG_SESSION_HPP_
