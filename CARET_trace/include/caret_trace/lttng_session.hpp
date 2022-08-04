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

#include <iostream>
#include <functional>
#include <unordered_set>
#include <type_traits>
#include <iterator>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"


class LttngSession
{
public:
  virtual ~LttngSession() {}
  virtual bool is_session_running() const = 0;
};

class LttngSessionImpl : public LttngSession
{
public:
  LttngSessionImpl()
  {
  }

  ~LttngSessionImpl() {}

  bool is_session_running() const
  {
    std::string command = "lttng list | grep -q \'\\[active\\]\'";
    auto fp = popen(command.c_str(), "r");
    if (fp == nullptr) {
      return false;
    }

    auto is_session_running = WEXITSTATUS(pclose(fp)) == 0;
    return is_session_running;
  }
};

#endif  // CARET_TRACE__LTTNG_SESSION_HPP_
#define CARET_TRACE__LTTNG_SESSION_HPP_
