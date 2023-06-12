// Copyright 2022 Research Institute of Systems Planning, Inc.
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
// limitations under the License.#include <memory>

#include "caret_trace/lttng_session.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

bool LttngSessionImpl::is_session_running() const
{
  std::lock_guard<std::mutex> lock(mtx_);

  // $ lttng list | grep --quiet '\[active\]'
  // Zero exit status if there is an active session, otherwise 1
  std::string command = "lttng list | grep --quiet \'\\[active\\]\'";
  auto fp = popen(command.c_str(), "r");
  if (fp == nullptr) {
    return false;
  }

  auto is_session_running = WEXITSTATUS(pclose(fp)) == 0;
  return is_session_running;
}

bool LttngSessionImpl::started_session_running() const
{
  return started_session_running_;
}
