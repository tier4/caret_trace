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

#include <lttng/ust-clock.h>

#include <cstdint>

#ifndef CARET_TRACE__CLOCK_HPP_

/// @brief Clock class.
class Clock
{
public:
  /// @brief Construct an instance.
  Clock();

  /// @brief Get current time in lttng clock. Default is monotonic clock.
  /// @return Current time.
  int64_t now();

private:
  lttng_ust_clock_read64_function lttng_clock_;
};

#endif  // CARET_TRACE__CLOCK_HPP_
#define CARET_TRACE__CLOCK_HPP_
