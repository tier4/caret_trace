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

#ifndef CARET_TRACE__CLOCK_HPP

static __inline__
uint64_t trace_clock_read64_monotonic(void)
{
  struct timespec ts;

  if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    return ((uint64_t) ts.tv_sec * 1000000000ULL) + ts.tv_nsec;
  }
  return 0;
}

#endif  // CARET_TRACE__CLOCK_HPP
#define CARET_TRACE__CLOCK_HPP
