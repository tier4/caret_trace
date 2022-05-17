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

#ifndef CARET_TRACE__DEBUG_HPP_

#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <utility>

extern std::mutex debug_mtx;

class Debug
{
public:
  template<class Head, class ... Tail>
  void print(Head && head, Tail && ... tail)
  {
    std::lock_guard<std::mutex> lock(debug_mtx);
    std::cerr << std::this_thread::get_id();
    print_(head, std::forward<Tail>(tail)...);
    std::cerr << std::flush;
  }

private:
  void print_()
  {
    std::cerr << std::endl;
  }

  template<class Head, class ... Tail>
  void print_(Head && head, Tail && ... tail)
  {
    std::cerr << "," << head;
    print_(std::forward<Tail>(tail)...);
  }
};

extern Debug debug;

#endif  // CARET_TRACE__DEBUG_HPP_
#define CARET_TRACE__DEBUG_HPP_
