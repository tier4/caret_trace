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

#include <gtest/gtest.h>

#include <utility>
#include <memory>

#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/trace_node.hpp"
#include "caret_msgs/msg/start.hpp"
#include "caret_msgs/msg/end.hpp"

// TEST(LttngSessionTest, Sample){
//     auto lttng_session = std::make_unique<LttngSession>();
// }
