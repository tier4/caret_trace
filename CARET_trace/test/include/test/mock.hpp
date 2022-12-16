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

#ifndef MOCK_HPP_

#include "caret_trace/context.hpp"
#include "caret_trace/data_container.hpp"
#include "caret_trace/trace_node.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

/// @private
/// @brief Mock class of TraceNodeInterface used for gest.
class CaretTraceNodeModeMock : public TraceNodeInterface
{
public:
  MOCK_CONST_METHOD0(is_recording_allowed, bool());
  MOCK_CONST_METHOD0(is_recording_allowed_init, bool());
  MOCK_CONST_METHOD0(is_timer_running, bool());
  MOCK_METHOD0(get_data_container, DataContainerInterface &());
  MOCK_CONST_METHOD0(get_status, const TRACE_STATUS &());
};

/// @private
/// @brief Mock class of LttngSession used for gest.
class LttngSessionMock : public LttngSession
{
public:
  MOCK_CONST_METHOD0(is_session_running, bool());
  MOCK_CONST_METHOD0(started_session_running, bool());
};

/// @private
/// @brief Mock class of DataContainerInterface used for gest.
class DataContainerMock : public DataContainerInterface
{
public:
  MOCK_METHOD1(record, bool(uint64_t));
  MOCK_METHOD0(start_recording, void());
  MOCK_METHOD0(reset, void());
};

#endif  // MOCK_HPP_
#define MOCK_HPP_
