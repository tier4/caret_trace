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
#include "caret_trace/tracing_controller.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

using ::testing::_;
using ::testing::MockFunction;
using ::testing::Return;

TEST(RecordableData, EmptyCase)
{
  using DataT = RecordableData<int, const char *>;
  auto set = std::make_unique<DataT>("test");

  EXPECT_EQ(set->size(), (size_t)0);
  EXPECT_EQ(set->trace_point(), "test");

  set->start();
  set->record_next_one();
  set->record_next_one();
  set->reset();
}

TEST(RecordableData, TwoTypesCase)
{
  using DataT = RecordableData<int, const char *>;

  auto set = std::make_unique<DataT>("test");

  MockFunction<DataT::FuncT> mock_func;
  EXPECT_CALL(mock_func, Call(_, _)).WillRepeatedly(Return());

  EXPECT_EQ(set->size(), (size_t)0);
  set->store(1, "test");
  EXPECT_EQ(set->size(), (size_t)1);

  EXPECT_FALSE(set->is_recording());

  set->start();

  EXPECT_TRUE(set->is_recording());
  EXPECT_FALSE(set->finished());

  EXPECT_EQ(set->get().first(), 1);

  auto is_equal = [](const char * lhs, const char * rhs) -> bool {
    return std::string(lhs).compare(rhs) == 0;
  };

  EXPECT_TRUE(is_equal(set->get().second(), "test"));

  set->assign(mock_func.AsStdFunction());
  set->record_next_one();
  EXPECT_TRUE(set->finished());
}

TEST(RecordableData, MergePendingData)
{
  using DataT = RecordableData<int>;
  MockFunction<DataT::FuncT> mock_func;
  EXPECT_CALL(mock_func, Call(_)).WillRepeatedly(Return());

  {
    auto set = std::make_unique<DataT>("test");
    set->assign(mock_func.AsStdFunction());

    set->store(1);
    set->start();
    set->store(3);

    EXPECT_TRUE(set->finished() == false);
    EXPECT_EQ(set->get().first(), 1);

    set->record_next_one();
    EXPECT_TRUE(set->finished() == true);
  }

  {
    auto set = std::make_unique<DataT>("test");
    set->assign(mock_func.AsStdFunction());

    set->store(1);
    set->start();
    set->store(3);

    EXPECT_EQ(set->size(), (size_t)1);

    set->record_next_one();
    set->record_next_one();

    EXPECT_EQ(set->size(), (size_t)2);
  }

  {
    auto set = std::make_unique<DataT>("test");
    set->assign(mock_func.AsStdFunction());

    set->store(1);
    set->start();
    set->store(3);

    EXPECT_EQ(set->size(), (size_t)1);
    set->reset();
    EXPECT_EQ(set->size(), (size_t)2);
  }

  {
    auto set = std::make_unique<DataT>("test");
    set->assign(mock_func.AsStdFunction());

    set->store(1);
    set->start();
    set->store(3);

    EXPECT_EQ(set->size(), (size_t)1);
    set->start();
    EXPECT_EQ(set->size(), (size_t)2);
  }
}
