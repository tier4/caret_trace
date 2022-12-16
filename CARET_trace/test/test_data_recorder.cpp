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

#include "caret_trace/data_container.hpp"
#include "caret_trace/keys_set.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

using ::testing::_;
using ::testing::InSequence;
using ::testing::MockFunction;
using ::testing::Return;

TEST(DataRecorderTest, EmptyCase)
{
  DataRecorder recorder({});
  recorder.start();
  EXPECT_FALSE(recorder.is_recording());
  EXPECT_TRUE(recorder.finished());
}

TEST(DataRecorderTest, AssignCaseOneKeys)
{
  auto key = std::make_shared<RecordableData<int>>("test");
  DataRecorder recorder({key});
  bool called = false;

  key->assign([&](const int arg) {
    (void)arg;
    called = true;
  });

  key->store(0);

  recorder.start();
  recorder.record_next_one();
  EXPECT_TRUE(called);

  EXPECT_TRUE(recorder.finished());
  EXPECT_FALSE(recorder.is_recording());
}

TEST(DataRecorderTest, AssignCaseTwoKeys)
{
  auto key_0 = std::make_shared<RecordableData<int>>("test");
  auto key_1 = std::make_shared<RecordableData<char>>("test");

  DataRecorder recorder({key_0, key_1});

  MockFunction<void(int arg)> key_0_func;
  MockFunction<void(char arg)> key_1_func;

  key_0->assign(key_0_func.AsStdFunction());
  key_1->assign(key_1_func.AsStdFunction());

  key_0->store(0);
  key_1->store('a');

  {
    InSequence s;

    EXPECT_CALL(key_0_func, Call(0)).Times(1);
    EXPECT_CALL(key_1_func, Call('a')).Times(1);
  }

  recorder.start();

  recorder.record_next_one();
  recorder.record_next_one();
}

TEST(DataRecorderTest, Reset)
{
  auto key = std::make_shared<RecordableData<int>>("trace_point_name");
  DataRecorder recorder({key});
  EXPECT_NO_THROW(recorder.reset());

  MockFunction<void(int arg)> key_func;

  key->assign(key_func.AsStdFunction());

  key->store(0);
  key->store(1);

  EXPECT_CALL(key_func, Call(0)).Times(2);
  EXPECT_CALL(key_func, Call(1)).Times(0);

  recorder.start();
  EXPECT_TRUE(recorder.is_recording());

  recorder.record_next_one();

  recorder.reset();
  EXPECT_FALSE(recorder.is_recording());

  recorder.start();
  recorder.record_next_one();
}

TEST(DataRecorderTest, MergePendingData)
{
  MockFunction<void(int arg)> key_func;
  EXPECT_CALL(key_func, Call(_)).WillRepeatedly(Return());
  {
    auto key = std::make_shared<RecordableData<int>>("test");
    DataRecorder recorder({key});

    key->assign(key_func.AsStdFunction());

    key->store(0);
    recorder.start();

    key->store(1);

    EXPECT_EQ(key->size(), (size_t)1);
    recorder.reset();
    EXPECT_EQ(key->size(), (size_t)2);
  }

  {
    auto key = std::make_shared<RecordableData<int>>("test");
    DataRecorder recorder({key});

    key->assign(key_func.AsStdFunction());

    key->store(0);
    recorder.start();

    key->store(1);

    recorder.record_next_one();
    recorder.record_next_one();

    EXPECT_EQ(key->size(), (size_t)2);
  }

  {
    auto key = std::make_shared<RecordableData<int>>("test");
    DataRecorder recorder({key});

    key->assign(key_func.AsStdFunction());

    key->store(0);
    recorder.start();
    key->store(1);

    recorder.start();

    EXPECT_EQ(key->size(), (size_t)2);
  }
}
