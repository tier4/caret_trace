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

#include "caret_trace/keys_set.hpp"
#include "caret_trace/data_container.hpp"

TEST(DataRecorderTest, EmptyCase) {
  DataRecorder recorder;
  recorder.begin();
  EXPECT_TRUE(recorder.is_iterating());
  EXPECT_TRUE(recorder.is_end());
  EXPECT_NO_THROW(recorder.next());
}

TEST(DataRecorderTest, AssignCaseOneKeys) {
  DataRecorder recorder;
  auto key = std::make_shared<IterableKeysSet<int>>();
  bool called = false;

  key->assign(
    [&](const int arg) {
      (void) arg;
      called = true;
    });

  key->insert(0);

  recorder.assign(key);
  recorder.begin();
  recorder.record_once();
  recorder.next();
  EXPECT_TRUE(called);

  EXPECT_TRUE(recorder.is_end());
  EXPECT_TRUE(recorder.is_iterating());
}

TEST(DataRecorderTest, AssignCaseTwoKeys) {
  DataRecorder recorder;

  auto key_0 = std::make_shared<IterableKeysSet<int>>();
  auto key_1 = std::make_shared<IterableKeysSet<char>>();
  bool called_0 = false;
  bool called_1 = false;

  key_0->assign(
    [&](const int arg) {
      (void) arg;
      called_0 = true;
    });
  key_1->assign(
    [&](const char arg) {
      (void) arg;
      called_1 = true;
    });

  key_0->insert(0);
  key_1->insert('a');
  recorder.assign(key_0);
  recorder.assign(key_1);

  recorder.begin();

  recorder.record_once();
  recorder.next();
  EXPECT_TRUE(called_0);
  EXPECT_FALSE(called_1);
  EXPECT_FALSE(recorder.is_end());
  EXPECT_TRUE(recorder.is_iterating());

  recorder.record_once();
  recorder.next();
  EXPECT_TRUE(called_0);
  EXPECT_TRUE(called_1);
  EXPECT_TRUE(recorder.is_end());
  EXPECT_TRUE(recorder.is_iterating());
}
