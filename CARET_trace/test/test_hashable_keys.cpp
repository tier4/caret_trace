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
#include <string>

#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/recordable_data.hpp"


TEST(RecordableData, IntCase) {
  HashableKeys<int> keys(1);
  HashableKeys<int> keys_(1);

  EXPECT_EQ(keys.first(), 1);
  EXPECT_EQ(keys_.first(), 1);
  EXPECT_TRUE(keys.equal_to(keys_));
  EXPECT_EQ(keys.hash(), keys_.hash());
}

TEST(RecordableData, MultiArgsCase) {
  HashableKeys<int, char, int64_t, void *, char *> keys(1, 'a', 2, nullptr, nullptr);
  HashableKeys<int, char, int64_t, void *, char *> keys_(1, 'a', 2, nullptr, nullptr);
  HashableKeys<int, char, int64_t, void *, char *> keys__(1, 'b', 2, nullptr, nullptr);

  EXPECT_EQ(keys.first(), 1);
  EXPECT_EQ(keys.second(), 'a');
  EXPECT_EQ(keys.third(), 2);
  EXPECT_EQ(keys.fourth(), nullptr);
  EXPECT_EQ(keys.fifth(), nullptr);
  EXPECT_TRUE(keys.equal_to(keys_));
  EXPECT_FALSE(keys.equal_to(keys__));
}

TEST(RecordableData, StringLiteralCase) {
  char s[] = "a";
  char s_[] = "a";

  EXPECT_NE(s, s_);

  HashableKeys<const char *> keys(s);
  HashableKeys<const char *> keys_(s_);

  auto is_equal = [](const char * lhs, const char * rhs) -> bool {
      return std::string(lhs).compare(rhs) == 0;
    };

  EXPECT_TRUE(is_equal(keys.first(), keys_.first()));
  EXPECT_EQ(keys.hash(), keys_.hash());
}

TEST(RecordableData, Size) {
  {
    HashableKeys<int8_t> keys_8(1);
    HashableKeys<int16_t> keys_16(1);
    HashableKeys<int32_t> keys_32(1);
    HashableKeys<int64_t> keys_64(1);

    EXPECT_LE(sizeof(keys_8), sizeof(keys_16));
    EXPECT_LE(sizeof(keys_16), sizeof(keys_32));
    EXPECT_LE(sizeof(keys_32), sizeof(keys_64));
  }

  {
    HashableKeys<int> keys_1(1);
    HashableKeys<int, int> keys_2(1, 2);
    HashableKeys<int, int, int> keys_3(1, 2, 3);
    HashableKeys<int, int, int, int> keys_4(1, 2, 3, 4);
    HashableKeys<int, int, int, int, int64_t> keys_5(1, 2, 3, 4, 5);

    EXPECT_LE(sizeof(keys_1), sizeof(keys_2));
    EXPECT_LE(sizeof(keys_2), sizeof(keys_3));
    EXPECT_LE(sizeof(keys_3), sizeof(keys_4));
    EXPECT_LE(sizeof(keys_4), sizeof(keys_5));
  }
}

TEST(RecordableData, LessOperator) {
  HashableKeys<int, int, int, int, int> keys(1, 2, 3, 4, 5);
  HashableKeys<int, int, int, int, int> keys_(1, 2, 3, 4, 5);
  HashableKeys<int, int, int, int, int> keys__(1, 2, 3, 4, 6);
  HashableKeys<int, int, int, int, int> keys___(2, 2, 3, 4, 6);

  EXPECT_FALSE(keys < keys_);
  EXPECT_TRUE(keys < keys__);
  EXPECT_TRUE(keys__ < keys___);
}
