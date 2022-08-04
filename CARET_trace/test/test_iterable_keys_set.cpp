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
#include "caret_trace/iterable_keys_set.hpp"


TEST(IterableKeysSet, EmptyCase) {
  using SetT = IterableKeysSet<int, const char *>;
  auto set = std::make_unique<SetT>();

  EXPECT_EQ(set->size(), (size_t) 0);

  set->begin();
  set->record_once();
  set->next();
  set->cancel();
}

TEST(IterableKeysSet, TwoTypesCase) {
  using SetT = IterableKeysSet<int, const char *>;
  auto set = std::make_unique<SetT>();

  EXPECT_EQ(set->size(), (size_t) 0);
  set->insert(1, "test");
  EXPECT_EQ(set->size(), (size_t) 1);

  EXPECT_FALSE(set->is_iterating());

  set->begin();

  EXPECT_TRUE(set->is_iterating());
  EXPECT_FALSE(set->is_end());

  EXPECT_EQ(set->get().first(), 1);
  EXPECT_EQ(set->get().second(), "test");

  set->next();
  EXPECT_TRUE(set->is_end());
}

TEST(IterableKeysSet, MergePendingSample) {
  using SetT = IterableKeysSet<int>;
  auto set = std::make_unique<SetT>();

  set->insert(1);
  set->begin();
  set->insert(3);

  EXPECT_TRUE(set->is_end() == false);
  set->next();
  EXPECT_TRUE(set->is_end() == true);

  set->begin();
  EXPECT_TRUE(set->is_end() == false);

  set->next();
  EXPECT_TRUE(set->is_end() == false);

  set->next();
  EXPECT_TRUE(set->is_end() == true);
}
