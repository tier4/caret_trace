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

#include <functional>
#include <memory>
#include <vector>
#include <initializer_list>

#include "caret_trace/function_traits.hpp"
#include "caret_trace/iterable_keys_set.hpp"

#ifndef CARET_TRACE__DATA_CONTAINER_HPP_


class DataRecorder
{
public:
  DataRecorder();

  void assign(std::shared_ptr<IterableKeysSetBase> iterable);
  void record_once();
  bool is_iterating();
  bool is_end();
  void begin();
  void next();
  size_t size();

private:
  IterableKeysSetBase & get_iterating();
  IterableKeysSetBase & get_next_iter();

  std::vector<std::shared_ptr<IterableKeysSetBase>> iterables_;
};

class DataContainerInterface
{
public:
  virtual ~DataContainerInterface() {}
  virtual bool record(uint64_t loop_count = 1) = 0;
};

class DataContainer : public DataContainerInterface
{
public:
  DataContainer();

  using RCL_INIT_TYPE = std::function<void (const void * context_handle)>;
  using RCL_NODE_INIT_TYPE = std::function<void (const void *, const void *, const char *,
      const char *)>;

  bool record(uint64_t loop_count = 1);
  void add_rcl_node_init(
    const void * obj, const void * rmw_handle, const char * node_name, const char * node_namespace);
  void add_rcl_init(const void * context_handle);

  void assign_rcl_node_init(RCL_NODE_INIT_TYPE rcl_node_init);
  void assign_rcl_init(RCL_INIT_TYPE add_rcl_init);

  bool is_assigned_rcl_node_init() const;
  bool is_assigned_rcl_init() const;

private:
  using ADD_RCL_NODE_INIT_TYPE = IterableKeysSet<const void *, const void *, const char *,
      const char *>;
  std::shared_ptr<ADD_RCL_NODE_INIT_TYPE> add_rcl_node_init_;
  using ADD_RCL_INIT_TYPE = IterableKeysSet<const void *>;
  std::shared_ptr<ADD_RCL_INIT_TYPE> add_rcl_init_;

  DataRecorder it_;
};

#endif  // CARET_TRACE__DATA_CONTAINER_HPP_
#define CARET_TRACE__DATA_CONTAINER_HPP_
