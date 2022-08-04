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


#include <utility>
#include <memory>
#include <vector>
#include <stdexcept>
#include <initializer_list>

#include "caret_trace/data_container.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/iterable_keys_set.hpp"

DataRecorder::DataRecorder()
{
}

void DataRecorder::assign(std::shared_ptr<IterableKeysSetBase> iterable)
{
  iterables_.emplace_back(iterable);
}

void DataRecorder::record_once()
{
  auto & iter = get_iterating();
  iter.record_once();
}

bool DataRecorder::is_iterating()
{
  auto & iter = get_iterating();
  return iter.is_iterating();
}

bool DataRecorder::is_end()
{
  auto & iter = get_iterating();
  return iter.is_end();
}

size_t DataRecorder::size()
{
  size_t size = 0;
  for (auto & it : iterables_) {
    size += it->size();
  }
  return size;
}

void DataRecorder::begin()
{
  if (iterables_.size() == 0) {
    return;
  }

  for (auto & it : iterables_) {
    it->begin();
  }
}

IterableKeysSetBase & DataRecorder::get_iterating()
{
  for (auto & it : iterables_) {
    if (it->is_iterating() && it->is_end()) {
      continue;
    }
    return *it;
  }
  return Singleton<DummyIterableKeysSet>::get_instance();
}

IterableKeysSetBase & DataRecorder::get_next_iter()
{
  for (auto & it : iterables_) {
    if (!it->is_iterating()) {
      return *it;
    }
  }
  return Singleton<DummyIterableKeysSet>::get_instance();
}

void DataRecorder::next()
{
  auto & iter = get_iterating();
  if (iter.is_end()) {
    iter = get_next_iter();
  }

  iter.next();
}

DataContainer::DataContainer()
: add_rcl_init_(std::make_shared<ADD_RCL_INIT_TYPE>()),
  add_rcl_node_init_(std::make_shared<ADD_RCL_NODE_INIT_TYPE>())
{
  it_.assign(add_rcl_node_init_);
  it_.assign(add_rcl_init_);
}

void DataContainer::assign_rcl_node_init(RCL_NODE_INIT_TYPE rcl_node_init)
{
  add_rcl_node_init_->assign(rcl_node_init);
}

void DataContainer::assign_rcl_init(
  RCL_INIT_TYPE rcl_init
)
{
  add_rcl_init_->assign(rcl_init);
}

bool DataContainer::is_assigned_rcl_init() const
{
  return add_rcl_init_->is_assigned();
}

bool DataContainer::is_assigned_rcl_node_init() const
{
  return add_rcl_node_init_->is_assigned();
}

bool DataContainer::record(uint64_t loop_count)
{
  if (!it_.is_iterating()) {
    it_.begin();
  }
  if (it_.is_end()) {
    return true;
  }

  for (uint64_t i = 0; i < loop_count; i++) {
    it_.record_once();
    if (it_.is_end()) {
      break;
    }
    it_.next();
  }

  return it_.is_end();
}


void DataContainer::add_rcl_node_init(
  const void * obj, const void * rmw_handle, const char * node_name,
  const char * node_namespace)
{
  add_rcl_node_init_->insert(obj, rmw_handle, node_name, node_namespace);
}

void DataContainer::add_rcl_init(const void * context_handle)
{
  add_rcl_init_->insert(context_handle);
}
