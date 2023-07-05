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

#include "caret_trace/data_recorder.hpp"
#include "caret_trace/recordable_data.hpp"
#include "caret_trace/singleton.hpp"

#include <cassert>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

DataContainer::DataContainer()
: DataContainer(
    std::make_shared<AddCallbackGroup::KeysT>("add_callback_group"),
    std::make_shared<AddCallbackGroupStaticExecutor::KeysT>("add_callback_group_static_executor"),
    std::make_shared<CallbackGroupAddClient::KeysT>("callback_group_add_client"),
    std::make_shared<CallbackGroupAddService::KeysT>("callback_group_add_service"),
    std::make_shared<CallbackGroupAddSubscription::KeysT>("callback_group_add_subscription"),
    std::make_shared<CallbackGroupAddTimer::KeysT>("callback_group_add_timer"),
    std::make_shared<ConstructExecutor::KeysT>("construct_executor"),
    std::make_shared<ConstructStaticExecutor::KeysT>("construct_static_executor"),
    std::make_shared<RclClientInit::KeysT>("rcl_client_init"),
    std::make_shared<RclInit::KeysT>("rcl_init"),
    std::make_shared<RclNodeInit::KeysT>("rcl_node_init"),
    std::make_shared<RclPublisherInit::KeysT>("rcl_publisher_init"),
    std::make_shared<RclServiceInit::KeysT>("rcl_service_init"),
    std::make_shared<RclSubscriptionInit::KeysT>("rcl_subscription_init"),
    std::make_shared<RclTimerInit::KeysT>("rcl_timer_init"),
    std::make_shared<RclcppCallbackRegister::KeysT>("rclcpp_callback_register"),
    std::make_shared<RclcppServiceCallbackAdded::KeysT>("rclcpp_service_callback_added"),
    std::make_shared<RclcppSubscriptionCallbackAdded::KeysT>("rclcpp_subscription_callback_added"),
    std::make_shared<RclcppSubscriptionInit::KeysT>("rclcpp_subscription_init"),
    std::make_shared<RclcppTimerCallbackAdded::KeysT>("rclcpp_timer_callback_added"),
    std::make_shared<RclcppTimerLinkNode::KeysT>("rclcpp_timer_link_node"),
    std::make_shared<RclcppConstructRingBuffer::KeysT>("rclcpp_construct_ring_buffer"),
    std::make_shared<RclcppBufferToIpb::KeysT>("rclcpp_buffer_to_ipb"),
    std::make_shared<RclcppIpbToSubscription::KeysT>("rclcpp_ipb_to_subscription"),
    std::make_shared<RmwImplementation::KeysT>("rmw_implementation"))
{
}

DataContainer::DataContainer(
  std::shared_ptr<AddCallbackGroup::KeysT> add_callback_group,
  std::shared_ptr<AddCallbackGroupStaticExecutor::KeysT> add_callback_group_static_executor,
  std::shared_ptr<CallbackGroupAddClient::KeysT> callback_group_add_client,
  std::shared_ptr<CallbackGroupAddService::KeysT> callback_group_add_service,
  std::shared_ptr<CallbackGroupAddSubscription::KeysT> callback_group_add_subscription,
  std::shared_ptr<CallbackGroupAddTimer::KeysT> callback_group_add_timer,
  std::shared_ptr<ConstructExecutor::KeysT> construct_executor,
  std::shared_ptr<ConstructStaticExecutor::KeysT> construct_static_executor,
  std::shared_ptr<RclClientInit::KeysT> rcl_client_init, std::shared_ptr<RclInit::KeysT> rcl_init,
  std::shared_ptr<RclNodeInit::KeysT> rcl_node_init,
  std::shared_ptr<RclPublisherInit::KeysT> rcl_publisher_init,
  std::shared_ptr<RclServiceInit::KeysT> rcl_service_init,
  std::shared_ptr<RclSubscriptionInit::KeysT> rcl_subscription_init,
  std::shared_ptr<RclTimerInit::KeysT> rcl_timer_init,
  std::shared_ptr<RclcppCallbackRegister::KeysT> rclcpp_callback_register,
  std::shared_ptr<RclcppServiceCallbackAdded::KeysT> rclcpp_service_callback_added,
  std::shared_ptr<RclcppSubscriptionCallbackAdded::KeysT> rclcpp_subscription_callback_added,
  std::shared_ptr<RclcppSubscriptionInit::KeysT> rclcpp_subscription_init,
  std::shared_ptr<RclcppTimerCallbackAdded::KeysT> rclcpp_timer_callback_added,
  std::shared_ptr<RclcppTimerLinkNode::KeysT> rclcpp_timer_link_node,
  std::shared_ptr<RclcppConstructRingBuffer::KeysT> rclcpp_construct_ring_buffer,
  std::shared_ptr<RclcppBufferToIpb::KeysT> rclcpp_buffer_to_ipb,
  std::shared_ptr<RclcppIpbToSubscription::KeysT> rclcpp_ipb_to_subscription,
  std::shared_ptr<RmwImplementation::KeysT> rmw_implementation)
: add_callback_group_(add_callback_group),
  add_callback_group_static_executor_(add_callback_group_static_executor),
  callback_group_add_client_(callback_group_add_client),
  callback_group_add_service_(callback_group_add_service),
  callback_group_add_subscription_(callback_group_add_subscription),
  callback_group_add_timer_(callback_group_add_timer),
  construct_executor_(construct_executor),
  construct_static_executor_(construct_static_executor),
  rcl_client_init_(rcl_client_init),
  rcl_init_(rcl_init),
  rcl_node_init_(rcl_node_init),
  rcl_publisher_init_(rcl_publisher_init),
  rcl_service_init_(rcl_service_init),
  rcl_subscription_init_(rcl_subscription_init),
  rcl_timer_init_(rcl_timer_init),
  rclcpp_callback_register_(rclcpp_callback_register),
  rclcpp_service_callback_added_(rclcpp_service_callback_added),
  rclcpp_subscription_callback_added_(rclcpp_subscription_callback_added),
  rclcpp_subscription_init_(rclcpp_subscription_init),
  rclcpp_timer_callback_added_(rclcpp_timer_callback_added),
  rclcpp_timer_link_node_(rclcpp_timer_link_node),
  rclcpp_construct_ring_buffer_(rclcpp_construct_ring_buffer),
  rclcpp_buffer_to_ipb_(rclcpp_buffer_to_ipb),
  rclcpp_ipb_to_subscription_(rclcpp_ipb_to_subscription),
  rmw_implementation_(rmw_implementation)
{
  std::vector<std::shared_ptr<RecordableDataInterface>> recordable_data;

  if (add_callback_group_) {
    recordable_data.emplace_back(add_callback_group_);
  }
  if (add_callback_group_static_executor_) {
    recordable_data.emplace_back(add_callback_group_static_executor_);
  }
  if (callback_group_add_client_) {
    recordable_data.emplace_back(callback_group_add_client_);
  }
  if (callback_group_add_service_) {
    recordable_data.emplace_back(callback_group_add_service_);
  }
  if (callback_group_add_subscription_) {
    recordable_data.emplace_back(callback_group_add_subscription_);
  }
  if (callback_group_add_timer_) {
    recordable_data.emplace_back(callback_group_add_timer_);
  }
  if (construct_executor_) {
    recordable_data.emplace_back(construct_executor_);
  }
  if (construct_static_executor_) {
    recordable_data.emplace_back(construct_static_executor_);
  }
  if (rcl_client_init_) {
    recordable_data.emplace_back(rcl_client_init_);
  }
  if (rcl_init_) {
    recordable_data.emplace_back(rcl_init_);
  }
  if (rcl_node_init_) {
    recordable_data.emplace_back(rcl_node_init_);
  }
  if (rcl_publisher_init_) {
    recordable_data.emplace_back(rcl_publisher_init_);
  }
  if (rcl_service_init_) {
    recordable_data.emplace_back(rcl_service_init_);
  }
  if (rcl_subscription_init_) {
    recordable_data.emplace_back(rcl_subscription_init_);
  }
  if (rcl_timer_init_) {
    recordable_data.emplace_back(rcl_timer_init_);
  }
  if (rclcpp_callback_register_) {
    recordable_data.emplace_back(rclcpp_callback_register_);
  }
  if (rclcpp_service_callback_added_) {
    recordable_data.emplace_back(rclcpp_service_callback_added_);
  }
  if (rclcpp_subscription_callback_added_) {
    recordable_data.emplace_back(rclcpp_subscription_callback_added_);
  }
  if (rclcpp_subscription_init_) {
    recordable_data.emplace_back(rclcpp_subscription_init_);
  }
  if (rclcpp_timer_callback_added_) {
    recordable_data.emplace_back(rclcpp_timer_callback_added_);
  }
  if (rclcpp_timer_link_node_) {
    recordable_data.emplace_back(rclcpp_timer_link_node_);
  }
  if (rclcpp_construct_ring_buffer_) {
    recordable_data.emplace_back(rclcpp_construct_ring_buffer_);
  }
  if (rclcpp_buffer_to_ipb_) {
    recordable_data.emplace_back(rclcpp_buffer_to_ipb_);
  }
  if (rclcpp_ipb_to_subscription_) {
    recordable_data.emplace_back(rclcpp_ipb_to_subscription_);
  }
  if (rmw_implementation_) {
    recordable_data.emplace_back(rmw_implementation_);
  }

  recorder_ = std::make_shared<DataRecorder>(recordable_data);
}

bool DataContainer::record(uint64_t loop_count)
{
  if (!recorder_->is_recording()) {
    recorder_->start();
  }
  if (recorder_->finished()) {
    return true;
  }

  for (uint64_t i = 0; i < loop_count; i++) {
    recorder_->record_next_one();
    if (recorder_->finished()) {
      break;
    }
  }
  return recorder_->finished();
}

void DataContainer::start_recording()
{
  if (!recorder_->is_recording()) {
    recorder_->start();
  }
}

void DataContainer::reset()
{
  recorder_->reset();
}

std::vector<std::string> DataContainer::trace_points() const
{
  return recorder_->trace_points();
}

void DataContainer::assign_add_callback_group(AddCallbackGroup::StdFuncT record)
{
  assert(add_callback_group_.get() != nullptr);
  add_callback_group_->assign(record);
}
void DataContainer::assign_add_callback_group_static_executor(
  AddCallbackGroupStaticExecutor::StdFuncT record)
{
  assert(add_callback_group_static_executor_.get() != nullptr);
  add_callback_group_static_executor_->assign(record);
}

void DataContainer::assign_callback_group_add_client(CallbackGroupAddClient::StdFuncT record)
{
  assert(callback_group_add_client_.get() != nullptr);
  callback_group_add_client_->assign(record);
}

void DataContainer::assign_callback_group_add_service(CallbackGroupAddService::StdFuncT record)
{
  assert(callback_group_add_service_.get() != nullptr);
  callback_group_add_service_->assign(record);
}

void DataContainer::assign_callback_group_add_subscription(
  CallbackGroupAddSubscription::StdFuncT record)
{
  assert(callback_group_add_subscription_.get() != nullptr);
  callback_group_add_subscription_->assign(record);
}

void DataContainer::assign_callback_group_add_timer(CallbackGroupAddTimer::StdFuncT record)
{
  assert(callback_group_add_timer_.get() != nullptr);
  callback_group_add_timer_->assign(record);
}

void DataContainer::assign_construct_executor(ConstructExecutor::StdFuncT record)
{
  assert(construct_executor_.get() != nullptr);
  construct_executor_->assign(record);
}

void DataContainer::assign_construct_static_executor(ConstructStaticExecutor::StdFuncT record)
{
  assert(construct_static_executor_.get() != nullptr);
  construct_static_executor_->assign(record);
}

void DataContainer::assign_rcl_client_init(RclClientInit::StdFuncT record)
{
  assert(rcl_client_init_.get() != nullptr);
  rcl_client_init_->assign(record);
}

void DataContainer::assign_rcl_init(RclInit::StdFuncT record)
{
  assert(rcl_init_.get() != nullptr);
  rcl_init_->assign(record);
}

void DataContainer::assign_rcl_node_init(RclNodeInit::StdFuncT record)
{
  assert(rcl_node_init_.get() != nullptr);
  rcl_node_init_->assign(record);
}

void DataContainer::assign_rcl_publisher_init(RclPublisherInit::StdFuncT record)
{
  assert(rcl_publisher_init_.get() != nullptr);
  rcl_publisher_init_->assign(record);
}

void DataContainer::assign_rcl_service_init(RclServiceInit::StdFuncT record)
{
  assert(rcl_service_init_.get() != nullptr);
  rcl_service_init_->assign(record);
}

void DataContainer::assign_rcl_subscription_init(RclSubscriptionInit::StdFuncT record)
{
  assert(rcl_subscription_init_.get() != nullptr);
  rcl_subscription_init_->assign(record);
}

void DataContainer::assign_rcl_timer_init(RclTimerInit::StdFuncT record)
{
  assert(rcl_timer_init_.get() != nullptr);
  rcl_timer_init_->assign(record);
}

void DataContainer::assign_rclcpp_callback_register(RclcppCallbackRegister::StdFuncT record)
{
  assert(rclcpp_callback_register_.get() != nullptr);
  rclcpp_callback_register_->assign(record);
}

void DataContainer::assign_rclcpp_service_callback_added(
  RclcppServiceCallbackAdded::StdFuncT record)
{
  assert(rclcpp_service_callback_added_.get() != nullptr);
  rclcpp_service_callback_added_->assign(record);
}

void DataContainer::assign_rclcpp_subscription_callback_added(
  RclcppSubscriptionCallbackAdded::StdFuncT record)
{
  assert(rclcpp_subscription_callback_added_.get() != nullptr);
  rclcpp_subscription_callback_added_->assign(record);
}

void DataContainer::assign_rclcpp_subscription_init(RclcppSubscriptionInit::StdFuncT record)
{
  assert(rclcpp_subscription_init_.get() != nullptr);
  rclcpp_subscription_init_->assign(record);
}

void DataContainer::assign_rclcpp_timer_callback_added(RclcppTimerCallbackAdded::StdFuncT record)
{
  assert(rclcpp_timer_callback_added_.get() != nullptr);
  rclcpp_timer_callback_added_->assign(record);
}

void DataContainer::assign_rclcpp_timer_link_node(RclcppTimerLinkNode::StdFuncT record)
{
  assert(rclcpp_timer_link_node_.get() != nullptr);
  rclcpp_timer_link_node_->assign(record);
}

void DataContainer::assign_rclcpp_construct_ring_buffer(RclcppConstructRingBuffer::StdFuncT record)
{
  assert(rclcpp_construct_ring_buffer_.get() != nullptr);
  rclcpp_construct_ring_buffer_->assign(record);
}

void DataContainer::assign_rclcpp_buffer_to_ipb(RclcppBufferToIpb::StdFuncT record)
{
  assert(rclcpp_buffer_to_ipb_.get() != nullptr);
  rclcpp_buffer_to_ipb_->assign(record);
}

void DataContainer::assign_rclcpp_ipb_to_subscription(RclcppIpbToSubscription::StdFuncT record)
{
  assert(rclcpp_ipb_to_subscription_.get() != nullptr);
  rclcpp_ipb_to_subscription_->assign(record);
}

void DataContainer::assign_rmw_implementation(RmwImplementation::StdFuncT record)
{
  assert(rmw_implementation_.get() != nullptr);
  rmw_implementation_->assign(record);
}

bool DataContainer::is_assigned_add_callback_group() const
{
  assert(add_callback_group_.get() != nullptr);
  return add_callback_group_->is_assigned();
}

bool DataContainer::is_assigned_add_callback_group_static_executor() const
{
  assert(add_callback_group_static_executor_.get() != nullptr);
  return add_callback_group_static_executor_->is_assigned();
}

bool DataContainer::is_assigned_callback_group_add_client() const
{
  assert(callback_group_add_client_.get() != nullptr);
  return callback_group_add_client_->is_assigned();
}

bool DataContainer::is_assigned_callback_group_add_service() const
{
  assert(callback_group_add_service_.get() != nullptr);
  return callback_group_add_service_->is_assigned();
}

bool DataContainer::is_assigned_callback_group_add_subscription() const
{
  assert(callback_group_add_subscription_.get() != nullptr);
  return callback_group_add_subscription_->is_assigned();
}

bool DataContainer::is_assigned_callback_group_add_timer() const
{
  assert(callback_group_add_timer_.get() != nullptr);
  return callback_group_add_timer_->is_assigned();
}

bool DataContainer::is_assigned_callback_group_static_executor() const
{
  assert(construct_static_executor_.get() != nullptr);
  return construct_static_executor_->is_assigned();
}

bool DataContainer::is_assigned_construct_executor() const
{
  assert(construct_executor_.get() != nullptr);
  return construct_executor_->is_assigned();
}

bool DataContainer::is_assigned_construct_static_executor() const
{
  //
  assert(callback_group_add_client_.get() != nullptr);
  return callback_group_add_client_->is_assigned();
}

bool DataContainer::is_assigned_rcl_client_init() const
{
  assert(rcl_client_init_.get() != nullptr);
  return rcl_client_init_->is_assigned();
}

bool DataContainer::is_assigned_rcl_init() const
{
  assert(rcl_init_.get() != nullptr);
  return rcl_init_->is_assigned();
}

bool DataContainer::is_assigned_rcl_node_init() const
{
  assert(rcl_node_init_.get() != nullptr);
  return rcl_node_init_->is_assigned();
}

bool DataContainer::is_assigned_rcl_publisher_init() const
{
  assert(rcl_publisher_init_.get() != nullptr);
  return rcl_publisher_init_->is_assigned();
}

bool DataContainer::is_assigned_rcl_service_init() const
{
  assert(rcl_service_init_.get() != nullptr);
  return rcl_service_init_->is_assigned();
}
bool DataContainer::is_assigned_rcl_subscription_init() const
{
  assert(rcl_subscription_init_.get() != nullptr);
  return rcl_subscription_init_->is_assigned();
}

bool DataContainer::is_assigned_rcl_timer_init() const
{
  assert(rcl_timer_init_.get() != nullptr);
  return rcl_timer_init_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_callback_register() const
{
  assert(rclcpp_callback_register_.get() != nullptr);
  return rclcpp_callback_register_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_service_callback_added() const
{
  assert(rclcpp_service_callback_added_.get() != nullptr);
  return rclcpp_service_callback_added_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_subscription_callback_added() const
{
  assert(rclcpp_subscription_callback_added_.get() != nullptr);
  return rclcpp_subscription_callback_added_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_subscription_init() const
{
  assert(rclcpp_subscription_init_.get() != nullptr);
  return rclcpp_subscription_init_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_timer_callback_added() const
{
  assert(rclcpp_timer_callback_added_.get() != nullptr);
  return rclcpp_timer_callback_added_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_timer_link_node() const
{
  assert(rclcpp_timer_link_node_.get() != nullptr);
  return rclcpp_timer_link_node_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_construct_ring_buffer() const
{
  assert(rclcpp_construct_ring_buffer_.get() != nullptr);
  return rclcpp_construct_ring_buffer_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_buffer_to_ipb() const
{
  assert(rclcpp_buffer_to_ipb_.get() != nullptr);
  return rclcpp_buffer_to_ipb_->is_assigned();
}

bool DataContainer::is_assigned_rclcpp_ipb_to_subscription() const
{
  assert(rclcpp_ipb_to_subscription_.get() != nullptr);
  return rclcpp_ipb_to_subscription_->is_assigned();
}

bool DataContainer::is_assigned_rmw_implementation() const
{
  assert(rmw_implementation_.get() != nullptr);
  return rmw_implementation_->is_assigned();
}
