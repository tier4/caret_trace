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

#include "caret_trace/container_traits.hpp"
#include "caret_trace/data_recorder.hpp"
#include "caret_trace/recordable_data.hpp"

#include <functional>
#include <initializer_list>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#ifndef CARET_TRACE__DATA_CONTAINER_HPP_

class DataContainerInterface
{
public:
  virtual ~DataContainerInterface() {}
  virtual bool record(uint64_t loop_count = 1) = 0;
  virtual void reset() = 0;
};

class DataContainer : public DataContainerInterface
{
public:
  using AddCallbackGroup = ContainerTraits<const void *, const void *, const char *>;
  using AddCallbackGroupStaticExecutor = ContainerTraits<const void *, const void *, const char *>;
  using CallbackGroupAddClient = ContainerTraits<const void *, const void *>;
  using CallbackGroupAddService = ContainerTraits<const void *, const void *>;
  using CallbackGroupAddStaticExecutor = ContainerTraits<const void *, const void *>;
  using CallbackGroupAddSubscription = ContainerTraits<const void *, const void *>;
  using CallbackGroupAddTimer = ContainerTraits<const void *, const void *>;
  using ConstructExecutor = ContainerTraits<const void *, const char *>;
  using ConstructStaticExecutor = ContainerTraits<const void *, const void *, const char *>;

  using RclInit = ContainerTraits<const void *>;
  using RclNodeInit = ContainerTraits<const void *, const void *, const char *, const char *>;
  using RclSubscriptionInit =
    ContainerTraits<const void *, const void *, const void *, const char *, size_t>;
  using RclcppCallbackRegister = ContainerTraits<const void *, const char *>;
  using RclcppSubscriptionInit = ContainerTraits<const void *, const void *>;
  using RclcppSubscriptionCallbackAdded = ContainerTraits<const void *, const void *>;
  using RclcppTimerCallbackAdded = ContainerTraits<const void *, const void *>;
  using RclcppTimerLinkNode = ContainerTraits<const void *, const void *>;
  using RclTimerInit = ContainerTraits<const void *, int64_t>;
  using RclPublisherInit =
    ContainerTraits<const void *, const void *, const void *, const char *, size_t>;
  using RclClientInit = ContainerTraits<const void *, const void *, const void *, const char *>;
  using RclcppServiceCallbackAdded = ContainerTraits<const void *, const void *>;
  using RclServiceInit = ContainerTraits<const void *, const void *, const void *, const char *>;
  using RmwImplementation = ContainerTraits<const char *>;

  DataContainer();

  DataContainer(
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
    std::shared_ptr<RmwImplementation::KeysT> rmw_implementation);

  /**
   * @brief record stored data.
   *
   * @param loop_count loop number to record data.
   * @return true finished to record.
   * @return false recording.
   */
  bool record(uint64_t loop_count = 1);

  void reset();

  template <typename... Args>
  bool store_add_callback_group(Args... args)
  {
    return add_callback_group_->store(args...);
  }

  template <typename... Args>
  bool store_add_callback_group_static_executor(Args... args)
  {
    return add_callback_group_static_executor_->store(args...);
  }

  template <typename... Args>
  bool store_callback_group_add_client(Args... args)
  {
    return callback_group_add_client_->store(args...);
  }

  template <typename... Args>
  bool store_callback_group_add_service(Args... args)
  {
    return callback_group_add_service_->store(args...);
  }

  template <typename... Args>
  bool store_callback_group_add_subscription(Args... args)
  {
    return callback_group_add_subscription_->store(args...);
  }

  template <typename... Args>
  bool store_callback_group_add_timer(Args... args)
  {
    return callback_group_add_timer_->store(args...);
  }

  template <typename... Args>
  bool store_construct_executor(Args... args)
  {
    return construct_executor_->store(args...);
  }

  template <typename... Args>
  bool store_construct_static_executor(Args... args)
  {
    return construct_static_executor_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_node_init(Args... args)
  {
    return rcl_node_init_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_init(Args... args)
  {
    return rcl_init_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_subscription_init(Args... args)
  {
    return rcl_subscription_init_->store(args...);
  }

  template <typename... Args>
  bool store_rclcpp_callback_register(Args... args)
  {
    return rclcpp_callback_register_->store(args...);
  }

  template <typename... Args>
  bool store_rclcpp_subscription_init(Args... args)
  {
    return rclcpp_subscription_init_->store(args...);
  }

  template <typename... Args>
  bool store_rclcpp_subscription_callback_added(Args... args)
  {
    return rclcpp_subscription_callback_added_->store(args...);
  }

  template <typename... Args>
  bool store_rclcpp_timer_callback_added(Args... args)
  {
    return rclcpp_timer_callback_added_->store(args...);
  }

  template <typename... Args>
  bool store_rclcpp_timer_link_node(Args... args)
  {
    return rclcpp_timer_link_node_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_timer_init(Args... args)
  {
    return rcl_timer_init_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_publisher_init(Args... args)
  {
    return rcl_publisher_init_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_client_init(Args... args)
  {
    return rcl_client_init_->store(args...);
  }

  template <typename... Args>
  bool store_rclcpp_service_callback_added(Args... args)
  {
    return rclcpp_service_callback_added_->store(args...);
  }

  template <typename... Args>
  bool store_rcl_service_init(Args... args)
  {
    return rcl_service_init_->store(args...);
  }

  template <typename... Args>
  bool store_rmw_implementation(Args... args)
  {
    return rmw_implementation_->store(args...);
  }

  void assign_add_callback_group(AddCallbackGroup::StdFuncT record);
  void assign_add_callback_group_static_executor(AddCallbackGroupStaticExecutor::StdFuncT record);
  void assign_callback_group_add_client(CallbackGroupAddClient::StdFuncT record);
  void assign_callback_group_add_service(CallbackGroupAddService::StdFuncT record);
  void assign_callback_group_add_subscription(CallbackGroupAddSubscription::StdFuncT record);
  void assign_callback_group_add_timer(CallbackGroupAddTimer::StdFuncT record);
  void assign_construct_executor(ConstructExecutor::StdFuncT record);
  void assign_construct_static_executor(ConstructStaticExecutor::StdFuncT record);
  void assign_rcl_client_init(RclClientInit::StdFuncT record);
  void assign_rcl_init(RclInit::StdFuncT record);
  void assign_rcl_node_init(RclNodeInit::StdFuncT record);
  void assign_rcl_publisher_init(RclPublisherInit::StdFuncT record);
  void assign_rcl_service_init(RclServiceInit::StdFuncT record);
  void assign_rcl_subscription_init(RclSubscriptionInit::StdFuncT record);
  void assign_rcl_timer_init(RclTimerInit::StdFuncT record);
  void assign_rclcpp_callback_register(RclcppCallbackRegister::StdFuncT record);
  void assign_rclcpp_service_callback_added(RclcppServiceCallbackAdded::StdFuncT record);
  void assign_rclcpp_subscription_callback_added(RclcppSubscriptionCallbackAdded::StdFuncT record);
  void assign_rclcpp_subscription_init(RclcppSubscriptionInit::StdFuncT record);
  void assign_rclcpp_timer_callback_added(RclcppTimerCallbackAdded::StdFuncT record);
  void assign_rclcpp_timer_link_node(RclcppTimerLinkNode::StdFuncT record);
  void assign_rmw_implementation(RmwImplementation::StdFuncT record);

  bool is_assigned_add_callback_group() const;
  bool is_assigned_add_callback_group_static_executor() const;
  bool is_assigned_callback_group_add_client() const;
  bool is_assigned_callback_group_add_service() const;
  bool is_assigned_callback_group_add_subscription() const;
  bool is_assigned_callback_group_add_timer() const;
  bool is_assigned_callback_group_static_executor() const;
  bool is_assigned_construct_executor() const;
  bool is_assigned_construct_static_executor() const;
  bool is_assigned_rcl_client_init() const;
  bool is_assigned_rcl_init() const;
  bool is_assigned_rcl_node_init() const;
  bool is_assigned_rcl_publisher_init() const;
  bool is_assigned_rcl_service_init() const;
  bool is_assigned_rcl_subscription_init() const;
  bool is_assigned_rcl_timer_init() const;
  bool is_assigned_rclcpp_callback_register() const;
  bool is_assigned_rclcpp_service_callback_added() const;
  bool is_assigned_rclcpp_subscription_callback_added() const;
  bool is_assigned_rclcpp_subscription_init() const;
  bool is_assigned_rclcpp_timer_callback_added() const;
  bool is_assigned_rclcpp_timer_link_node() const;
  bool is_assigned_rmw_implementation() const;

  std::vector<std::string> trace_points() const;

private:
  mutable std::shared_mutex mutex_;

  std::shared_ptr<AddCallbackGroup::KeysT> add_callback_group_;
  std::shared_ptr<AddCallbackGroupStaticExecutor::KeysT> add_callback_group_static_executor_;
  std::shared_ptr<CallbackGroupAddClient::KeysT> callback_group_add_client_;
  std::shared_ptr<CallbackGroupAddService::KeysT> callback_group_add_service_;
  std::shared_ptr<CallbackGroupAddSubscription::KeysT> callback_group_add_subscription_;
  std::shared_ptr<CallbackGroupAddTimer::KeysT> callback_group_add_timer_;
  std::shared_ptr<ConstructExecutor::KeysT> construct_executor_;
  std::shared_ptr<ConstructStaticExecutor::KeysT> construct_static_executor_;
  std::shared_ptr<RclClientInit::KeysT> rcl_client_init_;
  std::shared_ptr<RclInit::KeysT> rcl_init_;
  std::shared_ptr<RclNodeInit::KeysT> rcl_node_init_;
  std::shared_ptr<RclPublisherInit::KeysT> rcl_publisher_init_;
  std::shared_ptr<RclServiceInit::KeysT> rcl_service_init_;
  std::shared_ptr<RclSubscriptionInit::KeysT> rcl_subscription_init_;
  std::shared_ptr<RclTimerInit::KeysT> rcl_timer_init_;
  std::shared_ptr<RclcppCallbackRegister::KeysT> rclcpp_callback_register_;
  std::shared_ptr<RclcppServiceCallbackAdded::KeysT> rclcpp_service_callback_added_;
  std::shared_ptr<RclcppSubscriptionCallbackAdded::KeysT> rclcpp_subscription_callback_added_;
  std::shared_ptr<RclcppSubscriptionInit::KeysT> rclcpp_subscription_init_;
  std::shared_ptr<RclcppTimerCallbackAdded::KeysT> rclcpp_timer_callback_added_;
  std::shared_ptr<RclcppTimerLinkNode::KeysT> rclcpp_timer_link_node_;
  std::shared_ptr<RmwImplementation::KeysT> rmw_implementation_;

  std::shared_ptr<DataRecorder> recorder_;
};

#endif  // CARET_TRACE__DATA_CONTAINER_HPP_
#define CARET_TRACE__DATA_CONTAINER_HPP_
