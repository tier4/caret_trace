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

/// @brief Interface class for DataContainer.
class DataContainerInterface
{
public:
  virtual ~DataContainerInterface() {}

  /// @brief record stored data.
  /// @param loop_count  Number of records to be recorded at one time.
  /// @return True if record has finished, false otherwise.
  virtual bool record(uint64_t loop_count = 1) = 0;

  /// @brief transitionto recording state.
  virtual void start_recording() = 0;

  /// @brief Reset recording state.
  virtual void reset() = 0;
};

/// @brief DataContainer implementation class. This class contains all initialization-related data.
class DataContainer : public DataContainerInterface
{
public:
  /// @brief  for add_callback_group trace points.
  using AddCallbackGroup = ContainerTraits<const void *, const void *, const char *, int64_t>;

  /// @brief ContainerTraits for add_callback_group_static_executor trace points.
  using AddCallbackGroupStaticExecutor =
    ContainerTraits<const void *, const void *, const char *, int64_t>;

  /// @brief ContainerTraits for callback_group_add_client trace points.
  using CallbackGroupAddClient = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for callback_group_add_service trace points.
  using CallbackGroupAddService = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for callback_group_add_subscription trace points.
  using CallbackGroupAddSubscription = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for callback_group_add_timer trace points.
  using CallbackGroupAddTimer = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for construct_executor trace points.
  using ConstructExecutor = ContainerTraits<const void *, const char *, int64_t>;

  /// @brief ContainerTraits for construct_static_executor trace points.
  using ConstructStaticExecutor =
    ContainerTraits<const void *, const void *, const char *, int64_t>;

  /// @brief ContainerTraits for rcl_init trace points.
  using RclInit = ContainerTraits<const void *, int64_t>;

  /// @brief ContainerTraits for rcl_node_init trace points.
  using RclNodeInit =
    ContainerTraits<const void *, const void *, const char *, const char *, int64_t>;

  /// @brief ContainerTraits for rcl_subscription_init trace points.
  using RclSubscriptionInit =
    ContainerTraits<const void *, const void *, const void *, const char *, size_t, int64_t>;

  /// @brief ContainerTraits for rclcpp_callback_register trace points.
  using RclcppCallbackRegister = ContainerTraits<const void *, const char *, int64_t>;

  /// @brief ContainerTraits for rclcpp_subscription_init trace points.
  using RclcppSubscriptionInit = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for rclcpp_subscription_callback_added trace points.
  using RclcppSubscriptionCallbackAdded = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for rclcpp_timer_callback_added trace points.
  using RclcppTimerCallbackAdded = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for rclcpp_timer_link_node trace points.
  using RclcppTimerLinkNode = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for rcl_timer_init trace points.
  using RclTimerInit = ContainerTraits<const void *, int64_t, int64_t>;

  /// @brief ContainerTraits for rcl_publisher_init trace points.
  using RclPublisherInit =
    ContainerTraits<const void *, const void *, const void *, const char *, size_t, int64_t>;

  /// @brief ContainerTraits for rcl_client_init trace points.
  using RclClientInit =
    ContainerTraits<const void *, const void *, const void *, const char *, int64_t>;

  /// @brief ContainerTraits for rclcpp_service_callback_added trace points.
  using RclcppServiceCallbackAdded = ContainerTraits<const void *, const char *, int64_t>;

  /// @brief ContainerTraits for rcl_service_init trace points.
  using RclServiceInit =
    ContainerTraits<const void *, const void *, const void *, const char *, int64_t>;

  /// @brief ContainerTraits for rclcpp_construct_ring_buffer trace points.
  using RclcppConstructRingBuffer = ContainerTraits<const void *, int64_t, int64_t>;

  /// @brief ContainerTraits for rclcpp_buffer_to_ipb trace points.
  using RclcppBufferToIpb = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for rclcpp_ipb_to_subscription trace points.
  using RclcppIpbToSubscription = ContainerTraits<const void *, const void *, int64_t>;

  /// @brief ContainerTraits for rmw_implementation trace points.
  using RmwImplementation = ContainerTraits<const char *, int64_t>;

  /// @brief Construct an instance.
  DataContainer();

  /// @brief Construct an instance.
  /// @param add_callback_group Data instance for add_callback_group trace point.
  /// @param add_callback_group_static_executor Data instance for add_callback_group_static_executor
  /// trace point.
  /// @param callback_group_add_client Data instance for callback_group_add_client trace point.
  /// @param callback_group_add_service Data instance for callback_group_add_service trace point.
  /// @param callback_group_add_subscription Data instance for callback_group_add_subscription trace
  /// point.
  /// @param callback_group_add_timer Data instance for callback_group_add_timer trace point.
  /// @param construct_executor Data instance for construct_executor trace point.
  /// @param construct_static_executor Data instance for construct_static_executor trace point.
  /// @param rcl_client_init Data instance for rcl_client_init trace point.
  /// @param rcl_init Data instance for rcl_init trace point.
  /// @param rcl_node_init Data instance for rcl_node_init trace point.
  /// @param rcl_publisher_init Data instance for rcl_publisher_init trace point.
  /// @param rcl_service_init Data instance for rcl_service_init trace point.
  /// @param rcl_subscription_init Data instance for rcl_subscription_init trace point.
  /// @param rcl_timer_init Data instance for rcl_timer_init trace point.
  /// @param rclcpp_callback_register Data instance for rclcpp_callback_register trace point.
  /// @param rclcpp_service_callback_added Data instance for rclcpp_service_callback_added trace
  /// point.
  /// @param rclcpp_subscription_callback_added Data instance for rclcpp_subscription_callback_added
  /// trace point.
  /// @param rclcpp_subscription_init Data instance for rclcpp_subscription_init trace point.
  /// @param rclcpp_timer_callback_added Data instance for rclcpp_timer_callback_added trace point.
  /// @param rclcpp_timer_link_node Data instance for rclcpp_timer_link_node trace point.
  /// @param rmw_implementation Data instance for rmw_implementation trace point.
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
    std::shared_ptr<RclcppConstructRingBuffer::KeysT> rclcpp_construct_ring_buffer,
    std::shared_ptr<RclcppBufferToIpb::KeysT> rclcpp_buffer_to_ipb,
    std::shared_ptr<RclcppIpbToSubscription::KeysT> rclcpp_ipb_to_subscription,
    std::shared_ptr<RmwImplementation::KeysT> rmw_implementation);

  bool record(uint64_t loop_count = 1);

  void start_recording();

  void reset();

  /// @brief Store data for add_callback_group trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_add_callback_group(Args... args)
  {
    return add_callback_group_->store(args...);
  }

  /// @brief Store data for add_callback_group_static_executor trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_add_callback_group_static_executor(Args... args)
  {
    return add_callback_group_static_executor_->store(args...);
  }

  /// @brief Store data for callback_group_add_client trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_callback_group_add_client(Args... args)
  {
    return callback_group_add_client_->store(args...);
  }

  /// @brief Store data for callback_group_add_service trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_callback_group_add_service(Args... args)
  {
    return callback_group_add_service_->store(args...);
  }

  /// @brief Store data for callback_group_add_subscription trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_callback_group_add_subscription(Args... args)
  {
    return callback_group_add_subscription_->store(args...);
  }

  /// @brief Store data for callback_group_add_timer trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_callback_group_add_timer(Args... args)
  {
    return callback_group_add_timer_->store(args...);
  }

  /// @brief Store data for construct_executor trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_construct_executor(Args... args)
  {
    return construct_executor_->store(args...);
  }

  /// @brief Store data for construct_static_executor trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_construct_static_executor(Args... args)
  {
    return construct_static_executor_->store(args...);
  }

  /// @brief Store data for rcl_node_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_node_init(Args... args)
  {
    return rcl_node_init_->store(args...);
  }

  /// @brief Store data for rcl_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_init(Args... args)
  {
    return rcl_init_->store(args...);
  }

  /// @brief Store data for rcl_subscription_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_subscription_init(Args... args)
  {
    return rcl_subscription_init_->store(args...);
  }

  /// @brief Store data for rclcpp_callback_register trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_callback_register(Args... args)
  {
    return rclcpp_callback_register_->store(args...);
  }

  /// @brief Store data for rclcpp_subscription_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_subscription_init(Args... args)
  {
    return rclcpp_subscription_init_->store(args...);
  }

  /// @brief Store data for rclcpp_subscription_callback_added trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_subscription_callback_added(Args... args)
  {
    return rclcpp_subscription_callback_added_->store(args...);
  }

  /// @brief Store data for rclcpp_timer_callback_added trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_timer_callback_added(Args... args)
  {
    return rclcpp_timer_callback_added_->store(args...);
  }

  /// @brief Store data for rclcpp_timer_link_node trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_timer_link_node(Args... args)
  {
    return rclcpp_timer_link_node_->store(args...);
  }

  /// @brief Store data for rcl_timer_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_timer_init(Args... args)
  {
    return rcl_timer_init_->store(args...);
  }

  /// @brief Store data for rcl_publisher_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_publisher_init(Args... args)
  {
    return rcl_publisher_init_->store(args...);
  }

  /// @brief Store data for rcl_client_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_client_init(Args... args)
  {
    return rcl_client_init_->store(args...);
  }

  /// @brief Store data for rclcpp_service_callback_added trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_service_callback_added(Args... args)
  {
    return rclcpp_service_callback_added_->store(args...);
  }

  /// @brief Store data for rcl_service_init trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rcl_service_init(Args... args)
  {
    return rcl_service_init_->store(args...);
  }

  /// @brief Store data for rclcpp_construct_ring_buffer trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_construct_ring_buffer(Args... args)
  {
    return rclcpp_construct_ring_buffer_->store(args...);
  }

  /// @brief Store data for rclcpp_buffer_to_ipb trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_buffer_to_ipb(Args... args)
  {
    return rclcpp_buffer_to_ipb_->store(args...);
  }

  /// @brief Store data for rclcpp_ipb_to_subscription trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rclcpp_ipb_to_subscription(Args... args)
  {
    return rclcpp_ipb_to_subscription_->store(args...);
  }

  /// @brief Store data for rmw_implementation trace points.
  /// @tparam ...Args Data types to be stored.
  /// @param ...args Data to be stored.
  /// @return True, data was stored to pending set.
  /// @return False, data was stored to set.
  template <typename... Args>
  bool store_rmw_implementation(Args... args)
  {
    return rmw_implementation_->store(args...);
  }

  /// @brief Assign recording function for add_callback_group trace points.
  /// @param record recording function.
  void assign_add_callback_group(AddCallbackGroup::StdFuncT record);

  /// @brief Assign recording function for add_callback_group_static_executor trace points.
  /// @param record recording function.
  void assign_add_callback_group_static_executor(AddCallbackGroupStaticExecutor::StdFuncT record);

  /// @brief Assign recording function for callback_group_add_client trace points.
  /// @param record recording function.
  void assign_callback_group_add_client(CallbackGroupAddClient::StdFuncT record);

  /// @brief Assign recording function for callback_group_add_service trace points.
  /// @param record recording function.
  void assign_callback_group_add_service(CallbackGroupAddService::StdFuncT record);

  /// @brief Assign recording function for callback_group_add_subscription trace points.
  /// @param record recording function.
  void assign_callback_group_add_subscription(CallbackGroupAddSubscription::StdFuncT record);

  /// @brief Assign recording function for callback_group_add_timer trace points.
  /// @param record recording function.
  void assign_callback_group_add_timer(CallbackGroupAddTimer::StdFuncT record);

  /// @brief Assign recording function for construct_executor trace points.
  /// @param record recording function.
  void assign_construct_executor(ConstructExecutor::StdFuncT record);

  /// @brief Assign recording function for construct_static_executor trace points.
  /// @param record recording function.
  void assign_construct_static_executor(ConstructStaticExecutor::StdFuncT record);

  /// @brief Assign recording function for rcl_client_init trace points.
  /// @param record recording function.
  void assign_rcl_client_init(RclClientInit::StdFuncT record);

  /// @brief Assign recording function for rcl_init trace points.
  /// @param record recording function.
  void assign_rcl_init(RclInit::StdFuncT record);

  /// @brief Assign recording function for rcl_node_init trace points.
  /// @param record recording function.
  void assign_rcl_node_init(RclNodeInit::StdFuncT record);

  /// @brief Assign recording function for rcl_publisher_init trace points.
  /// @param record recording function.
  void assign_rcl_publisher_init(RclPublisherInit::StdFuncT record);

  /// @brief Assign recording function for rcl_service_init trace points.
  /// @param record recording function.
  void assign_rcl_service_init(RclServiceInit::StdFuncT record);

  /// @brief Assign recording function for rcl_subscription_init trace points.
  /// @param record recording function.
  void assign_rcl_subscription_init(RclSubscriptionInit::StdFuncT record);

  /// @brief Assign recording function for rcl_timer_init trace points.
  /// @param record recording function.
  void assign_rcl_timer_init(RclTimerInit::StdFuncT record);

  /// @brief Assign recording function for rclcpp_callback_register trace points.
  /// @param record recording function.
  void assign_rclcpp_callback_register(RclcppCallbackRegister::StdFuncT record);

  /// @brief Assign recording function for rclcpp_service_callback_added trace points.
  /// @param record recording function.
  void assign_rclcpp_service_callback_added(RclcppServiceCallbackAdded::StdFuncT record);

  /// @brief Assign recording function for rclcpp_subscription_callback_added trace points.
  /// @param record recording function.
  void assign_rclcpp_subscription_callback_added(RclcppSubscriptionCallbackAdded::StdFuncT record);

  /// @brief Assign recording function for rclcpp_subscription_init trace points.
  /// @param record recording function.

  void assign_rclcpp_subscription_init(RclcppSubscriptionInit::StdFuncT record);

  /// @brief Assign recording function for rclcpp_timer_callback_added trace points.
  /// @param record recording function.
  void assign_rclcpp_timer_callback_added(RclcppTimerCallbackAdded::StdFuncT record);

  /// @brief Assign recording function for rclcpp_timer_link_node trace points.
  /// @param record recording function.
  void assign_rclcpp_timer_link_node(RclcppTimerLinkNode::StdFuncT record);

  /// @brief Assign recording function for rclcpp_construct_ring_buffer trace points.
  /// @param record recording function.
  void assign_rclcpp_construct_ring_buffer(RclcppConstructRingBuffer::StdFuncT record);

  /// @brief Assign recording function for rclcpp_buffer_to_ipb trace points.
  /// @param record recording function.
  void assign_rclcpp_buffer_to_ipb(RclcppBufferToIpb::StdFuncT record);

  /// @brief Assign recording function for rclcpp_ipb_to_subscription trace points.
  /// @param record recording function.
  void assign_rclcpp_ipb_to_subscription(RclcppIpbToSubscription::StdFuncT record);

  /// @brief Assign recording function for rmw_implementation trace points.
  /// @param record recording function.
  void assign_rmw_implementation(RmwImplementation::StdFuncT record);

  /// @brief Check whether recording function for add_callback_group trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_add_callback_group() const;

  /// @brief Check whether recording function for add_callback_group_static_executor trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_add_callback_group_static_executor() const;

  /// @brief Check whether recording function for callback_group_add_client trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_callback_group_add_client() const;

  /// @brief Check whether recording function for callback_group_add_service trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_callback_group_add_service() const;

  /// @brief Check whether recording function for callback_group_add_subscription trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_callback_group_add_subscription() const;

  /// @brief Check whether recording function for callback_group_add_timer trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_callback_group_add_timer() const;

  /// @brief Check whether recording function for callback_group_static_executor trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_callback_group_static_executor() const;

  /// @brief Check whether recording function for construct_executor trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_construct_executor() const;

  /// @brief Check whether recording function for construct_static_executor trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_construct_static_executor() const;

  /// @brief Check whether recording function for rcl_client_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_client_init() const;

  /// @brief Check whether recording function for rcl_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_init() const;

  /// @brief Check whether recording function for rcl_node_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_node_init() const;

  /// @brief Check whether recording function for rcl_publisher_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_publisher_init() const;

  /// @brief Check whether recording function for rcl_service_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_service_init() const;

  /// @brief Check whether recording function for rcl_subscription_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_subscription_init() const;

  /// @brief Check whether recording function for rcl_timer_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rcl_timer_init() const;

  /// @brief Check whether recording function for rclcpp_callback_register trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_callback_register() const;

  /// @brief Check whether recording function for rclcpp_service_callback_added trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_service_callback_added() const;

  /// @brief Check whether recording function for rclcpp_subscription_callback_added trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_subscription_callback_added() const;

  /// @brief Check whether recording function for rclcpp_subscription_init trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_subscription_init() const;

  /// @brief Check whether recording function for rclcpp_timer_callback_added trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_timer_callback_added() const;

  /// @brief Check whether recording function for rclcpp_timer_link_node trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_timer_link_node() const;

  /// @brief Check whether recording function for rclcpp_construct_ring_buffer trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_construct_ring_buffer() const;

  /// @brief Check whether recording function for rclcpp_buffer_to_ipb trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_buffer_to_ipb() const;

  /// @brief Check whether recording function for rclcpp_ipb_to_subscription trace point is
  /// assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rclcpp_ipb_to_subscription() const;

  /// @brief Check whether recording function for rmw_implementation trace point is assigned.
  /// @return True if function is assigned, false otherwise.
  bool is_assigned_rmw_implementation() const;

  /// @brief Get trace point names.
  /// @return Trace point names.
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
  std::shared_ptr<RclcppConstructRingBuffer::KeysT> rclcpp_construct_ring_buffer_;
  std::shared_ptr<RclcppBufferToIpb::KeysT> rclcpp_buffer_to_ipb_;
  std::shared_ptr<RclcppIpbToSubscription::KeysT> rclcpp_ipb_to_subscription_;
  std::shared_ptr<RmwImplementation::KeysT> rmw_implementation_;

  std::shared_ptr<DataRecorder> recorder_;
};

#endif  // CARET_TRACE__DATA_CONTAINER_HPP_
#define CARET_TRACE__DATA_CONTAINER_HPP_
