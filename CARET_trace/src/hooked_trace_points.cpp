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

#include "rcl/rcl.h"
#include "rcutils/shared_library.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include <dlfcn.h>

#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#define TRACEPOINT_DEFINE
#include "caret_trace/context.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/tp.h"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/get_env.hpp"
#include "rcpputils/shared_library.hpp"

// #define DEBUG_OUTPUT

#include "dds/ddsi/ddsi_serdata.h"
#include "fastdds/rtps/common/CacheChange.h"

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

#include "caret_trace/keys_set.hpp"

#define SYMBOL_CONCAT_2(x, y) x##y
#define SYMBOL_CONCAT_3(x, y, z) x##y##z

static thread_local const void * serialized_message_addr;
extern thread_local bool trace_filter_is_rcl_publish_recorded;

// Declare a prototype in order to use the functions implemented in cyclonedds.
rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid);

// cspell: ignore WRITECDR
namespace CYCLONEDDS
{
void * DDS_WRITE_IMPL;
void * DDS_WRITECDR_IMPL;
}  // namespace CYCLONEDDS

// For FastDDS
namespace FASTDDS
{
static void * SET_FRAGMENTS;
}

namespace rclcpp
{
namespace executors
{
class StaticSingleThreadedExecutorPublic : public rclcpp::Executor
{
public:
  explicit StaticSingleThreadedExecutorPublic(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  RCLCPP_PUBLIC
  virtual ~StaticSingleThreadedExecutorPublic();

  RCLCPP_PUBLIC
  void spin() override;

  RCLCPP_PUBLIC
  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  RCLCPP_PUBLIC
  void spin_all(std::chrono::nanoseconds max_duration) override;

  RCLCPP_PUBLIC
  void add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups() override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups() override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes()
    override;

  // protected:
  RCLCPP_PUBLIC
  bool execute_ready_executables(bool spin_once = false);

  RCLCPP_PUBLIC
  void spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

  RCLCPP_PUBLIC
  void spin_once_impl(std::chrono::nanoseconds timeout) override;

  // private:
  // RCLCPP_DISABLE_COPY(StaticSingleThreadedExecutor)

  StaticExecutorEntitiesCollector::SharedPtr entities_collector_;
};

}  // namespace executors
}  // namespace rclcpp

extern "C" {
// Get symbols from the DDS shared library
// The dds-related-symbol, which is set by an environment variable, cannot be obtained by dlsym.
// It is necessary to hook load_library and specify the library to be loaded to get them.
// std::shared_ptr<rcpputils::SharedLibrary> _Z12load_libraryv()
void update_dds_function_addr()
{
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  auto now = clock.now();

  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);

  std::string env_var;
  try {
    env_var = rcpputils::get_env_var("RMW_IMPLEMENTATION");
  } catch (const std::exception & e) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to fetch RMW_IMPLEMENTATION "
      "from environment due to %s",
      e.what());
  }

  if (env_var.empty()) {
    env_var = STRINGIFY(DEFAULT_RMW_IMPLEMENTATION);
  }

  // ref. rosidl_typesupport/rosidl_typesupport_cpp/src/type_support_dispatch.hpp
  std::string library_name;
  try {
    library_name = rcpputils::get_platform_library_name(env_var);
  } catch (const std::runtime_error & e) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Failed to compute library name for '%s' due to %s", env_var.c_str(), e.what());
  }
  rcpputils::SharedLibrary * lib = nullptr;
  try {
    lib = new rcpputils::SharedLibrary(library_name);
  } catch (const std::runtime_error & e) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Could not load library %s: %s", library_name.c_str(), e.what());
  } catch (const std::bad_alloc & e) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Could not load library %s: %s", library_name.c_str(), e.what());
  }

  static auto record = [](const char * rmw_implementation, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, rmw_implementation, rmw_implementation, init_time);
  };

  if (!data_container.is_assigned_rmw_implementation()) {
    data_container.assign_rmw_implementation(record);
  }

  data_container.store_rmw_implementation(env_var.c_str(), now);

  record(env_var.c_str(), now);

  if (env_var == "rmw_fastrtps_cpp") {
    // clang-format off
    // // rmw_fastrtps_shared_cpp::TypeSupport::serialize(void*,
    // eprosima::fastrtps::rtps::SerializedPayload_t*)  // NOLINT
    FASTDDS::SET_FRAGMENTS = lib->get_symbol(
      "_ZN8eprosima8fastrtps4rtps13WriterHistory13set_fragmentsEPNS1_13CacheChange_tE");  // NOLINT
    // clang-format on
  } else if (env_var == "rmw_cyclonedds_cpp") {
    CYCLONEDDS::DDS_WRITE_IMPL = lib->get_symbol("dds_write_impl");
    CYCLONEDDS::DDS_WRITECDR_IMPL = lib->get_symbol("dds_writecdr_impl");
  }
}

// clang-format off

// for cyclonedds
// bind : &ros_message -> source_timestamp
int dds_write_impl(void * wr, void * data, long tstamp, int action)  // NOLINT
{
  static auto & context = Singleton<Context>::get_instance();
  using functionT = int (*)(void *, void *, long, int);  // NOLINT

  // clang-format on
  if (CYCLONEDDS::DDS_WRITE_IMPL == nullptr) {
    update_dds_function_addr();
  }
  int dds_return = ((functionT)CYCLONEDDS::DDS_WRITE_IMPL)(wr, data, tstamp, action);

  if (context.is_recording_allowed() && trace_filter_is_rcl_publish_recorded) {
    tracepoint(TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, data, tstamp);
#ifdef DEBUG_OUTPUT
    std::cerr << "dds_bind_addr_to_stamp," << data << "," << tstamp << std::endl;
#endif
  }
  return dds_return;
}

// for cyclonedds
// bind : &ros_message -> source_timestamp
// cspell: ignore ddsi, serdata, dinp
int dds_writecdr_impl(void * wr, void * xp, struct ddsi_serdata * dinp, bool flush)  // NOLINT
{
  static auto & context = Singleton<Context>::get_instance();
  using functionT = int (*)(void *, void *, struct ddsi_serdata *, bool);  // NOLINT

  // clang-format on
  if (CYCLONEDDS::DDS_WRITECDR_IMPL == nullptr) {
    update_dds_function_addr();
  }
  int dds_return = ((functionT)CYCLONEDDS::DDS_WRITECDR_IMPL)(wr, xp, dinp, flush);

  if (context.is_recording_allowed()) {
    tracepoint(
      TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, serialized_message_addr, dinp->timestamp.v);
#ifdef DEBUG_OUTPUT
    std::cerr << "dds_bind_addr_to_stamp," << data << "," << tstamp << std::endl;
#endif
  }
  return dds_return;
}

// for fastdds
// bind : &ros_message -> source_timestamp
void _ZN8eprosima8fastrtps4rtps13WriterHistory13set_fragmentsEPNS1_13CacheChange_tE(
  void * obj, eprosima::fastrtps::rtps::CacheChange_t * change)
{
  static auto & context = Singleton<Context>::get_instance();

  using functionT = void (*)(void *, eprosima::fastrtps::rtps::CacheChange_t *);
  if (FASTDDS::SET_FRAGMENTS == nullptr) {
    update_dds_function_addr();
  }
  ((functionT)FASTDDS::SET_FRAGMENTS)(obj, change);
  if (context.is_recording_allowed()) {
    tracepoint(
      TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, nullptr, change->sourceTimestamp.to_ns());
#ifdef DEBUG_OUTPUT
    std::cerr << "dds_bind_addr_to_stamp," << change->sourceTimestamp.to_ns() << std::endl;
#endif
  }
}

// for rmw_cyclonedds_cpp
// store message address in tread local memory
rmw_ret_t rmw_publish_serialized_message(
  const rmw_publisher_t * publisher, const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  serialized_message_addr = static_cast<const void *>(serialized_message);
  using functionT = rmw_ret_t (*)(const void *, const void *, const void *);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  rmw_ret_t ret = ((functionT)orig_func)(publisher, serialized_message, allocation);
  return ret;
}

// rclcpp::executors::SingleThreadedExecutor::SingleThreadedExecutor(rclcpp::ExecutorOptions const&)
void _ZN6rclcpp9executors22SingleThreadedExecutorC1ERKNS_15ExecutorOptionsE(
  void * obj, const void * option)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](const void * obj, const char * executor_type_name, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, construct_executor, obj, executor_type_name, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "construct_executor," << executor_type_name << "," << obj << std::endl;
#endif
  };
  auto now = clock.now();
  using functionT = void (*)(void *, const void *);
  ((functionT)orig_func)(obj, option);

  const std::string executor_type_name = "single_threaded_executor";

  if (!data_container.is_assigned_construct_executor()) {
    data_container.assign_construct_executor(record);
  }

  data_container.store_construct_executor(obj, executor_type_name.c_str(), now);
  record(obj, executor_type_name.c_str(), now);
}

// rclcpp::executors::MultiThreadedExecutor::MultiThreadedExecutor(
// rclcpp::ExecutorOptions const&, unsigned long, bool,
// std::chrono::duration<long, std::ratio<1l, 1000000000l> >)
void SYMBOL_CONCAT_2(
  _ZN6rclcpp9executors21MultiThreadedExecutor,
  C1ERKNS_15ExecutorOptionsEmbNSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE)(
  void * obj, const void * option, size_t number_of_thread, bool yield_before_execute,
  const void * timeout)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto record = [](const void * obj, const char * executor_type_name, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, construct_executor, obj, executor_type_name, init_time);
#ifdef DEBUG_OUTPUT
    std::cerr << "construct_executor," << executor_type_name << "," << obj << std::endl;
#endif
  };
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  auto now = clock.now();

  static auto & data_container = context.get_data_container();
  const std::string executor_type_name = "multi_threaded_executor";

  using functionT = void (*)(void *, const void *, size_t, bool, const void *);
  ((functionT)orig_func)(obj, option, number_of_thread, yield_before_execute, timeout);

  if (!data_container.is_assigned_construct_executor()) {
    data_container.assign_construct_executor(record);
  }

  data_container.store_construct_executor(obj, executor_type_name.c_str(), now);
  record(obj, executor_type_name.c_str(), now);
}

// rclcpp::executors::StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(
// rclcpp::ExecutorOptions const&)
void _ZN6rclcpp9executors28StaticSingleThreadedExecutorC1ERKNS_15ExecutorOptionsE(
  void * obj, const void * option)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](
                         const void * obj, const void * entities_collector_ptr,
                         const char * executor_type, int64_t init_time) {
    tracepoint(
      TRACEPOINT_PROVIDER, construct_static_executor, obj, entities_collector_ptr, executor_type,
      init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "construct_static_executor,"
              << "static_single_threaded_executor"
              << "," << obj << "," << entities_collector_ptr << std::endl;
#endif
  };
  auto now = clock.now();

  using functionT = void (*)(void *, const void *);
  ((functionT)orig_func)(obj, option);

  using StaticSingleThreadedExecutorPublic = rclcpp::executors::StaticSingleThreadedExecutorPublic;
  auto exec_ptr = reinterpret_cast<StaticSingleThreadedExecutorPublic *>(obj);

  if (!data_container.is_assigned_construct_static_executor()) {
    data_container.assign_construct_static_executor(record);
  }

  auto entities_collector_ptr = static_cast<const void *>(exec_ptr->entities_collector_.get());
  data_container.store_add_callback_group_static_executor(
    obj, entities_collector_ptr, "static_single_threaded_executor", now);
  record(obj, entities_collector_ptr, "static_single_threaded_executor", now);
}

// rclcpp::Executor::add_callback_group_to_map(
//   std::shared_ptr<rclcpp::CallbackGroup>,
//   std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>,
//   std::map<std::weak_ptr<rclcpp::CallbackGroup>,
//   std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface>,
//   std::owner_less<std::weak_ptr<rclcpp::CallbackGroup> >,
//   std::allocator<std::pair<std::weak_ptr<rclcpp::CallbackGroup> const,
//   std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> > > >&,
//   bool)
void SYMBOL_CONCAT_3(
  _ZN6rclcpp8Executor25add_callback_group_to_map,
  ESt10shared_ptrINS_13CallbackGroupEES1_INS_15node_interfaces17NodeBaseInterface,
  EERSt3mapISt8weak_ptrIS2_ES8_IS5_ESt10owner_lessIS9_ESaISt4pairIKS9_SA_EEEb)(
  void * obj, rclcpp::CallbackGroup::SharedPtr group_ptr, const void * node_ptr,
  const void * weak_groups_to_nodes, bool notify)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record =
    [](const void * obj, const void * group_addr, const char * group_type_name, int64_t init_time) {
      tracepoint(
        TRACEPOINT_PROVIDER, add_callback_group, obj, group_addr, group_type_name, init_time);

#ifdef DEBUG_OUTPUT
      std::cerr << "add_callback_group," << obj << "," << group_addr << "," << group_type_name
                << std::endl;
#endif
    };
  auto now = clock.now();

  using functionT =
    void (*)(void *, rclcpp::CallbackGroup::SharedPtr, const void *, const void *, bool);
  auto group_addr = static_cast<const void *>(group_ptr.get());

  ((functionT)orig_func)(obj, group_ptr, node_ptr, weak_groups_to_nodes, notify);

  if (!data_container.is_assigned_add_callback_group()) {
    data_container.assign_add_callback_group(record);
  }

  static KeysSet<void *, void *, void *> recorded_args;

  auto node_ptr_ = const_cast<void *>(node_ptr);
  auto group_addr_ = const_cast<void *>(group_addr);

  std::string group_type_name = "unknown";
  auto group_type = group_ptr->type();
  if (group_type == rclcpp::CallbackGroupType::MutuallyExclusive) {
    group_type_name = "mutually_exclusive";
  } else if (group_type == rclcpp::CallbackGroupType::Reentrant) {
    group_type_name = "reentrant";
  }

  data_container.store_add_callback_group(obj, group_addr, group_type_name.c_str(), now);
  if (!recorded_args.has(obj, group_addr_, node_ptr_)) {
    recorded_args.insert(obj, group_addr_, node_ptr_);

    record(obj, group_addr, group_type_name.c_str(), now);
  }
}

bool SYMBOL_CONCAT_3(
  _ZN6rclcpp9executors31StaticExecutorEntitiesCollector18add_callback_groupESt10shared_ptr,
  INS_13CallbackGroupEES2_INS_15node_interfaces17NodeBaseInterface,
  EERSt3mapISt8weak_ptrIS3_ES9_IS6_ESt10owner_lessISA_ESaISt4pairIKSA_SB_EEE)(
  void * obj, rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = bool (*)(
    void *, rclcpp::CallbackGroup::SharedPtr, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record =
    [](const void * obj, const void * group_addr, const char * group_type_name, int64_t init_time) {
      tracepoint(
        TRACEPOINT_PROVIDER, add_callback_group_static_executor, obj, group_addr, group_type_name,
        init_time);

#ifdef DEBUG_OUTPUT
      std::cerr << "add_callback_group_static_executor," << obj << "," << group_addr << ","
                << group_type_name << std::endl;
#endif
    };

  auto now = clock.now();
  auto group_addr = static_cast<const void *>(group_ptr.get());
  std::string group_type_name = "unknown";
  auto group_type = group_ptr->type();
  if (group_type == rclcpp::CallbackGroupType::MutuallyExclusive) {
    group_type_name = "mutually_exclusive";
  } else if (group_type == rclcpp::CallbackGroupType::Reentrant) {
    group_type_name = "reentrant";
  }

  auto ret = ((functionT)orig_func)(obj, group_ptr, node_ptr, weak_groups_to_nodes);

  if (!data_container.is_assigned_add_callback_group_static_executor()) {
    data_container.assign_add_callback_group_static_executor(record);
  }

  data_container.store_add_callback_group_static_executor(
    obj, group_addr, group_type_name.c_str(), now);
  record(obj, group_addr, group_type_name.c_str(), now);

  return ret;
}

//  rclcpp::CallbackGroup::add_timer(std::shared_ptr<rclcpp::TimerBase>)
void _ZN6rclcpp13CallbackGroup9add_timerESt10shared_ptrINS_9TimerBaseEE(
  // ok
  void * obj, const rclcpp::TimerBase::SharedPtr timer_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::TimerBase::SharedPtr);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](const void * obj, const void * timer_handle, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, callback_group_add_timer, obj, timer_handle, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "callback_group_add_timer," << obj << "," << timer_handle << std::endl;
#endif
  };

  auto now = clock.now();
  auto timer_handle = static_cast<const void *>(timer_ptr->get_timer_handle().get());
  ((functionT)orig_func)(obj, timer_ptr);

  if (!data_container.is_assigned_callback_group_add_timer()) {
    data_container.assign_callback_group_add_timer(record);
  }

  data_container.store_callback_group_add_timer(obj, timer_handle, now);
  record(obj, timer_handle, now);
}

// rclcpp::CallbackGroup::add_subscription(std::shared_ptr<rclcpp::SubscriptionBase>)
void _ZN6rclcpp13CallbackGroup16add_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
  void * obj, const rclcpp::SubscriptionBase::SharedPtr subscription_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::SubscriptionBase::SharedPtr);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](const void * obj, const void * subscription_handle, int64_t init_time) {
    tracepoint(
      TRACEPOINT_PROVIDER, callback_group_add_subscription, obj, subscription_handle, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "callback_group_add_subscription," << obj << "," << subscription_handle
              << std::endl;
#endif
  };

  auto now = clock.now();
  auto subscription_handle =
    static_cast<const void *>(subscription_ptr->get_subscription_handle().get());
  ((functionT)orig_func)(obj, subscription_ptr);

  if (!data_container.is_assigned_callback_group_add_subscription()) {
    data_container.assign_callback_group_add_subscription(record);
  }

  data_container.store_callback_group_add_subscription(obj, subscription_handle, now);
  record(obj, subscription_handle, now);
}

// rclcpp::CallbackGroup::add_service(std::shared_ptr<rclcpp::ServiceBase>)
void _ZN6rclcpp13CallbackGroup11add_serviceESt10shared_ptrINS_11ServiceBaseEE(
  void * obj, const rclcpp::ServiceBase::SharedPtr service_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::ServiceBase::SharedPtr);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](const void * obj, const void * service_handle, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, callback_group_add_service, obj, service_handle, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "callback_group_add_service," << obj << "," << service_handle << std::endl;
#endif
  };

  auto now = clock.now();
  auto service_handle = static_cast<const void *>(service_ptr->get_service_handle().get());
  ((functionT)orig_func)(obj, service_ptr);

  if (!data_container.is_assigned_callback_group_add_service()) {
    data_container.assign_callback_group_add_service(record);
  }

  data_container.store_callback_group_add_service(obj, service_handle, now);
  record(obj, service_handle, now);
}

// rclcpp::CallbackGroup::add_client(std::shared_ptr<rclcpp::ClientBase>)
void _ZN6rclcpp13CallbackGroup10add_clientESt10shared_ptrINS_10ClientBaseEE(
  void * obj, const rclcpp::ClientBase::SharedPtr client_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::ClientBase::SharedPtr);
  static auto & context = Singleton<Context>::get_instance();
  static auto & clock = context.get_clock();
  static auto & data_container = context.get_data_container();
  static auto record = [](const void * obj, const void * client_handle, int64_t init_time) {
    tracepoint(TRACEPOINT_PROVIDER, callback_group_add_client, obj, client_handle, init_time);

#ifdef DEBUG_OUTPUT
    std::cerr << "callback_group_add_client," << obj << "," << client_handle << std::endl;
#endif
  };

  auto now = clock.now();
  auto client_handle = static_cast<const void *>(client_ptr->get_client_handle().get());
  ((functionT)orig_func)(obj, client_ptr);

  if (!data_container.is_assigned_callback_group_add_client()) {
    data_container.assign_callback_group_add_client(record);
  }

  data_container.store_callback_group_add_client(obj, client_handle, now);
  record(obj, client_handle, now);
}
}
