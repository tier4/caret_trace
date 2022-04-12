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


#include <dlfcn.h>

#include <iostream>
#include <memory>
#include <unordered_set>
#include <iomanip>
#include <string>
#include <mutex>
#include <vector>

#include "rcl/rcl.h"
#include "rmw/rmw.h"
#include "rmw/event.h"
#include "rcutils/shared_library.h"

#define TRACEPOINT_DEFINE
#include "caret_trace/tp.h"

#include "rcpputils/shared_library.hpp"
#include "rcpputils/get_env.hpp"
#include "rclcpp/rclcpp.hpp"

// #define DEBUG_OUTPUT

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

// for fastrtps
#include "fastdds/rtps/common/WriteParams.h"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"

// for cyclonedds
#include "dds/dds.h"

#define SYMBOL_CONCAT_2(x, y)  x ## y
#define SYMBOL_CONCAT_3(x, y, z)  x ## y ## z


// Declare a prototype in order to use the functions implemented in cyclonedds.
rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid);

namespace CYCLONEDDS
{
void * DDS_WRITE_IMPL;
static dds_listener_t * LISTENER;
static uint8_t DUMMY_BUF[] = {0};  // Dummy buffer for retrieving message info
}

// fortrtps用
namespace FASTDDS
{
static void * ON_DATA_AVAILABLE;
static void * SERIALIZE;
}


namespace rclcpp
{
namespace executors
{
class StaticSingleThreadedExecutorPublic : public rclcpp::Executor
{
public:
  // RCLCPP_SMART_PTR_DEFINITIONS(StaticSingleThreadedExecutorStaticSingleThreadedExecutorPublic)
  RCLCPP_PUBLIC
  explicit StaticSingleThreadedExecutorPublic(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  RCLCPP_PUBLIC
  virtual ~StaticSingleThreadedExecutorPublic();

  RCLCPP_PUBLIC
  void
  spin() override;

  RCLCPP_PUBLIC
  void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  RCLCPP_PUBLIC
  void
  spin_all(std::chrono::nanoseconds max_duration) override;

  RCLCPP_PUBLIC
  void
  add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true) override;

  RCLCPP_PUBLIC
  void
  remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    bool notify = true) override;

  RCLCPP_PUBLIC
  void
  add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true) override;

  RCLCPP_PUBLIC
  void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void
  remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true) override;

  RCLCPP_PUBLIC
  void
  remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups() override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups() override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes() override;

// protected:
  RCLCPP_PUBLIC
  bool
  execute_ready_executables(bool spin_once = false);

  RCLCPP_PUBLIC
  void
  spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

  RCLCPP_PUBLIC
  void
  spin_once_impl(std::chrono::nanoseconds timeout) override;

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
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);

  std::string env_var;
  try {
    env_var = rcpputils::get_env_var("RMW_IMPLEMENTATION");
  } catch (const std::exception & e) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to fetch RMW_IMPLEMENTATION "
      "from environment due to %s", e.what());
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
      "Failed to compute library name for '%s' due to %s",
      env_var.c_str(), e.what());
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

  tracepoint(TRACEPOINT_PROVIDER, rmw_implementation, env_var.c_str());

  if (env_var == "rmw_fastrtps_cpp") {
    // SubListener::on_data_available(eprosima::fastdds::dds::DataReader*)
    FASTDDS::ON_DATA_AVAILABLE = lib->get_symbol(
      "_ZThn8_N11SubListener17on_data_availableEPN8eprosima7fastdds3dds10DataReaderE");

    // rmw_fastrtps_shared_cpp::TypeSupport::serialize(void*, eprosima::fastrtps::rtps::SerializedPayload_t*)  // NOLINT
    FASTDDS::SERIALIZE = lib->get_symbol(
      "_ZN23rmw_fastrtps_shared_cpp11TypeSupport9serializeEPvPN8eprosima8fastrtps4rtps19SerializedPayload_tE");  // NOLINT
  } else if (env_var == "rmw_cyclonedds_cpp") {
    CYCLONEDDS::DDS_WRITE_IMPL = lib->get_symbol("dds_write_impl");
  }
}


// for cyclonedds
// bind : &ros_message -> source_timestamp
int dds_write_impl(void * wr, void * data, long tstamp, int action)  // NOLINT
{
  using functionT = int (*)(void *, void *, long, int);   // NOLINT
  if (CYCLONEDDS::DDS_WRITE_IMPL == nullptr) {
    update_dds_function_addr();
  }
  int dds_return = ((functionT) CYCLONEDDS::DDS_WRITE_IMPL)(wr, data, tstamp, action);

  tracepoint(TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, data, tstamp);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_bind_addr_to_stamp," << data << "," << tstamp << std::endl;
#endif
  return dds_return;
}

// For cyclone_dds
// measure the time when the DDS communication is completed.
static void on_data_available(dds_entity_t reader, void * arg)
{
  (void) on_data_available;
  (void) arg;
  static uint64_t last_timestamp_ns;
  dds_sample_info_t si;
  void * buf_ptr[] = {&CYCLONEDDS::DUMMY_BUF};

  // cyclonedds does not have an API to get only message info.
  // Therefore, we use a dummy message buffer to get the message.
  // However, an error will occur when deserializing.
  // I also added a hook to avoid this error.
  dds_read(reader, reinterpret_cast<void **>(&buf_ptr), &si, 1, 1);
  uint64_t timestamp_ns = si.source_timestamp;

  // Omit the output of the trace points for the same message.
  // This is to reduce the output, so it does not need to be strict.
  if (timestamp_ns != last_timestamp_ns) {
    tracepoint(TRACEPOINT_PROVIDER, on_data_available, timestamp_ns);
#ifdef DEBUG_OUTPUT
    std::cerr << "on_data_available," << timestamp_ns << std::endl;
#endif
  }
  last_timestamp_ns = timestamp_ns;
}

// Configuration to run on_data_available
// By setting the listener to the parent, the child entities will inherit it.
dds_entity_t dds_create_subscriber(
  dds_entity_t participant,
  const dds_qos_t * qos,
  const dds_listener_t * listener
)
{
  using functionT = dds_entity_t (*)(
    const dds_domainid_t, const dds_qos_t *,
    const dds_listener_t *);

  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  if (listener) {
    RCLCPP_WARN(
      rclcpp::get_logger("caret"),
      "dds_create_participant passes non-null listener."
      "caret implementation assumes listener = nullptr.");
  }

  CYCLONEDDS::LISTENER = dds_create_listener(nullptr);

  // disable on_data_available hook
  // dds_lset_data_available(CYCLONEDDS::LISTENER, &on_data_available);

  return ((functionT) orig_func)(participant, qos, CYCLONEDDS::LISTENER);
}

// For CycloneDDS
// Configuration to run on_data_available.
dds_return_t dds_waitset_attach(
  dds_entity_t waitset,
  dds_entity_t entity,
  dds_attach_t x)
{
  using functionT = dds_return_t (*)(dds_entity_t, dds_entity_t, dds_attach_t);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  // disable on_data_available hook
  // dds_set_status_mask(entity, DDS_DATA_AVAILABLE_STATUS);

  return ((functionT) orig_func)(waitset, entity, x);
}

// Skip deserialize when a dummy buffer for getting message info is received.
bool ddsi_serdata_to_sample(
  const struct ddsi_serdata * d, void * sample, void ** bufptr,
  void * buflim)
{
  using functionT = bool (*)(const struct ddsi_serdata *, void *, void **, void *);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  if (sample == &CYCLONEDDS::DUMMY_BUF) {
    return true;
  }
  return ((functionT) orig_func)(d, sample, bufptr, buflim);
}

// for cyclonedds
// For measuring rcl layers.
dds_return_t dds_write(dds_entity_t writer, const void * data)
{
  using functionT = dds_return_t (*)(dds_entity_t, const void *);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  tracepoint(TRACEPOINT_PROVIDER, dds_write, data);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_write," << data << std::endl;
#endif
  return ((functionT) orig_func)(writer, data);
}

// for fstartps
// measure the time when the DDS communication is completed.
// SubListener::on_data_available(eprosima::fastdds::dds::DataReader*)
void _ZThn8_N11SubListener17on_data_availableEPN8eprosima7fastdds3dds10DataReaderE(
  void * obj,
  eprosima::fastdds::dds::DataReader * reader)
{
  static uint64_t last_timestamp_ns;
  using functionT = void (*)(void *, eprosima::fastdds::dds::DataReader *);

  eprosima::fastdds::dds::SampleInfo sinfo;
  ((functionT) FASTDDS::ON_DATA_AVAILABLE)(obj, reader);

  // fastrps has an API to get the info directly, so use it.
  if (reader->get_first_untaken_info(&sinfo) == ReturnCode_t::RETCODE_OK) {
    if (sinfo.valid_data) {
      uint64_t timestamp_ns = sinfo.source_timestamp.to_ns();
      // Omit the output of a tracepoint for the same message.
      // This is to reduce the output, so it does not need to be strict.
      if (timestamp_ns != last_timestamp_ns) {
        tracepoint(TRACEPOINT_PROVIDER, on_data_available, timestamp_ns);
#ifdef DEBUG_OUTPUT
        std::cerr << "on_data_available," << timestamp_ns << std::endl;
#endif
      }
      last_timestamp_ns = timestamp_ns;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("caret"), "failed to get message info");
    }
  }
}

// for fastrtps
// For measuring rcl layers.
bool _ZN8eprosima7fastdds3dds10DataWriter5writeEPv(void * obj, void * data)
{
  using functionT = bool (*)(void *, void *);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  auto ser_data = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);

  tracepoint(TRACEPOINT_PROVIDER, dds_write, ser_data->data);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_write," << ser_data->data << std::endl;
#endif

  return ((functionT) orig_func)(obj, data);
}

// for fastrtps
// For measuring rcl layers.
bool _ZN8eprosima7fastdds3dds10DataWriter5writeEPvRNS_8fastrtps4rtps11WriteParamsE(
  void * obj,
  void * data,
  eprosima::fastrtps::rtps::WriteParams & params)
{
  using functionT = bool (*)(void *, void *, eprosima::fastrtps::rtps::WriteParams &);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  auto ser_data = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);

  tracepoint(TRACEPOINT_PROVIDER, dds_write, ser_data->data);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_write," << ser_data->data << std::endl;
#endif
  return ((functionT) orig_func)(obj, data, params);
}


// In fastrtps, there is no place for ros_message and source_timestamp to be in the same function.
// In order to bind ros_message and source_timestamp, the address of payload is used.
// The bind from &payload to source_timestamp is done by unsent_change_added_to_history.
// bind: &ros_message -> &payload
// rmw_fastrtps_shared_cpp::TypeSupport::serialize(void*, eprosima::fastrtps::rtps::SerializedPayload_t*)   // NOLINT
bool SYMBOL_CONCAT_2(
  _ZN23rmw_fastrtps_shared_cpp11TypeSupport9serialize,
  EPvPN8eprosima8fastrtps4rtps19SerializedPayload_tE)(
  // NOLINT
  void * obj, void * data, eprosima::fastrtps::rtps::SerializedPayload_t * payload)
{
  using functionT = bool (*)(void *, void *, eprosima::fastrtps::rtps::SerializedPayload_t *);

  auto ser_data = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);
  auto payload_ptr = static_cast<void *>(payload->data);
  if (FASTDDS::SERIALIZE == nullptr) {
    update_dds_function_addr();
  }
  auto ret = ((functionT) FASTDDS::SERIALIZE)(obj, data, payload);

  tracepoint(TRACEPOINT_PROVIDER, dds_bind_addr_to_addr, ser_data->data, payload_ptr);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_bind_addr_to_addr," << ser_data->data << "," << payload_ptr << std::endl;
#endif

  return ret;
}

// for fastrtps
// bind: &payload -> source_timestamp
// unsent_change_added_to_history
void SYMBOL_CONCAT_3(
  _ZN8eprosima8fastrtps4rtps15StatelessWriter30unsent_change_added_to_history,
  EPNS1_13CacheChange_tERKNSt6chrono10time_point,
  INS5_3_V212steady_clockENS5_8durationIlSt5ratioILl1ELl1000000000EEEEEE)(
  // NOLINT
  void * obj,
  eprosima::fastrtps::rtps::CacheChange_t * change,
  const std::chrono::time_point<std::chrono::steady_clock>&max_blocking_time
  )
{
  using functionT = bool (*)(
    void *,
    eprosima::fastrtps::rtps::CacheChange_t *,
    const std::chrono::time_point<std::chrono::steady_clock> &);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  auto payload_data_ptr = static_cast<void *>(change->serializedPayload.data);
  auto source_timestamp = change->sourceTimestamp.to_ns();

  ((functionT) orig_func)(obj, change, max_blocking_time);

  tracepoint(TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, payload_data_ptr, source_timestamp);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_bind_addr_to_stamp," << payload_data_ptr << "," << source_timestamp <<
    std::endl;
#endif
}


// for fastrtps
// bind: &payload -> source_timestamp
// unsent_change_added_to_history
void SYMBOL_CONCAT_3(
  _ZN8eprosima8fastrtps4rtps14StatefulWriter30unsent_change_added_to_history,
  EPNS1_13CacheChange_tERKNSt6chrono10time_point,
  INS5_3_V212steady_clockENS5_8durationIlSt5ratioILl1ELl1000000000EEEEEE)(
  // NOLINT
  void * obj,
  eprosima::fastrtps::rtps::CacheChange_t * change,
  const std::chrono::time_point<std::chrono::steady_clock>&max_blocking_time
  )
{
  using functionT =
    bool (*)(
    void *, eprosima::fastrtps::rtps::CacheChange_t *,
    const std::chrono::time_point<std::chrono::steady_clock> &);
  static void * orig_func = dlsym(RTLD_NEXT, __func__);

  auto payload_data_ptr = static_cast<void *>(change->serializedPayload.data);
  auto source_timestamp = change->sourceTimestamp.to_ns();

  ((functionT) orig_func)(obj, change, max_blocking_time);

  tracepoint(TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, payload_data_ptr, source_timestamp);
#ifdef DEBUG_OUTPUT
  std::cerr << "dds_bind_addr_to_stamp," << payload_data_ptr << "," << source_timestamp <<
    std::endl;
#endif
}

// rclcpp::executors::SingleThreadedExecutor::SingleThreadedExecutor(rclcpp::ExecutorOptions const&)
void _ZN6rclcpp9executors22SingleThreadedExecutorC1ERKNS_15ExecutorOptionsE(
  void * obj,
  const void * option)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const void *);
  ((functionT) orig_func)(obj, option);
  const std::string executor_type_name = "single_threaded_executor";

  tracepoint(TRACEPOINT_PROVIDER, construct_executor, obj, executor_type_name.c_str());
#ifdef DEBUG_OUTPUT
  std::cerr << "construct_executor," <<
    executor_type_name << "," <<
    obj << std::endl;
#endif
}

// rclcpp::executors::MultiThreadedExecutor::MultiThreadedExecutor(
// rclcpp::ExecutorOptions const&, unsigned long, bool,
// std::chrono::duration<long, std::ratio<1l, 1000000000l> >)
void
SYMBOL_CONCAT_2(
  _ZN6rclcpp9executors21MultiThreadedExecutor,
  C1ERKNS_15ExecutorOptionsEmbNSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE)(
  void * obj,
  const void * option,
  size_t number_of_thread,
  bool yield_before_execute,
  const void * timeout)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const void *, size_t, bool, const void *);
  ((functionT) orig_func)(obj, option, number_of_thread, yield_before_execute, timeout);
  const std::string executor_type_name = "multi_threaded_executor";

  tracepoint(TRACEPOINT_PROVIDER, construct_executor, obj, executor_type_name.c_str());
#ifdef DEBUG_OUTPUT
  std::cerr << "construct_executor," <<
    executor_type_name << "," <<
    obj << std::endl;
#endif
}

// rclcpp::executors::StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(
// rclcpp::ExecutorOptions const&)
void _ZN6rclcpp9executors28StaticSingleThreadedExecutorC1ERKNS_15ExecutorOptionsE(
  void * obj,
  const void * option)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const void *);
  ((functionT) orig_func)(obj, option);

  using StaticSingleThreadedExecutorPublic = rclcpp::executors::StaticSingleThreadedExecutorPublic;
  auto exec_ptr = reinterpret_cast<StaticSingleThreadedExecutorPublic *>(obj);

  auto entities_collector_ptr = static_cast<const void *>(exec_ptr->entities_collector_.get());
  tracepoint(
    TRACEPOINT_PROVIDER,
    construct_static_executor,
    obj,
    entities_collector_ptr,
    "static_single_threaded_executor");
#ifdef DEBUG_OUTPUT
  std::cerr << "construct_static_executor," <<
    "static_single_threaded_executor" << "," <<
    obj << "," << entities_collector_ptr << std::endl;
#endif
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
  void * obj,
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  const void * node_ptr,
  const void * weak_groups_to_nodes,
  bool notify
  )
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(
    void *,
    rclcpp::CallbackGroup::SharedPtr,
    const void *,
    const void *,
    bool);
  auto group_addr = static_cast<const void *>(group_ptr.get());
  std::string group_type_name = "unknown";
  auto group_type = group_ptr->type();
  if (group_type == rclcpp::CallbackGroupType::MutuallyExclusive) {
    group_type_name = "mutually_exclusive";
  } else if (group_type == rclcpp::CallbackGroupType::Reentrant) {
    group_type_name = "reentrant";
  }

  ((functionT) orig_func)(obj, group_ptr, node_ptr, weak_groups_to_nodes, notify);

  tracepoint(TRACEPOINT_PROVIDER, add_callback_group, obj, group_addr, group_type_name.c_str());
#ifdef DEBUG_OUTPUT
  std::cerr << "add_callback_group," << obj << "," << group_addr << "," <<
    group_type_name << std::endl;
#endif
}

bool SYMBOL_CONCAT_3(
  _ZN6rclcpp9executors31StaticExecutorEntitiesCollector18add_callback_groupESt10shared_ptr,
  INS_13CallbackGroupEES2_INS_15node_interfaces17NodeBaseInterface,
  EERSt3mapISt8weak_ptrIS3_ES9_IS6_ESt10owner_lessISA_ESaISt4pairIKSA_SB_EEE) (
  void * obj,
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = bool (*)(
    void *,
    rclcpp::CallbackGroup::SharedPtr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &);

  auto group_addr = static_cast<const void *>(group_ptr.get());
  std::string group_type_name = "unknown";
  auto group_type = group_ptr->type();
  if (group_type == rclcpp::CallbackGroupType::MutuallyExclusive) {
    group_type_name = "mutually_exclusive";
  } else if (group_type == rclcpp::CallbackGroupType::Reentrant) {
    group_type_name = "reentrant";
  }

  auto ret = ((functionT) orig_func)(obj, group_ptr, node_ptr, weak_groups_to_nodes);

  tracepoint(
    TRACEPOINT_PROVIDER, add_callback_group_static_executor,
    obj, group_addr, group_type_name.c_str());
#ifdef DEBUG_OUTPUT
  std::cerr << "add_callback_group_static_executor," << obj << "," << group_addr << "," <<
    group_type_name << std::endl;
#endif

  return ret;
}

//  rclcpp::CallbackGroup::add_timer(std::shared_ptr<rclcpp::TimerBase>)
void _ZN6rclcpp13CallbackGroup9add_timerESt10shared_ptrINS_9TimerBaseEE(
  // ok
  void * obj,
  const rclcpp::TimerBase::SharedPtr timer_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::TimerBase::SharedPtr);

  auto timer_handle = static_cast<const void *>(timer_ptr->get_timer_handle().get());
  ((functionT) orig_func)(obj, timer_ptr);

  tracepoint(TRACEPOINT_PROVIDER, callback_group_add_timer, obj, timer_handle);

#ifdef DEBUG_OUTPUT
  std::cerr << "callback_group_add_timer," << obj << "," << timer_handle << std::endl;
#endif
}

// rclcpp::CallbackGroup::add_subscription(std::shared_ptr<rclcpp::SubscriptionBase>)
void _ZN6rclcpp13CallbackGroup16add_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
  void * obj,
  const rclcpp::SubscriptionBase::SharedPtr subscription_ptr
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::SubscriptionBase::SharedPtr);

  auto subscription_handle = static_cast<const void *>(
    subscription_ptr->get_subscription_handle().get());
  ((functionT) orig_func)(obj, subscription_ptr);

  tracepoint(TRACEPOINT_PROVIDER, callback_group_add_subscription, obj, subscription_handle);

#ifdef DEBUG_OUTPUT
  std::cerr << "callback_group_add_subscription," << obj << "," << subscription_handle << std::endl;
#endif
}

// rclcpp::CallbackGroup::add_service(std::shared_ptr<rclcpp::ServiceBase>)
void _ZN6rclcpp13CallbackGroup11add_serviceESt10shared_ptrINS_11ServiceBaseEE(
  void * obj,
  const rclcpp::ServiceBase::SharedPtr service_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::ServiceBase::SharedPtr);

  auto service_handle = static_cast<const void *>(service_ptr->get_service_handle().get());
  ((functionT) orig_func)(obj, service_ptr);

  tracepoint(TRACEPOINT_PROVIDER, callback_group_add_service, obj, service_handle);

#ifdef DEBUG_OUTPUT
  std::cerr << "callback_group_add_service," << obj << "," << service_handle << std::endl;
#endif
}

// rclcpp::CallbackGroup::add_client(std::shared_ptr<rclcpp::ClientBase>)
void _ZN6rclcpp13CallbackGroup10add_clientESt10shared_ptrINS_10ClientBaseEE(
  void * obj,
  const rclcpp::ClientBase::SharedPtr client_ptr)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const rclcpp::ClientBase::SharedPtr);

  auto client_handle = static_cast<const void *>(client_ptr->get_client_handle().get());
  ((functionT) orig_func)(obj, client_ptr);

  tracepoint(TRACEPOINT_PROVIDER, callback_group_add_client, obj, client_handle);

#ifdef DEBUG_OUTPUT
  std::cerr << "callback_group_add_client," << obj << "," << client_handle << std::endl;
#endif
}
}
