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
#include <iomanip>
#include <string>
#include <mutex>
#include <vector>
#include <tuple>
#include <utility>

#include "caret_trace/thread_local.hpp"
#include "caret_trace/keys_set.hpp"
#include "caret_trace/tracing_controller.hpp"
#include "caret_trace/singleton.hpp"
#include "caret_trace/virtual_member_variable.hpp"

#include "rcl/rcl.h"
#include "rmw/rmw.h"
#include "rmw/event.h"
#include "rcutils/shared_library.h"

#define TRACEPOINT_DEFINE
#include "caret_trace/tp.h"
#include "caret_trace/debug.hpp"

#include "rcpputils/shared_library.hpp"
#include "rcpputils/get_env.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"


#define DEBUG_OUTPUT

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

// for fastrtps
#include "fastdds/rtps/common/WriteParams.h"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"

// for cyclonedds
#include "dds/dds.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define SYMBOL_CONCAT_2(x, y)  x ## y
#define SYMBOL_CONCAT_3(x, y, z)  x ## y ## z

#define FRAME_ID_COMPACT_SIZE 256

static VirtualMemberVariables<std::string, uint32_t> tf_buffer_frame_id_compact_map;
static std::unordered_map<void *, void *> tf_cache_to_buffer_core_map;

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

  StaticExecutorEntitiesCollector::SharedPtr entities_collector_;
};

}  // namespace executors
}  // namespace rclcpp


namespace tf2
{
class BufferCorePublic : public tf2::BufferCoreInterface
{
public:
  /************* Constants ***********************/
  //!< Maximum graph search depth (deeper graphs will be assumed to have loops)
  TF2_PUBLIC
  static const uint32_t MAX_GRAPH_DEPTH = 1000UL;

  /** Constructor
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   *
   */
  TF2_PUBLIC
  explicit BufferCorePublic(tf2::Duration cache_time_ = tf2::BUFFER_CORE_DEFAULT_CACHE_TIME);

  TF2_PUBLIC
  virtual ~BufferCorePublic(void);

  /** \brief Clear all data */
  TF2_PUBLIC
  void clear() override;

  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
   * \return True unless an error occured
   */
  TF2_PUBLIC
  bool setTransform(
    const geometry_msgs::msg::TransformStamped & transform,
    const std::string & authority, bool is_static = false);

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  TF2_PUBLIC
  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const TimePoint & time) const override;

  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time.
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */

  TF2_PUBLIC
  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame, const TimePoint & target_time,
    const std::string & source_frame, const TimePoint & source_time,
    const std::string & fixed_frame) const override;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise
   */
  TF2_PUBLIC
  bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const TimePoint & time, std::string * error_msg = NULL) const override;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise
   */
  TF2_PUBLIC
  bool canTransform(
    const std::string & target_frame, const TimePoint & target_time,
    const std::string & source_frame, const TimePoint & source_time,
    const std::string & fixed_frame, std::string * error_msg = NULL) const override;

  /** \brief Get all frames that exist in the system.
   */
  TF2_PUBLIC
  std::vector<std::string> getAllFrameNames() const override;

  /** \brief A way to see what frames have been cached in yaml format
   * Useful for debugging tools
   */
  TF2_PUBLIC
  std::string allFramesAsYAML(TimePoint current_time) const;

  /** Backwards compatibility for #84
  */
  TF2_PUBLIC
  std::string allFramesAsYAML() const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  TF2_PUBLIC
  std::string allFramesAsString() const;

  using TransformableCallback = std::function<
    void (TransformableRequestHandle request_handle, const std::string & target_frame,
    const std::string & source_frame,
    TimePoint time, TransformableResult result)>;

  /// \brief Internal use only
  TF2_PUBLIC
  TransformableRequestHandle addTransformableRequest(
    const TransformableCallback & cb,
    const std::string & target_frame,
    const std::string & source_frame,
    TimePoint time);
  /// \brief Internal use only
  TF2_PUBLIC
  void cancelTransformableRequest(TransformableRequestHandle handle);


  // Tell the buffer that there are multiple threads serviciing it.
  // This is useful for derived classes to know if they can block or not.
  TF2_PUBLIC
  void setUsingDedicatedThread(bool value) {using_dedicated_thread_ = value;}
  // Get the state of using_dedicated_thread_
  TF2_PUBLIC
  bool isUsingDedicatedThread() const {return using_dedicated_thread_;}


  /* Backwards compatability section for tf::Transformer you should not use these
   */

  /**@brief Check if a frame exists in the tree
   * @param frame_id_str The frame id in question  */
  TF2_PUBLIC
  bool _frameExists(const std::string & frame_id_str) const;

  /**@brief Fill the parent of a frame.
   * @param frame_id The frame id of the frame in question
   * @param parent The reference to the string to fill the parent
   * Returns true unless "NO_PARENT" */
  TF2_PUBLIC
  bool _getParent(const std::string & frame_id, TimePoint time, std::string & parent) const;

  /** \brief A way to get a std::vector of available frame ids */
  TF2_PUBLIC
  void _getFrameStrings(std::vector<std::string> & ids) const;


  TF2_PUBLIC
  CompactFrameID _lookupFrameNumber(const std::string & frameid_str) const
  {
    return lookupFrameNumber(frameid_str);
  }
  TF2_PUBLIC
  CompactFrameID _lookupOrInsertFrameNumber(const std::string & frameid_str)
  {
    return lookupOrInsertFrameNumber(frameid_str);
  }

  TF2_PUBLIC
  tf2::TF2Error _getLatestCommonTime(
    CompactFrameID target_frame, CompactFrameID source_frame,
    TimePoint & time, std::string * error_string) const
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    return getLatestCommonTime(target_frame, source_frame, time, error_string);
  }

  TF2_PUBLIC
  CompactFrameID _validateFrameId(
    const char * function_name_arg,
    const std::string & frame_id) const
  {
    return validateFrameId(function_name_arg, frame_id);
  }

  /**@brief Get the duration over which this transformer will cache */
  TF2_PUBLIC
  tf2::Duration getCacheLength() {return cache_time_;}

  /** \brief Backwards compatabilityA way to see what frames have been cached
   * Useful for debugging
   */
  TF2_PUBLIC
  std::string _allFramesAsDot(TimePoint current_time) const;
  TF2_PUBLIC
  std::string _allFramesAsDot() const;

  /** \brief Backwards compatabilityA way to see what frames are in a chain
   * Useful for debugging
   */
  TF2_PUBLIC
  void _chainAsVector(
    const std::string & target_frame, TimePoint target_time,
    const std::string & source_frame, TimePoint source_time,
    const std::string & fixed_frame,
    std::vector<std::string> & output) const;

// private:
  /** \brief A way to see what frames have been cached
   * Useful for debugging. Use this call internally.
   */
  std::string allFramesAsStringNoLock() const;


  /******************** Internal Storage ****************/

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  typedef std::vector<TimeCacheInterfacePtr> V_TimeCacheInterface;
  V_TimeCacheInterface frames_;

  /** \brief A mutex to protect testing and allocating new frames on the above vector. */
  mutable std::mutex frame_mutex_;

  /** \brief A map from string frame ids to CompactFrameID */
  typedef std::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
  M_StringToCompactFrameID frameIDs_;
  /** \brief A map from CompactFrameID frame_id_numbers to string for debugging and output */
  std::vector<std::string> frameIDs_reverse_;
  /** \brief A map to lookup the most recent authority for a given frame */
  std::map<CompactFrameID, std::string> frame_authority_;


  /// How long to cache transform history
  tf2::Duration cache_time_;

  typedef uint32_t TransformableCallbackHandle;

  typedef std::unordered_map<TransformableCallbackHandle,
      TransformableCallback> M_TransformableCallback;
  M_TransformableCallback transformable_callbacks_;
  uint32_t transformable_callbacks_counter_;
  std::mutex transformable_callbacks_mutex_;

  struct TransformableRequest
  {
    TimePoint time;
    TransformableRequestHandle request_handle;
    TransformableCallbackHandle cb_handle;
    CompactFrameID target_id;
    CompactFrameID source_id;
    std::string target_string;
    std::string source_string;
  };
  typedef std::vector<TransformableRequest> V_TransformableRequest;
  V_TransformableRequest transformable_requests_;
  std::mutex transformable_requests_mutex_;
  uint64_t transformable_requests_counter_;


  /************************* Internal Functions ****************************/

  bool setTransformImpl(
    const tf2::Transform & transform_in, const std::string frame_id,
    const std::string child_frame_id, const TimePoint stamp,
    const std::string & authority, bool is_static);
  void lookupTransformImpl(
    const std::string & target_frame, const std::string & source_frame,
    const TimePoint & time_in, tf2::Transform & transform, TimePoint & time_out) const;

  void lookupTransformImpl(
    const std::string & target_frame, const TimePoint & target_time,
    const std::string & source_frame, const TimePoint & source_time,
    const std::string & fixed_frame, tf2::Transform & transform, TimePoint & time_out) const;

  /** \brief An accessor to get a frame, which will throw an exception if the frame is no there.
   * \param frame_number The frameID of the desired Reference Frame
   *
   * This is an internal function which will get the pointer to the frame associated with the frame id
   * Possible Exception: tf::LookupException
   */
  TimeCacheInterfacePtr getFrame(CompactFrameID c_frame_id) const;

  TimeCacheInterfacePtr allocateFrame(CompactFrameID cfid, bool is_static);

  /** \brief Validate a frame ID format and look up its CompactFrameID.
    *   For invalid cases, produce an message.
    * \param function_name_arg string to print out in the message,
    *   the current function and argument name being validated
    * \param frame_id name of the tf frame to validate
    * \param[out] error_msg if non-NULL, fill with produced error messaging.
    *   Otherwise messages are logged as warning.
    * \return The CompactFrameID of the frame or 0 if not found.
    */
  CompactFrameID validateFrameId(
    const char * function_name_arg,
    const std::string & frame_id,
    std::string * error_msg) const;

  /** \brief Validate a frame ID format and look it up its compact ID.
    *   Raise an exception for invalid cases.
    * \param function_name_arg string to print out in the exception,
    *   the current function and argument name being validated
    * \param frame_id name of the tf frame to validate
    * \return The CompactFrameID of the existing frame.
    * \raises InvalidArgumentException if the frame_id string has an invalid format
    * \raises LookupException if frame_id did not exist
    */
  CompactFrameID validateFrameId(
    const char * function_name_arg,
    const std::string & frame_id) const;

  /// String to number for frame lookup. Returns 0 if the frame was not found.
  CompactFrameID lookupFrameNumber(const std::string & frameid_str) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookupOrInsertFrameNumber(const std::string & frameid_str);

  /// Number to string frame lookup may throw LookupException if number invalid
  const std::string & lookupFrameString(CompactFrameID frame_id_num) const;

  void createConnectivityErrorString(
    CompactFrameID source_frame, CompactFrameID target_frame,
    std::string * out) const;

  /**@brief Return the latest rostime which is common across the spanning set
   * zero if fails to cross */
  tf2::TF2Error getLatestCommonTime(
    CompactFrameID target_frame, CompactFrameID source_frame,
    TimePoint & time, std::string * error_string) const;

  template<typename F>
  tf2::TF2Error walkToTopParent(
    F & f, TimePoint time, CompactFrameID target_id,
    CompactFrameID source_id, std::string * error_string) const;

  /**@brief Traverse the transform tree. If frame_chain is not NULL, store the traversed frame tree in vector frame_chain.
   * */
  template<typename F>
  tf2::TF2Error walkToTopParent(
    F & f, TimePoint time, CompactFrameID target_id,
    CompactFrameID source_id, std::string * error_string,
    std::vector<CompactFrameID> * frame_chain) const;

  void testTransformableRequests();
  // Thread safe transform check, acquire lock and call canTransformNoLock.
  bool canTransformInternal(
    CompactFrameID target_id, CompactFrameID source_id,
    const TimePoint & time, std::string * error_msg) const;
  // Actual implementation to walk the transform tree and find out if a transform exists.
  bool canTransformNoLock(
    CompactFrameID target_id, CompactFrameID source_id,
    const TimePoint & time, std::string * error_msg) const;

  // Whether it is safe to use canTransform with a timeout.
  // (If another thread is not provided it will always timeout.)
  bool using_dedicated_thread_;
};

}
namespace tf2_ros
{
class TransformBroadcasterPublic
{
public:
  template<class NodeT, class AllocatorT = std::allocator<void>>
  TransformBroadcasterPublic(
    NodeT && node,
    const rclcpp::QoS & qos = DynamicBroadcasterQoS(),
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = []() {
      rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
      options.qos_overriding_options = rclcpp::QosOverridingOptions{
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability};
      return options;
    } ())
  {
    publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
      node, "/tf", qos, options);
  }

  /** \brief Send a TransformStamped message
   *
   * The transform ʰTₐ added is from `child_frame_id`, `a` to `header.frame_id`,
   * `h`. That is, position in `child_frame_id` ᵃp can be transformed to
   * position in `header.frame_id` ʰp such that ʰp = ʰTₐ ᵃp .
   *
   */
  TF2_ROS_PUBLIC
  void sendTransform(const geometry_msgs::msg::TransformStamped & transform);

  /** \brief Send a vector of TransformStamped messages
   *
   * The transforms ʰTₐ added are from `child_frame_id`, `a` to `header.frame_id`,
   * `h`. That is, position in `child_frame_id` ᵃp can be transformed to
   * position in `header.frame_id` ʰp such that ʰp = ʰTₐ ᵃp .
   */
  TF2_ROS_PUBLIC
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms);

public:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

}  // namespace tf2_ros

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

static void * lib_tf2_handler;
__attribute__((constructor))
static void before_main(void)
{
  lib_tf2_handler = dlopen("libtf2_ros.so", RTLD_LAZY);

  if (!lib_tf2_handler) {
      std::cerr << "Failed to load libtf2_ros.so. " << dlerror() << std::endl;
      exit(EXIT_FAILURE);
  }
}

__attribute__((destructor))
static void after_main(void)
{
  dlclose(lib_tf2_handler);
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
  debug.print(
    "dds_bind_addr_to_stamp",
    data,
    std::to_string(tstamp)
  );
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
    debug.print(
      "on_data_available",
      timestamp_ns
    );
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
  debug.print("dds_write", data);
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
        debug.print("on_data_available", timestamp_ns);
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
  debug.print("dds_write", ser_data->data);
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
  debug.print("dds_write", ser_data->data);
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
  debug.print(
    "dds_bind_addr_to_addr",
    ser_data->data,
    payload_ptr
  );
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
  debug.print(
    "dds_bind_addr_to_stamp",
    payload_data_ptr,
    source_timestamp
  );
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
  debug.print("dds_bind_addr_to_stamp", payload_data_ptr, source_timestamp);
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
  debug.print("construct_executor", executor_type_name, obj);
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
  debug.print("construct_executor", executor_type_name, obj);
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
  debug.print(
    "construct_static_executor",
    "static_single_threaded_executor",
    obj,
    entities_collector_ptr
  );
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
  debug.print(
    "add_callback_group",
    obj,
    group_addr,
    group_type_name
  );
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
  debug.print(
    "add_callback_group_static_executor",
    obj,
    group_addr,
    group_type_name
  );
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
  debug.print(
    "callback_group_add_timer",
    obj,
    timer_handle
  );
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
  debug.print(
    "callback_group_add_subscription",
    obj,
    subscription_handle
  );
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
  debug.print(
    "callback_group_add_service",
    obj,
    service_handle
  );
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
  debug.print(
    "callback_group_add_client",
    obj,
    client_handle
  );
#endif
}

//  tf2_ros::TransformBroadcaster::sendTransform(std::vector<geometry_msgs::msg::TransformStamped_<std::allocator<void> >, std::allocator<geometry_msgs::msg::TransformStamped_<std::allocator<void> > > > const&)
void
_ZN7tf2_ros20TransformBroadcaster13sendTransformERKSt6vectorIN13geometry_msgs3msg17TransformStamped_ISaIvEEESaIS6_EE(
  void * obj,
  const std::vector<geometry_msgs::msg::TransformStamped> &msgtf
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, const std::vector<geometry_msgs::msg::TransformStamped> &);

  static auto & controller = Singleton<TracingController>::get_instance();

  static KeysSet<void *, void *> transform_init_recorded;
  auto obj_public = reinterpret_cast<tf2_ros::TransformBroadcasterPublic *>(obj);
  auto pub_handle = obj_public->publisher_->get_publisher_handle().get();

  if (controller.is_tf_allowed()) {
    if (!transform_init_recorded.has(obj, pub_handle)) {
      transform_init_recorded.insert(obj, pub_handle);
#ifdef DEBUG_OUTPUT
      debug.print(
        "init_bind_transform_broadcaster",
        obj,
        pub_handle
      );
#endif
      tracepoint(
        TRACEPOINT_PROVIDER,
        init_bind_transform_broadcaster,
        obj,
        pub_handle
      );
    }

    static KeysSet<void *, std::string, std::string> sendtransform_init_recorded;

    static VirtualMemberVariables<std::string, uint32_t> membar_vars;
    auto &vars = membar_vars.get_vars(obj);

    auto export_frame_id_compact = [&obj, &vars](const std::string &frame_id){
        tracepoint(
          TRACEPOINT_PROVIDER,
          init_tf_broadcaster_frame_id_compact,
          obj,
          frame_id.c_str(),
          vars.get(frame_id)
        );
#ifdef DEBUG_OUTPUT
        debug.print(
          "init_tf_broadcaster_frame_id",
          obj,
          frame_id.c_str(),
          vars.get(frame_id)
        );
#endif
    };

    uint32_t frame_ids[FRAME_ID_COMPACT_SIZE] = {0};
    uint32_t child_frame_ids[FRAME_ID_COMPACT_SIZE] = {0};
    uint64_t stamps[FRAME_ID_COMPACT_SIZE] = {0};

    for (size_t i=0 ; i<msgtf.size() ; i++) {
      const auto & t = msgtf[i];

      stamps[i] =  t.header.stamp.nanosec + t.header.stamp.sec * 1000000000L;
      std::string frame_ids_[] = {t.header.frame_id, t.child_frame_id};
      for (auto & frame_id_ : frame_ids_) {
        if (!vars.has(frame_id_)) {
          vars.set(frame_id_, vars.size());
          export_frame_id_compact(frame_id_);
        }
      }
      frame_ids[i] = vars.get(t.header.frame_id);
      child_frame_ids[i] = vars.get(t.child_frame_id);

      if (!sendtransform_init_recorded.has(obj, t.header.frame_id, t.child_frame_id)) {
        sendtransform_init_recorded.insert(obj, t.header.frame_id, t.child_frame_id);
        tracepoint(
          TRACEPOINT_PROVIDER,
          init_bind_transform_broadcaster_frames,
          obj,
          t.header.frame_id.c_str(),
          t.child_frame_id.c_str()
        );
#ifdef DEBUG_OUTPUT
        debug.print(
          "init_bind_transform_broadcaster_frames",
          obj,
          t.header.frame_id,
          t.child_frame_id
        );
#endif
      }
#ifdef DEBUG_OUTPUT
      debug.print(
        "send_transform",
        obj,
        std::to_string(stamps[i]),
        std::to_string(frame_ids[i]),
        std::to_string(child_frame_ids[i])
      );
#endif
    }

    tracepoint(
      TRACEPOINT_PROVIDER,
      send_transform,
      obj,
      stamps,
      frame_ids,
      child_frame_ids,
      msgtf.size()
    );
  }

  ((functionT) orig_func)(obj, msgtf);
}

// findClosest
uint8_t
_ZN3tf29TimeCache11findClosestERPNS_16TransformStorageES3_NSt6chrono10time_pointINS4_3_V212system_clockENS4_8durationIlSt5ratioILl1ELl1000000000EEEEEEPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE(
  void * obj,
  tf2::TransformStorage * & one,
  tf2::TransformStorage * & two,
  tf2::TimePoint target_time,
  std::string * error_str
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = uint8_t (*)(
    void *,
    tf2::TransformStorage * &,
    tf2::TransformStorage * &,
    tf2::TimePoint,
    std::string *
  );

  static auto & controller = Singleton<TracingController>::get_instance();

  uint8_t ret = ((functionT) orig_func)(obj, one, two, target_time, error_str);

  if (controller.is_tf_allowed()) {
    if (tf_cache_to_buffer_core_map.count(obj) > 0) {
      auto buffer_core = tf_cache_to_buffer_core_map[obj];
      if (ret == 1) {
        tracepoint(
          TRACEPOINT_PROVIDER,
          tf_buffer_find_closest,
          buffer_core,
          one->frame_id_,
          one->child_frame_id_,
          one->stamp_.time_since_epoch().count(),
          0,
          0,
          0);
#ifdef DEBUG_OUTPUT
        debug.print(
          "tf_buffer_find_closest",
          buffer_core,
          one->frame_id_,
          one->child_frame_id_,
          std::to_string(one->stamp_.time_since_epoch().count()),
          0,
          0,
          0);
#endif
      } else if (ret == 2) {
        tracepoint(
          TRACEPOINT_PROVIDER,
          tf_buffer_find_closest,
          buffer_core,
          one->frame_id_,
          one->child_frame_id_,
          one->stamp_.time_since_epoch().count(),
          two->frame_id_,
          two->child_frame_id_,
          two->stamp_.time_since_epoch().count()
        );
#ifdef DEBUG_OUTPUT
        debug.print(
          "tf_buffer_find_closest",
          buffer_core,
          one->frame_id_,
          one->child_frame_id_,
          std::to_string(one->stamp_.time_since_epoch().count()),
          two->frame_id_,
          two->child_frame_id_,
          std::to_string(two->stamp_.time_since_epoch().count())
        );
#endif
      }
    }
  }

  return ret;
}



// lookupOrInsertFrameNumber
tf2::CompactFrameID
_ZN3tf210BufferCore25lookupOrInsertFrameNumberERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE(
  void * obj,
  const std::string frameid_str
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = tf2::CompactFrameID (*)(
    void *,
    const std::string
  );

  static auto & controller = Singleton<TracingController>::get_instance();

  tf2::CompactFrameID ret = ((functionT) orig_func)(obj, frameid_str);

  if (controller.is_tf_allowed()) {
    auto &map = tf_buffer_frame_id_compact_map.get_vars(obj);

    if (map.has(frameid_str) == 0) {
      map.set(frameid_str, ret);
      tracepoint(
        TRACEPOINT_PROVIDER,
        init_tf_buffer_frame_id_compact,
        obj,
        frameid_str.c_str(),
        ret
      );
#ifdef DEBUG_OUTPUT
      debug.print(
        "init_tf_buffer_frame_id_compact",
        obj,
        frameid_str,
        ret
      );
#endif
    }
  }

  return ret;
}


// setTransform
bool
_ZN3tf210BufferCore12setTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEb(
  void * obj,
  const geometry_msgs::msg::TransformStamped & transform,
  const std::string & authority,
  bool is_static
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = uint8_t (*)(
    void *,
    const geometry_msgs::msg::TransformStamped &,
    const std::string &,
    bool
  );

  static KeysSet<void *> init_recorded_keys;

  static auto & controller = Singleton<TracingController>::get_instance();

  if (controller.is_tf_allowed()) {
    // transform.header.frame_id;
    // transform.child_frame_id;
    if (!init_recorded_keys.has(obj)) {
      init_recorded_keys.insert(obj);
      tracepoint(
        TRACEPOINT_PROVIDER,
        init_bind_tf_buffer_core,
        obj,
        get_callback()
      );
  #ifdef DEBUG_OUTPUT
      debug.print(
        "init_bind_tf_buffer_core",
        obj,
        get_callback()
      );
  #endif
    }
  }

  bool ret = ((functionT) orig_func)(obj, transform, authority, is_static);

  if (controller.is_tf_allowed()) {
    uint64_t stamp = transform.header.stamp.sec * 1000000000L   + transform.header.stamp.nanosec;
    auto &map = tf_buffer_frame_id_compact_map.get_vars(obj);

    if (map.has(transform.header.frame_id) && map.has(transform.child_frame_id)) {
#ifdef DEBUG_OUTPUT
      debug.print(
        "tf_set_transform",
        obj,
        stamp,
        transform.header.frame_id,
        transform.child_frame_id
      );
#endif
      tracepoint(
        TRACEPOINT_PROVIDER,
        tf_set_transform,
        obj,
        stamp,
        map.get(transform.header.frame_id),
        map.get(transform.child_frame_id)
      );
    }
  }

  return ret;
}

geometry_msgs::msg::TransformStamped
_ZNK3tf210BufferCore15lookupTransformERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNSt6chrono10time_pointINS9_3_V212system_clockENS9_8durationIlSt5ratioILl1ELl1000000000EEEEEES8_SJ_S8_(
  void * obj,
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = geometry_msgs::msg::TransformStamped (*)(
    void *,
    const std::string &,
    const tf2::TimePoint &,
    const std::string &,
    const tf2::TimePoint &,
    const std::string &
  );

  static auto & controller = Singleton<TracingController>::get_instance();
  if (controller.is_tf_allowed()) {
#ifdef DEBUG_OUTPUT
    debug.print(
      "lookupTransform",
      obj,
      target_frame,
      source_frame,
      target_time.time_since_epoch().count()
    );
#endif
  }

  geometry_msgs::msg::TransformStamped ret = ((functionT) orig_func)(
    obj,
    target_frame,
    target_time,
    source_frame,
    source_time,
    fixed_frame
  );
  return ret;
}


geometry_msgs::msg::TransformStamped
_ZNK3tf210BufferCore15lookupTransformERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES8_RKNSt6chrono10time_pointINS9_3_V212system_clockENS9_8durationIlSt5ratioILl1ELl1000000000EEEEEE(
  void * obj,
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = geometry_msgs::msg::TransformStamped (*)(
    void *,
    const std::string &,
    const std::string &,
    const tf2::TimePoint &
  );

  static auto & controller = Singleton<TracingController>::get_instance();

  auto &map = tf_buffer_frame_id_compact_map.get_vars(obj);
  auto target_time = std::chrono::time_point_cast<std::chrono::nanoseconds>(time).time_since_epoch().count();

  auto is_frame_id_compact_initialized =  map.has(target_frame) && map.has(source_frame);
  if (controller.is_tf_allowed()) {
    static KeysSet<void*, uint32_t, uint32_t> tf_lookup_transform;
    if (is_frame_id_compact_initialized) {
      auto target_frame_id_compact = map.get(target_frame);
      auto source_frame_id_compact = map.get(source_frame);

      if (!tf_lookup_transform.has(obj, target_frame_id_compact, source_frame_id_compact)) {
        tf_lookup_transform.insert(obj, target_frame_id_compact, source_frame_id_compact);
        tracepoint(
          TRACEPOINT_PROVIDER,
          init_tf_buffer_lookup_transform,
          obj,
          target_frame_id_compact,
          source_frame_id_compact
        );
      }
      tracepoint(
        TRACEPOINT_PROVIDER,
        tf_lookup_transform_start,
        obj,
        target_time,
        target_frame_id_compact,
        source_frame_id_compact
      );
#ifdef DEBUG_OUTPUT
      debug.print(
        "tf_lookup_transform_start",
        obj,
        target_time,
        target_frame,
        source_frame
      );
#endif
    }
  }

  geometry_msgs::msg::TransformStamped ret = ((functionT) orig_func)(
    obj,
    target_frame,
    source_frame,
    time
  );

  if (controller.is_tf_allowed()) {
    if (is_frame_id_compact_initialized) {
      tracepoint(
        TRACEPOINT_PROVIDER,
        tf_lookup_transform_end,
        obj
      );
#ifdef DEBUG_OUTPUT
      debug.print(
        "tf_lookup_transform_end",
        obj
      );
#endif
    }
  }

  return ret;
}

// getFrame
tf2::TimeCacheInterfacePtr _ZNK3tf210BufferCore8getFrameEj(
  void * obj,
  tf2::CompactFrameID frame_id
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = tf2::TimeCacheInterfacePtr (*)(
    void *,
    tf2::CompactFrameID
  );

  auto ret = ((functionT) orig_func)(
    obj,
    frame_id
  );

  auto time_cache =  ret.get();
  if(time_cache != nullptr && tf_cache_to_buffer_core_map.count(time_cache) == 0) {
    tf_cache_to_buffer_core_map[time_cache]= obj;
  }

  return ret;
}



inline void tf2_ros_const_buffer(void * obj, rclcpp::Clock::SharedPtr & clock){
  static auto & controller = Singleton<TracingController>::get_instance();

  if (controller.is_tf_allowed()) {
    auto tf_buffer_core = get_tf_buffer_core();
    tracepoint(
      TRACEPOINT_PROVIDER,
      construct_tf_buffer,
      obj,
      tf_buffer_core,
      clock.get()
    );
#ifdef DEBUG_OUTPUT
    debug.print(
      "construct_tf_buffer",
      obj,
      tf_buffer_core,
      clock.get()
    );
#endif
  }
}

tf2_ros::Buffer *
_ZN7tf2_ros6BufferC1ESt10shared_ptrIN6rclcpp5ClockEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEES1_INS2_4NodeEE(
  void * obj,
  rclcpp::Clock::SharedPtr clock,
  tf2::Duration cache_time,
  rclcpp::Node::SharedPtr node
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = tf2_ros::Buffer * (*)(
    void *,
    rclcpp::Clock::SharedPtr,
    tf2::Duration,
    rclcpp::Node::SharedPtr
  );

  auto ret = ((functionT) orig_func)(
    obj,
    clock,
    cache_time,
    node
  );

  tf2_ros_const_buffer(obj, clock);

  return ret;
}
// tf2_ros::Buffer constructor
tf2_ros::Buffer *
_ZN7tf2_ros6BufferC2ESt10shared_ptrIN6rclcpp5ClockEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEES1_INS2_4NodeEE(
  void * obj,
  rclcpp::Clock::SharedPtr clock,
  tf2::Duration cache_time,
  rclcpp::Node::SharedPtr node
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = tf2_ros::Buffer * (*)(
    void *,
    rclcpp::Clock::SharedPtr,
    tf2::Duration,
    rclcpp::Node::SharedPtr
  );

  auto ret = ((functionT) orig_func)(
    obj,
    clock,
    cache_time,
    node
  );

  tf2_ros_const_buffer(obj, clock);

  return ret;
}

// tf2::BufferCore constructor
tf2::BufferCore *
_ZN3tf210BufferCoreC2ENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE(
  void * obj,
  tf2::Duration cache_time
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = tf2_ros::Buffer * (*)(
    void *,
    tf2::Duration
  );

  auto ret = ((functionT) orig_func)(
    obj,
    cache_time
  );

  // tracepoint(
  //   TRACEPOINT_PROVIDER,
  //   construct_tf_buffer_core,
  //   obj
  // );
  // debug.print(
  //   "construct_tf_buffer_core",
  //   obj
  // );
  set_tf_buffer_core(obj);

  return ret;
}
// _ZN7tf2_ros6BufferC2ESt10shared_ptrIN6rclcpp5ClockEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEES1_INS2_4NodeEE

rclcpp::Node *
_ZN6rclcpp4NodeC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES8_RKNS_11NodeOptionsE(
  rclcpp::Node * obj,
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options
)
{
  static void * orig_func = dlsym(lib_tf2_handler, __func__);
  using functionT = rclcpp::Node * (*)(
    void *,
    const std::string &,
    const std::string &,
    const rclcpp::NodeOptions &
  );

  auto ret = ((functionT) orig_func)(
    obj,
    node_name,
    namespace_,
    options
  );

  auto node_handle = obj->get_node_base_interface()->get_rcl_node_handle();
  tracepoint(
    TRACEPOINT_PROVIDER,
    construct_node_hook,
    node_handle,
    obj->get_clock().get()
  );
#ifdef DEBUG_OUTPUT
  debug.print(
    "construct_node_hook_impl",
    node_handle,
    obj->get_clock().get()
  );
#endif

  return ret;
}


// construct IntraProcessManager
void _ZN6rclcpp12experimental19IntraProcessManagerC1Ev(
  void * obj
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *);
  ((functionT) orig_func)(obj);

  tracepoint(TRACEPOINT_PROVIDER, construct_ipm, obj);
#ifdef DEBUG_OUTPUT
  debug.print("construct_ipm", obj);
#endif
}

// ipm add_publisher
uint64_t _ZN6rclcpp12experimental19IntraProcessManager13add_publisherESt10shared_ptrINS_13PublisherBaseEE
(
  void * obj,
  rclcpp::PublisherBase::SharedPtr publisher
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = uint64_t (*)(void *, rclcpp::PublisherBase::SharedPtr);
  uint64_t ret =  ((functionT) orig_func)(obj, publisher);

  auto publisher_handle = publisher->get_publisher_handle().get();
  tracepoint(TRACEPOINT_PROVIDER, ipm_add_publisher, obj, publisher_handle, ret);

#ifdef DEBUG_OUTPUT
  debug.print("ipm_add_publisher", obj, publisher_handle, ret);
#endif
  return ret;
}

// ipm add_subscription
uint64_t _ZN6rclcpp12experimental19IntraProcessManager16add_subscriptionESt10shared_ptrINS0_28SubscriptionIntraProcessBaseEE
(
  void * obj,
  rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr subscription
)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = uint64_t (*)(void *, rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr);
  uint64_t ret =  ((functionT) orig_func)(obj, subscription);

  auto subscription_handle = get_subscription_handle();

  tracepoint(TRACEPOINT_PROVIDER, ipm_add_subscription, obj, subscription_handle, ret);

#ifdef DEBUG_OUTPUT
  debug.print("ipm_add_subscription", obj, subscription_handle, ret);
#endif
  return ret;
}

// ipm insert_sub_id_for_pub
void _ZN6rclcpp12experimental19IntraProcessManager21insert_sub_id_for_pubEmmb
(
  void * obj,
  uint64_t sub_id,
  uint64_t pub_id,
  bool use_take_shared_method
){
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = void (*)(void *, uint64_t, uint64_t, bool);
  ((functionT) orig_func)(obj, sub_id, pub_id, use_take_shared_method);

  tracepoint(
    TRACEPOINT_PROVIDER, ipm_insert_sub_id_for_pub, obj, sub_id, pub_id, use_take_shared_method);

#ifdef DEBUG_OUTPUT
  debug.print("ipm_insert_sub_id_for_pub", obj, sub_id, pub_id, use_take_shared_method);
#endif
}

rcl_ret_t rcl_timer_cancel(rcl_timer_t * timer)
{
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = rcl_ret_t (*)(rcl_timer_t *);
  rcl_ret_t ret = ((functionT) orig_func)(timer);

  tracepoint(TRACEPOINT_PROVIDER, rcl_timer_cancel, timer);

#ifdef DEBUG_OUTPUT
  debug.print("rcl_timer_cancel", timer);
#endif

  return ret;
}

rcl_ret_t rcl_timer_reset(rcl_timer_t * timer) {
  static void * orig_func = dlsym(RTLD_NEXT, __func__);
  using functionT = rcl_ret_t (*)(rcl_timer_t *);
  rcl_ret_t ret = ((functionT) orig_func)(timer);

  tracepoint(TRACEPOINT_PROVIDER, rcl_timer_reset, timer);

#ifdef DEBUG_OUTPUT
  debug.print("rcl_timer_reset", timer);
#endif
  return ret;
}
}
