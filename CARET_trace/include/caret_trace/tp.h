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

// Provide fake header guard for cpplint
#undef CARET_TRACE__TP_H_
#ifndef CARET_TRACE__TP_H_
#define CARET_TRACE__TP_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2_caret

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "caret_trace/tp.h"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

// clang-format off

#include <lttng/tracepoint.h>
#include <lttng/tracepoint-event.h>


TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  on_data_available,
  TP_ARGS(
    const uint64_t, source_stamp_arg
  ),
  TP_FIELDS(
    ctf_integer(const uint64_t, source_stamp, source_stamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  dds_write,
  TP_ARGS(
    const void *, message_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, message, message_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  dds_bind_addr_to_stamp,
  TP_ARGS(
    const void *, addr_arg,
    const uint64_t, source_stamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, addr, addr_arg)
    ctf_integer(const uint64_t *, source_stamp, source_stamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  dds_bind_addr_to_addr,
  TP_ARGS(
    const void *, addr_from_arg,
    const void *, addr_to_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, addr_from, addr_from_arg)
    ctf_integer_hex(const void *, addr_to, addr_to_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rmw_implementation,
  TP_ARGS(
    const char *, rmw_impl_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_string(rmw_impl, rmw_impl_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_executor,
  TP_ARGS(
    const void *, executor_addr_arg,
    const char *, executor_type_name_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, executor_addr, executor_addr_arg)
    ctf_string(executor_type_name, executor_type_name_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_static_executor,
  TP_ARGS(
    const void *, executor_addr_arg,
    const void *, entities_collector_addr_arg,
    const char *, executor_type_name_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, executor_addr, executor_addr_arg)
    ctf_integer_hex(const void *, entities_collector_addr, entities_collector_addr_arg)
    ctf_string(executor_type_name, executor_type_name_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  add_callback_group,
  TP_ARGS(
    const void *, executor_addr_arg,
    const void *, callback_group_addr_arg,
    const char *, group_type_name_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, executor_addr, executor_addr_arg)
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_string(group_type_name, group_type_name_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  add_callback_group_static_executor,
  TP_ARGS(
    const void *, entities_collector_addr_arg,
    const void *, callback_group_addr_arg,
    const char *, group_type_name_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, entities_collector_addr, entities_collector_addr_arg)
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_string(group_type_name, group_type_name_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_timer,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, timer_handle_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_subscription,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, subscription_handle_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, subscription_handle, subscription_handle_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_service,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, service_handle_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, service_handle, service_handle_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_client,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, client_handle_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, client_handle, client_handle_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  sim_time,
  TP_ARGS(
    const uint64_t, stamp_arg
  ),
  TP_FIELDS(
    ctf_integer(const uint64_t, stamp, stamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  caret_init,
  TP_ARGS(
    int64_t, clock_offset_arg,
    const char *, distribution_arg
  ),
  TP_FIELDS(
    ctf_integer(const int64_t, clock_offset, clock_offset_arg)
    ctf_string(distribution, distribution_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_init,
  TP_ARGS(
    const void *, context_handle_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, context_handle, context_handle_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_node_init,
  TP_ARGS(
    const void *, node_handle_arg,
    const void *, rmw_handle_arg,
    const char *, node_name_arg,
    const char *, namespace_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, rmw_handle, rmw_handle_arg)
    ctf_string(node_name, node_name_arg)
    ctf_string(namespace, namespace_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_publisher_init,
  TP_ARGS(
    const void *, publisher_handle_arg,
    const void *, node_handle_arg,
    const void *, rmw_publisher_handle_arg,
    const char *, topic_name_arg,
    const size_t, queue_depth_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, publisher_handle, publisher_handle_arg)
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, rmw_publisher_handle, rmw_publisher_handle_arg)
    ctf_string(topic_name, topic_name_arg)
    ctf_integer(const size_t, queue_depth, queue_depth_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_subscription_init,
  TP_ARGS(
    const void *, subscription_handle_arg,
    const void *, node_handle_arg,
    const void *, rmw_subscription_handle_arg,
    const char *, topic_name_arg,
    const size_t, queue_depth_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, subscription_handle, subscription_handle_arg)
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, rmw_subscription_handle, rmw_subscription_handle_arg)
    ctf_string(topic_name, topic_name_arg)
    ctf_integer(const size_t, queue_depth, queue_depth_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_subscription_init,
  TP_ARGS(
    const void *, subscription_handle_arg,
    const void *, subscription_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, subscription_handle, subscription_handle_arg)
    ctf_integer_hex(const void *, subscription, subscription_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_subscription_callback_added,
  TP_ARGS(
    const void *, subscription_arg,
    const void *, callback_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, subscription, subscription_arg)
    ctf_integer_hex(const void *, callback, callback_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)


TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_timer_init,
  TP_ARGS(
    const void *, timer_handle_arg,
    int64_t, period_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
    ctf_integer(const int64_t, period, period_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_timer_callback_added,
  TP_ARGS(
    const void *, timer_handle_arg,
    const void *, callback_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
    ctf_integer_hex(const void *, callback, callback_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_timer_link_node,
  TP_ARGS(
    const void *, timer_handle_arg,
    const void *, node_handle_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_callback_register,
  TP_ARGS(
    const void *, callback_arg,
    const char *, symbol_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback, callback_arg)
    ctf_string(symbol, symbol_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)


TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_service_init,
  TP_ARGS(
    const void *, service_handle_arg,
    const void *, node_handle_arg,
    const void *, rmw_service_handle_arg,
    const char *, service_name_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, service_handle, service_handle_arg)
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, rmw_service_handle, rmw_service_handle_arg)
    ctf_string(service_name, service_name_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_service_callback_added,
  TP_ARGS(
    const void *, service_handle_arg,
    const void *, callback_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, service_handle, service_handle_arg)
    ctf_integer_hex(const void *, callback, callback_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_client_init,
  TP_ARGS(
    const void *, client_handle_arg,
    const void *, node_handle_arg,
    const void *, rmw_client_handle_arg,
    const char *, service_name_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, client_handle, client_handle_arg)
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, rmw_client_handle, rmw_client_handle_arg)
    ctf_string(service_name, service_name_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_lifecycle_state_machine_init,
  TP_ARGS(
    const void *, node_handle_arg,
    const void *, state_machine_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, state_machine, state_machine_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_ipb_to_subscription,
  TP_ARGS(
    const void *, ipb_arg,
    const void *, subscription_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, ipb, ipb_arg)
    ctf_integer_hex(const void *, subscription, subscription_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_buffer_to_ipb,
  TP_ARGS(
    const void *, buffer_arg,
    const void *, ipb_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, buffer, buffer_arg)
    ctf_integer_hex(const void *, ipb, ipb_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rclcpp_construct_ring_buffer,
  TP_ARGS(
    const void *, buffer_arg,
    const uint64_t, capacity_arg,
    int64_t, init_timestamp_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, buffer, buffer_arg)
    ctf_integer(const uint64_t, capacity, capacity_arg)
    ctf_integer(const int64_t, init_timestamp, init_timestamp_arg)
  )
)

// clang-format on

#endif /* _TP_H */

#endif  // CARET_TRACE__TP_H_
