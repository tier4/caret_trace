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

#define LIB_CARET_VERSION "0.3"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

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
    const char *, rmw_impl_arg
  ),
  TP_FIELDS(
    ctf_string(rmw_impl, rmw_impl_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_executor,
  TP_ARGS(
    const void *, executor_addr_arg,
    const char *, executor_type_name_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, executor_addr, executor_addr_arg)
    ctf_string(executor_type_name, executor_type_name_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_static_executor,
  TP_ARGS(
    const void *, executor_addr_arg,
    const void *, entities_collector_addr_arg,
    const char *, executor_type_name_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, executor_addr, executor_addr_arg)
    ctf_integer_hex(const void *, entities_collector_addr, entities_collector_addr_arg)
    ctf_string(executor_type_name, executor_type_name_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  add_callback_group,
  TP_ARGS(
    const void *, executor_addr_arg,
    const void *, callback_group_addr_arg,
    const char *, group_type_name_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, executor_addr, executor_addr_arg)
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_string(group_type_name, group_type_name_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  add_callback_group_static_executor,
  TP_ARGS(
    const void *, entities_collector_addr_arg,
    const void *, callback_group_addr_arg,
    const char *, group_type_name_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, entities_collector_addr, entities_collector_addr_arg)
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_string(group_type_name, group_type_name_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_timer,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, timer_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_subscription,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, subscription_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, subscription_handle, subscription_handle_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_service,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, service_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, service_handle, service_handle_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  callback_group_add_client,
  TP_ARGS(
    const void *, callback_group_addr_arg,
    const void *, client_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, callback_group_addr, callback_group_addr_arg)
    ctf_integer_hex(const void *, client_handle, client_handle_arg)
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
  init_bind_transform_broadcaster,
  TP_ARGS(
    const void *, tf_broadcaster_arg,
    const void *, publisher_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_broadcaster, tf_broadcaster_arg)
    ctf_integer_hex(const void *, publisher_handle, publisher_handle_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  init_bind_transform_broadcaster_frames,
  TP_ARGS(
    const void *, tf_broadcaster_arg,
    const char *, frame_id_arg,
    const char *, child_frame_id_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_broadcaster, tf_broadcaster_arg)
    ctf_string(frame_id, frame_id_arg)
    ctf_string(child_frame_id, child_frame_id_arg)
  )
)


TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  init_tf_broadcaster_frame_id_compact,
  TP_ARGS(
    const void *, tf_broadcaster_arg,
    const char *, frame_id_arg,
    const uint32_t, frame_id_compact_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_broadcaster, tf_broadcaster_arg)
    ctf_string(frame_id, frame_id_arg)
    ctf_integer(const uint32_t, frame_id_compact, frame_id_compact_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  send_transform,
  TP_ARGS(
    const void *, tf_broadcaster_arg,
    uint64_t *, stamps_arg,
    uint32_t *, frame_ids_compact_arg,
    uint32_t *, child_frame_ids_compact_arg,
    size_t, size_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_broadcaster, tf_broadcaster_arg)
    ctf_sequence_hex(uint64_t, stamps, stamps_arg, size_t, size_arg)
    ctf_sequence(uint32_t, frame_ids_compact, frame_ids_compact_arg, size_t, size_arg)
    ctf_sequence(uint32_t, child_frame_ids_compact, child_frame_ids_compact_arg, size_t, size_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_tf_buffer,
  TP_ARGS(
    const void *, tf_buffer_arg,
    const void *, tf_buffer_core_arg,
    const void *, clock_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer, tf_buffer_arg)
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_integer_hex(const void *, clock, clock_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  init_bind_tf_buffer_core,
  TP_ARGS(
    const void *, tf_buffer_core_arg,
    const void *, callback
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_integer_hex(const void *, callback, callback)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_node_hook,
  TP_ARGS(
    const void *, node_handle_arg,
    const void *, clock_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer_hex(const void *, clock, clock_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  tf_lookup_transform_start,
  TP_ARGS(
    const void *, tf_buffer_core_arg,
    const uint64_t, target_time_arg,
    const uint32_t, frame_id_compact_arg,
    const uint32_t, child_frame_id_compact_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_integer(const uint64_t, target_time, target_time_arg)
    ctf_integer(const uint32_t, frame_id_compact, frame_id_compact_arg)
    ctf_integer(const uint32_t, child_frame_id_compact, child_frame_id_compact_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  init_tf_buffer_lookup_transform,
  TP_ARGS(
    const void *, tf_buffer_core_arg,
    const uint32_t, frame_id_compact_arg,
    const uint32_t, child_frame_id_compact_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_integer(const uint32_t, frame_id_compact, frame_id_compact_arg)
    ctf_integer(const uint32_t, child_frame_id_compact, child_frame_id_compact_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  tf_lookup_transform_end,
  TP_ARGS(
    const void *, tf_buffer_core_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  init_tf_buffer_frame_id_compact,
  TP_ARGS(
    const void *, tf_buffer_core_arg,
    const char *, frame_id_arg,
    const uint32_t, frame_id_compact_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_string(frame_id, frame_id_arg)
    ctf_integer(const uint32_t, frame_id_compact, frame_id_compact_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  tf_buffer_find_closest,
  TP_ARGS(
    const void *, tf_buffer_core_arg,
    const uint32_t, frame_id_compact_arg,
    const uint32_t, child_frame_id_compact_arg,
    const uint64_t, stamp_arg,
    const uint32_t, frame_id_compact_arg_,
    const uint32_t, child_frame_id_compact_arg_,
    const uint64_t, stamp_arg_
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_integer(const uint32_t, frame_id_compact, frame_id_compact_arg)
    ctf_integer(const uint32_t, child_frame_id_compact, child_frame_id_compact_arg)
    ctf_integer(const uint64_t, stamp, stamp_arg)
    ctf_integer(const uint32_t, frame_id_compact_, frame_id_compact_arg_)
    ctf_integer(const uint32_t, child_frame_id_compact_, child_frame_id_compact_arg_)
    ctf_integer(const uint64_t, stamp_, stamp_arg_)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  tf_set_transform,
  TP_ARGS(
    const void *, tf_buffer_core_arg,
    const uint64_t, stamp_arg,
    const uint32_t, frame_id_compact_arg,
    const uint32_t, child_frame_id_compact_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, tf_buffer_core, tf_buffer_core_arg)
    ctf_integer(const uint64_t, stamp, stamp_arg)
    ctf_integer(const uint32_t, frame_id_compact, frame_id_compact_arg)
    ctf_integer(const uint32_t, child_frame_id_compact, child_frame_id_compact_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  construct_ipm,
  TP_ARGS(
    const void *, ipm_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, ipm, ipm_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  ipm_add_publisher,
  TP_ARGS(
    const void *, ipm_arg,
    const void *, publisher_handle_arg,
    const uint32_t, id_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, ipm, ipm_arg)
    ctf_integer_hex(const void *, publisher_handle, publisher_handle_arg)
    ctf_integer(const uint32_t, id, id_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  ipm_add_subscription,
  TP_ARGS(
    const void *, ipm_arg,
    const void *, subscription_handle_arg,
    const uint32_t, id_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, ipm, ipm_arg)
    ctf_integer_hex(const void *, subscription_handle, subscription_handle_arg)
    ctf_integer(const uint32_t, id, id_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  ipm_insert_sub_id_for_pub,
  TP_ARGS(
    const void *, ipm_arg,
    const uint32_t, sub_id_arg,
    const uint32_t, pub_id_arg,
    const int, use_take_shared_method_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, ipm, ipm_arg)
    ctf_integer(const uint32_t, sub_id, sub_id_arg)
    ctf_integer(const uint32_t, pub_id, pub_id_arg)
    ctf_integer(const uint32_t, use_take_shared_method, use_take_shared_method_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_init_caret,
  TP_ARGS(),
  TP_FIELDS(
    ctf_string(lib_caret_version, LIB_CARET_VERSION)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_timer_cancel,
  TP_ARGS(
    const void *, timer_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  rcl_timer_reset,
  TP_ARGS(
    const void *, timer_handle_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, timer_handle, timer_handle_arg)
  )
)

#endif /* _TP_H */

#endif  // CARET_TRACE__TP_H_
