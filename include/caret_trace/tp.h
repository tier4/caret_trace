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

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2_caret

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "caret_trace/tp.h"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>

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

#endif /* _TP_H */

#include <lttng/tracepoint-event.h>
