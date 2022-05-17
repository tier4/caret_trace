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

#ifndef CARET_TRACE__THREAD_LOCAL_HPP_

#include <unordered_map>
#include <thread>


void * get_callback();
void set_callback(const void * callback);
void unset_callback();

void * get_publisher_handle();
void set_publisher_handle(const void * publisher_handle);
void unset_publisher_handle();

void * get_subscription_handle();
void set_subscription_handle(const void * subscription_handle);
void unset_subscription_handle();

void * get_tf_buffer_core();
void set_tf_buffer_core(const void * tf_buffer_core);
void unset_tf_buffer_core();

uint64_t get_callback_start();
void set_callback_start(uint64_t stamp);

bool get_is_intra_process();
void set_is_intra_process(bool is_intra_process);

uint64_t get_rclcpp_publish();
void set_rclcpp_publish(uint64_t stamp);

uint64_t get_rcl_publish();
void set_rcl_publish(uint64_t stamp);

uint64_t get_dds_write();
void set_dds_write(uint64_t stamp);

uint64_t get_source_stamp();
void set_source_stamp(uint64_t stamp);

uint64_t get_message_stamp();
void set_message_stamp(uint64_t stamp);

uint32_t get_target_frame_id();
void set_target_frame_id(uint32_t frame_id_compact);

uint32_t get_source_frame_id();
void set_source_frame_id(uint32_t frame_id_compact);

void * get_ipc_message();
void set_ipc_message(const void * message);

void * get_ipc_buffer();
void set_ipc_buffer(const void * buffer);

uint64_t get_ipc_size();
void set_ipc_size(uint64_t size);

uint64_t get_lookup_transform_start();
void set_lookup_transform_start(uint64_t stamp);

#endif  // CARET_TRACE__THREAD_LOCAL_HPP_
#define CARET_TRACE__THREAD_LOCAL_HPP_
