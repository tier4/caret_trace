#include <exception>
#include <thread>

#include "caret_trace/thread_local.hpp"

thread_local void * callback_id_;
thread_local void * tf_buffer_core_;
thread_local void * publisher_handle_;
thread_local void * subscription_handle_;
thread_local uint64_t callback_start_;
thread_local bool is_intra_process_;
thread_local uint64_t rclcpp_publish_;
thread_local uint64_t rcl_publish_;
thread_local uint64_t dds_write_;
thread_local uint64_t source_stamp_;
thread_local uint64_t message_stamp_;
thread_local uint32_t target_frame_id_;
thread_local uint32_t source_frame_id_;
thread_local void * ipc_message_;
thread_local void * ipc_buffer_;
thread_local uint64_t ipc_size_;
thread_local uint64_t lookup_transform_start_;

void * get_callback()
{
  return callback_id_;
}

void unset_callback()
{
  callback_id_ = 0;
}

void set_callback(const void * callback_id)
{
  callback_id_ = const_cast<void *>(callback_id);
}

void * get_publisher_handle()
{
  return publisher_handle_;
}

void unset_publisher_handle()
{
  publisher_handle_ = 0;
}

void set_publisher_handle(const void * publisher_handle)
{
  publisher_handle_ = const_cast<void *>(publisher_handle);
}

void * get_subscription_handle()
{
  return subscription_handle_;
}

void unset_subscription_handle()
{
  subscription_handle_ = 0;
}

void set_subscription_handle(const void * subscription_handle)
{
  subscription_handle_ = const_cast<void *>(subscription_handle);
}

void * get_tf_buffer_core()
{
  return tf_buffer_core_;
}

void unset_tf_buffer_core()
{
  tf_buffer_core_ = 0;
}

void set_tf_buffer_core(const void * tf_buffer_core)
{
  tf_buffer_core_ = const_cast<void *>(tf_buffer_core);
}

uint64_t get_callback_start() {
  return callback_start_;
}

void set_callback_start(uint64_t get_callback_start){
  callback_start_ = get_callback_start;
}

bool get_is_intra_process() {
  return is_intra_process_;
}

void set_is_intra_process(bool is_intra_process){
  is_intra_process_ = is_intra_process;
}

uint64_t get_rclcpp_publish() {
  return rclcpp_publish_;
}

void set_rclcpp_publish(uint64_t rclcpp_publish){
  rclcpp_publish_ = rclcpp_publish;
}

uint64_t get_rcl_publish() {
  return rcl_publish_;
}

void set_rcl_publish(uint64_t rcl_publish){
  rcl_publish_ = rcl_publish;
}

uint64_t get_dds_write() {
  return dds_write_;
}

void set_dds_write(uint64_t dds_write){
  dds_write_ = dds_write;
}

uint64_t get_source_stamp() {
  return source_stamp_;
}
void set_source_stamp(uint64_t stamp){
  source_stamp_ = stamp;
}

uint64_t get_message_stamp(){
  return message_stamp_;
}
void set_message_stamp(uint64_t stamp){
  message_stamp_ = stamp;
}

uint32_t get_target_frame_id(){
  return target_frame_id_;
}
void set_target_frame_id(uint32_t target_frame_id){
  target_frame_id_ = target_frame_id;
}


uint32_t get_source_frame_id(){
  return source_frame_id_;
}
void set_source_frame_id(uint32_t frame_id){
  source_frame_id_ = frame_id;
}

void * get_ipc_message() {
  return ipc_message_;
}
void set_ipc_message(const void * message) {
  ipc_message_ = const_cast<void *>(message);
}
void * get_ipc_buffer() {
  return ipc_buffer_;
}
void set_ipc_buffer(const void * buffer) {
  ipc_buffer_ = const_cast<void *>(buffer);
}
uint64_t get_ipc_size() {
  return ipc_size_;
}
void set_ipc_size(const uint64_t size) {
  ipc_size_ = size;
}
uint64_t get_lookup_transform_start() {
  return lookup_transform_start_;
}
void set_lookup_transform_start(const uint64_t lookup_transform_start) {
  lookup_transform_start_ = lookup_transform_start;
}
