#include <exception>
#include <thread>

#include "caret_trace/thread_local.hpp"

thread_local void * callback_id_;
thread_local void * tf_buffer_core_;

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