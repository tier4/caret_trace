#include <unordered_map>
#include <thread>


void * get_callback();
void set_callback(const void *);
void unset_callback();

void * get_tf_buffer_core();
void set_tf_buffer_core(const void *);
void unset_tf_buffer_core();
