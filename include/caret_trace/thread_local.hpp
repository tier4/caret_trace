#include <unordered_map>
#include <thread>


void * get_callback();
void set_callback(const void *);
void unset_callback();

void * get_publisher_handle();
void set_publisher_handle(const void *);
void unset_publisher_handle();

void * get_subscription_handle();
void set_subscription_handle(const void *);
void unset_subscription_handle();

void * get_tf_buffer_core();
void set_tf_buffer_core(const void *);
void unset_tf_buffer_core();
