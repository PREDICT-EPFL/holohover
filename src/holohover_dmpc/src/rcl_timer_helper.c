#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl_timer_helper.h"
#include "rcutils/stdatomic_helper.h"

// taken from rcl/src/rcl/timer.c
struct rcl_timer_impl_s
{
  // The clock providing time.
  rcl_clock_t * clock;
  // The associated context.
  rcl_context_t * context;
  // A guard condition used to wake the associated wait set, either when
  // ROSTime causes the timer to expire or when the timer is reset.
  rcl_guard_condition_t guard_condition;
  // The user supplied callback.
  atomic_uintptr_t callback;
  // This is a duration in nanoseconds.
  atomic_uint_least64_t period;
  // This is a time in nanoseconds since an unspecified time.
  atomic_int_least64_t last_call_time;
  // This is a time in nanoseconds since an unspecified time.
  atomic_int_least64_t next_call_time;
  // Credit for time elapsed before ROS time is activated or deactivated.
  atomic_int_least64_t time_credit;
  // A flag which indicates if the timer is canceled.
  atomic_bool canceled;
  // The user supplied allocator.
  rcl_allocator_t allocator;
};

int64_t get_last_call_time(const rcl_timer_t* timer)
{
    int64_t last_call_time = rcutils_atomic_load_int64_t(&timer->impl->last_call_time);
    return last_call_time;
}

void set_next_call_time(rcl_timer_t* timer, int64_t next_call_time)
{
    rcutils_atomic_store(&timer->impl->next_call_time, next_call_time);
}

#ifdef __cplusplus
}
#endif
