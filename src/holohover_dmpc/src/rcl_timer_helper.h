#ifndef RCL_TIMER_HELPER_H_
#define RCL_TIMER_HELPER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl/timer.h"

int64_t get_last_call_time(const rcl_timer_t* timer);

void set_next_call_time(rcl_timer_t* timer, int64_t next_call_time);

#ifdef __cplusplus
}
#endif

#endif  // RCL_TIME_HELPERR_H_