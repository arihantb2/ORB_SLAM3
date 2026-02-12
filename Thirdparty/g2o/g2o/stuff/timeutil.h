#ifndef G2O_TIMEUTIL_H
#define G2O_TIMEUTIL_H

#include <sys/time.h>
#include <string>

#include "g2o_stuff_api.h"

/** @addtogroup utils **/
// @{

/** \file timeutil.h
 * \brief utility functions for handling time related stuff
 */

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
if (1) {\
  static double s_lastDone_ = (currentTime); \
  double s_now_ = (currentTime); \
  if (s_lastDone_ > s_now_) \
    s_lastDone_ = s_now_; \
  if (s_now_ - s_lastDone_ > (secs)) { \
    code; \
    s_lastDone_ = s_now_; \
  }\
} else \
  (void)0
#endif

#ifndef DO_EVERY
#define DO_EVERY(secs, code) DO_EVERY_TS(secs, g2o::get_time(), code)
#endif

#ifndef MEASURE_TIME
#define MEASURE_TIME(text, code) \
  if(1) { \
    double _start_time_ = g2o::get_time(); \
    code; \
    fprintf(stderr, "%s took %f sec\n", text, g2o::get_time() - _start_time_); \
  } else \
    (void) 0
#endif

namespace g2o {

/**
 * return the current time in seconds since 1. Jan 1970
 */
inline double get_time()
{
  struct timeval ts;
  gettimeofday(&ts, 0);
  return ts.tv_sec + ts.tv_usec*1e-6;
}

/**
 * return a monotonic increasing time (Linux: clock_gettime(CLOCK_MONOTONIC)).
 */
G2O_STUFF_API double get_monotonic_time();

/**
 * \brief Class to measure the time spent in a scope
 */
class G2O_STUFF_API ScopeTime {
  public:
    ScopeTime(const char* title);
    ~ScopeTime();
  private:
    std::string _title;
    double _startTime;
};

} // end namespace

#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME \
  g2o::ScopeTime scopeTime(__PRETTY_FUNCTION__)
#endif

// @}
#endif
