#pragma once
#include <boost/integer.hpp>//int64_t
namespace ecto {
  namespace profile {

    unsigned long read_tsc(void);

    struct stats_type
    {
      unsigned ncalls;
      int64_t total_ticks;
    };

    struct stats_collector
    {
      int64_t start;
      stats_type& stats;

      stats_collector(stats_type& stats) : start(read_tsc()), stats(stats) 
      { 
        ++stats.ncalls;
      }

      ~stats_collector() {
        stats.total_ticks += (read_tsc() - start);
      }
    };

    double elapsed_time(const stats_type& stats);
    double period(const stats_type& stats);
    double frequency(const stats_type& stats);
 }
}
